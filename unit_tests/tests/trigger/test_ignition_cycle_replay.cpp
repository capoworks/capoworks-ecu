// Diagnostic extension of the branch replay: 15-degree TDC-offset steps include
// both tooth-boundary and halfway-between-tooth targets. Production code is unchanged.
// Future timestamps are used only by the offline physical-phase interpolation observer.
// Decoder synchronization/PMS alignment and hardware interrupt latency are outside this replay.
#include "pch.h"
#include "spark_logic.h"
#include "engine_math.h"
#include "idle_thread.h"
#include "trigger_universal.h"
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
using ::testing::_;
#if __has_include("ignition_cycle_profile.h")
#define HAS_CYCLE_PROFILE 1
#else
#define HAS_CYCLE_PROFILE 0
#endif
#if __has_include("ignition_retarget.h")
#define HAS_RETARGET 1
#else
#define HAS_RETARGET 0
#endif
namespace {
struct IdleTimingInputProxy final : IIdleController {
	IdleController& real;
	explicit IdleTimingInputProxy(IdleController& controller)
		: real(controller) {}
	float getCrankingOpenLoop(float clt) const override {
		return real.getCrankingOpenLoop(clt);
	}
	float getRunningOpenLoop(float rpm, float clt, SensorResult tps) override {
		return real.getRunningOpenLoop(rpm, clt, tps);
	}
	float getOpenLoop(Phase p, float r, float c, SensorResult t, float f) override {
		return real.getOpenLoop(p, r, c, t, f);
	}
	float getClosedLoop(Phase p, float r, float rate, float target) override {
		return real.getClosedLoop(p, r, rate, target);
	}
	bool isIdlingOrTaper() const override {
		return true;
	}
	float getIdleTimingAdjustment(float rpm, float rpmRate) override {
		return real.getIdleTimingAdjustment(rpm, rpmRate, 1200, Phase::Idling);
	}
};

struct Tooth {
	double time;
	double angle;
};
std::vector<Tooth> readProfile(const std::string& name) {
	std::vector<Tooth> teeth;
	const char* root = std::getenv("FOME_REPLAY_DIR");
	if (!root) {
		return teeth;
	}
	std::ifstream input(std::string(root) + "/profiles/" + name + ".csv");
	std::string line;
	std::getline(input, line);
	while (std::getline(input, line)) {
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream row(line);
		Tooth tooth;
		if (row >> tooth.time >> tooth.angle) {
			tooth.time = std::round(tooth.time);
			teeth.push_back(tooth);
		}
	}
	return teeth;
}
void runReplay(
		const std::string& profile,
		float nominalRpm,
		bool instant,
		int phase,
		float gain,
		int fastPhaseUs = 0,
		float dwellMs = 2.86f) {
	const auto teeth = readProfile(profile);
	ASSERT_GT(teeth.size(), 100u);
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (auto& pin : enginePins.coils) {
		pin.setLow();
		ASSERT_FALSE(pin.getLogicValue());
	}
	for (auto& pin : enginePins.trailingCoils) {
		pin.setLow();
		ASSERT_FALSE(pin.getLogicValue());
	}
	setCylinderCount(2);
	engineConfiguration->firingOrder = FO_1_2;
	engineConfiguration->timing_offset_cylinder[0] = phase;
	engineConfiguration->timing_offset_cylinder[1] = phase - 60;
	engineConfiguration->alwaysInstantRpm = false;
#if HAS_CYCLE_PROFILE
	engineConfiguration->ignitionCycleProfile = instant;
	engineConfiguration->ignitionTimeBudget = std::getenv("FOME_TIME_BUDGET") != nullptr;
#endif
	engineConfiguration->instantRpmRange = 90;
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->useIdleTimingPidControl = true;
	engineConfiguration->idleTimingPid = {};
	engineConfiguration->idleTimingPid.pFactor = gain;
	engineConfiguration->idleTimingPid.minValue = -6;
	engineConfiguration->idleTimingPid.maxValue = 12;
	engineConfiguration->minimumIgnitionTiming = -20;
	engineConfiguration->maximumIgnitionTiming = 40;
	engineConfiguration->useSeparateAdvanceForIdle = false;
	engineConfiguration->multisparkEnable = false;
	setTable(config->ignitionTable, 10);
	setTable(config->ignitionIatCorrTable, 0);
	setArrayValues(config->cltTimingExtra, 0);
	setArrayValues(config->sparkDwellValues, dwellMs);
	setArrayValues(config->dwellVoltageCorrValues, 1);
	for (auto& trim : config->ignTrims) {
		setTable(trim.table, 0);
	}
	prepareOutputSignals();
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 0);
	Sensor::setMockValue(SensorType::BatteryVoltage, 13.5f);
	auto& realIdle = engine->module<IdleController>().unmock();
	realIdle.init();
	IdleTimingInputProxy timingInput(realIdle);
	engine->engineModules.get<IdleController>().set(&timingInput);
	engine->rpmCalculator.setRpmValue(nominalRpm);
	engine->rpmCalculator.rpmRate = 0;

	auto& tc = engine->triggerCentral;
	initializeSkippedToothTrigger(&tc.triggerShape, 36, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	// Complete the same geometry metadata used by the production initializer.
	ASSERT_EQ(tc.triggerShape.primaryTeethPerCycle, 0u); // documents original fixture defect
	tc.triggerShape.calculateExpectedEventCounts();
	ASSERT_EQ(tc.triggerShape.primaryTeethPerCycle, 68u);
	ASSERT_EQ(tc.triggerShape.getLength(), 136u);
	tc.engineCycleEventCount = 136;
	tc.triggerState.setNeedsDisambiguation(true, true);
	tc.triggerState.syncEnginePhase(2, 0, 720);
	ASSERT_EQ(getCurrentIgnitionMode(), IM_INDIVIDUAL_COILS);
	for (int k = 0; k < 68; k++) {
		float angle = (k / 34) * 360 + (k % 34) * 10;
		tc.triggerFormDetails.eventAngles[k * 2] = angle;
		tc.triggerFormDetails.eventAngles[k * 2 + 1] = angle;
	}
	tc.instantRpm.resetInstantRpm();
#if HAS_CYCLE_PROFILE
	tc.instantRpm.ignitionProfile.configure(tc.triggerShape, tc.triggerFormDetails);
#endif
	// Establish a running fixture; subsequent RPM updates use the upstream callback.
	setTimeNowUs(teeth.front().time);
	engine->rpmCalculator.lastTdcTimer.reset();
	tc.m_lastEventTimer.reset();
	engine->rpmCalculator.setRpmValue(nominalRpm);
	engine->periodicFastCallback();
	auto phaseAt = [&](double time) {
		auto it = std::upper_bound(
				teeth.begin(), teeth.end(), time, [](double t, const Tooth& tooth) { return t < tooth.time; });
		if (it == teeth.begin()) {
			return teeth.front().angle;
		}
		if (it == teeth.end()) {
			return teeth.back().angle;
		}
		auto prev = it - 1;
		return prev->angle + (time - prev->time) / (it->time - prev->time) * (it->angle - prev->angle);
	};
	const char* root = std::getenv("FOME_REPLAY_DIR");
	std::string name = profile + "_instant" + std::to_string(instant) + "_phi" + std::to_string(phase) + "_gain" +
					   std::to_string(gain) + "_fast" + std::to_string(fastPhaseUs);
	std::ofstream trace(std::string(root) + "/" + std::getenv("FOME_REPLAY_VERSION") + "-replay/" + name + ".csv");
	ASSERT_TRUE(trace.good());
	trace << "time_us,unwrapped_angle_deg,cylinder,actual_btdc,requested_btdc,published_btdc,scheduling_error_deg,"
			 "dwell_ratio,fallback,instant_rpm,scheduling_rpm,min_dwell_delays,coil_charged,charge_duration_us,limited,"
			 "duplicate_charges,profile_available,predicted_conversions,legacy_conversions,decision_requested_btdc,"
			 "decision_status\n"
		  << std::setprecision(12);
	std::ofstream toothTrace;
	if (phase == 0 && gain == 0) {
		toothTrace.open(
				std::string(root) + "/" + std::getenv("FOME_REPLAY_VERSION") + "-replay/" + name + "_teeth.csv");
		toothTrace << "j,time_us,span_deg,observed_next_us,legacy_next_us,raw_next_us,guarded_next_us,valid\n"
				   << std::setprecision(12);
	}
	std::ofstream plannerTrace;
	if (std::getenv("FOME_REPLAY_TRACE")) {
		plannerTrace.open(
				std::string(root) + "/" + std::getenv("FOME_REPLAY_VERSION") + "-replay/" + name + "_planner.csv");
		plannerTrace << "time_us,angle_deg,cylinder,published_btdc,raw_target,before_state,state,planner_cycle,planned_"
						"cycle,planned_valid,planned_angle,expired_before,expired,high\n"
					 << std::setprecision(12);
	}
#if HAS_RETARGET
	std::ofstream decisions(
			std::string(root) + "/" + std::getenv("FOME_REPLAY_VERSION") + "-replay/" + name + "_decisions.csv");
	decisions << "time_us,angle_deg,cylinder,state,tdc_cycle,requested_btdc,applied_btdc,earliest_btdc,latest_btdc,"
				 "status\n"
			  << std::setprecision(12);
	engine->onIgnitionRetarget = [&](int c, float request, float applied, float earliest, float latest, int status) {
		const auto& e = engine->ignitionEvents.elements[c];
		const float tdc = engine->cylinders[c].getAngleOffset();
		decisions << getTimeNowUs() << ',' << phaseAt(getTimeNowUs()) << ',' << c + 1 << ','
				  << static_cast<int>(e.state) << ','
				  << static_cast<int>(e.candidateTdcCycle) + static_cast<int>(std::floor(teeth.front().angle / 720))
				  << ',' << tdc - request << ',' << tdc - applied << ',' << tdc - earliest << ',' << tdc - latest << ','
				  << status << '\n';
	};
#endif
	bool available = false;
	int counts[2] = {};
	int delays[2] = {};
	int fallback = 0;
	bool charged[2] = {};
	double chargeUs[2] = {};
	int duplicateCharges[2] = {};
	const double warmupAngle = teeth.front().angle + 2160;
	unsigned predictedConversions = 0, legacyConversions = 0;
#if HAS_CYCLE_PROFILE
	engine->onIgnitionTiming = [&](bool used) {
		if (phaseAt(getTimeNowUs()) >= warmupAngle) {
			if (used) {
				predictedConversions++;
			} else {
				legacyConversions++;
			}
		}
	};
#endif
	engine->onIgnitionEvent = [&](IgnitionContext ctx, bool charging) {
		if (charging) {
			duplicateCharges[ctx.eventIndex] += charged[ctx.eventIndex];
			// Repeated setHigh resets the firmware dwell timer, but the physical coil
			// has remained energized since the first charge callback.
			if (!charged[ctx.eventIndex]) {
				chargeUs[ctx.eventIndex] = getTimeNowUs();
			}
			charged[ctx.eventIndex] = true;
			return;
		}
		bool pinCharged = false;
		forEachSetBit(ctx.outputsMask(), [&](size_t idx) { pinCharged |= enginePins.coils[idx].getLogicValue(); });
		bool wasCharged = charged[ctx.eventIndex] && pinCharged;
		charged[ctx.eventIndex] = false;
		double actualAngle = phaseAt(getTimeNowUs());
		if (actualAngle < warmupAngle || getTimeNowUs() >= teeth.back().time) {
			return;
		}
		int cylinder = ctx.eventIndex;
		auto& event = engine->ignitionEvents.elements[cylinder];
		double tdc = engine->cylinders[cylinder].getAngleOffset();
		double actual = std::remainder(tdc - actualAngle, 720.0);
		double request = std::remainder(tdc - tc.toEngPhase(event.sparkEvent.eventPhase).angle, 720.0);
		double error = std::remainder(actualAngle - tc.toEngPhase(event.sparkEvent.eventPhase).angle, 720.0);
		double ratio = event.actualDwellTimer.getElapsedSeconds(getTimeNowNt()) * 1000 / event.sparkDwell;
		trace << getTimeNowUs() << ',' << actualAngle << ',' << cylinder + 1 << ',' << actual << ',' << request << ','
			  << engine->cylinders[cylinder].getIgnitionTimingBtdc() << ',' << error << ',' << ratio << ','
			  << ctx.isOverdwellProtect << ',' << tc.instantRpm.getInstantRpm() << ','
			  << engine->rpmCalculator.getCachedRpm() << ',' << delays[cylinder] << ',' << wasCharged << ','
			  << (wasCharged ? getTimeNowUs() - chargeUs[cylinder] : 0) << ',' << event.wasSparkLimited << ','
			  << duplicateCharges[cylinder] << ','
#if HAS_CYCLE_PROFILE
			  << available
#else
			  << 0
#endif
			  << ',' << predictedConversions << ',' << legacyConversions
#if HAS_RETARGET
			  << ',' << (event.plannedByTime ? tdc - event.requestedSparkAngle : request) << ','
			  << static_cast<int>(event.retargetStatus)
#else
			  << ',' << request << ',' << 0
#endif
			  << '\n';
		if (profile.find("uniform") == 0 && wasCharged) {
			EXPECT_LT(std::abs(error), 0.061);
		}

		counts[cylinder]++;
		fallback += ctx.isOverdwellProtect;
	};
	auto advanceEvents = [&](int target) {
		while (auto* head = engine->scheduler.getHead()) {
			if (head->momentX > target) {
				break;
			}
			setTimeNowUs(head->momentX);
			bool firingTimer = false;
			for (int c = 0; c < 2; c++) {
				firingTimer |= head == &engine->ignitionEvents.elements[c].sparkEvent.scheduling;
			}
			if (firingTimer) {
				IgnitionContext ctx;
				ctx._pad = head->action.getArgument();
				auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
#if HAS_CYCLE_PROFILE
				const float dwell = event.occurrenceDwell;
				const bool ownsCharge = event.state == IgnitionOccurrenceState::Charging;
#else
				const float dwell = event.sparkDwell;
				const bool ownsCharge = true;
#endif
				if (ownsCharge && phaseAt(getTimeNowUs()) >= warmupAngle && !ctx.isOverdwellProtect &&
					event.actualDwellTimer.getElapsedSeconds(getTimeNowNt()) * 1000 < 0.8f * dwell) {
					delays[ctx.eventIndex]++;
				}
			}
			engine->scheduler.executeAll(getTimeNowUs());
		}
		setTimeNowUs(target);
	};
	int nextFast = teeth.front().time + fastPhaseUs;
	for (size_t i = 0; i + 1 < teeth.size(); i++) {
		int time = teeth[i].time;
		while (nextFast <= time) {
			advanceEvents(nextFast);
			engine->periodicFastCallback();
			nextFast += 4000;
		}
		advanceEvents(time);
		float current = std::fmod(teeth[i].angle + 720000, 720.0);
		float next = std::fmod(teeth[i + 1].angle + 720000, 720.0);
		int index = (int(current) / 360 * 34 + int(current) % 360 / 10) * 2;
		ASSERT_LT(index, 136);
		ASSERT_NEAR(tc.triggerFormDetails.eventAngles[index], current, 0.01);
		tc.m_lastEventTimer.reset();
		EnginePhaseInfo info{
				getTimeNowNt(), {current}, {next}, tc.toEngPhase(TrgPhase{current}), tc.toEngPhase(TrgPhase{next})};
		rpmShaftPositionCallback(index, info);
		if (i >= 71) {
			ASSERT_FLOAT_EQ(engine->rpmCalculator.getCachedRpm(), tc.instantRpm.getInstantRpm());
		}
#if HAS_CYCLE_PROFILE
		available = bool(tc.instantRpm.ignitionProfile.getDelayNt(info, 0));
#endif
		if (toothTrace.good() && toothTrace.is_open()) {
			float span = next - current;
			if (span < 0) {
				span += 720;
			}
			double raw = 0;
			if (i >= 71) {
				raw = (teeth[i - 67].time - teeth[i - 68].time) * (teeth[i].time - teeth[i - 3].time) /
					  (teeth[i - 68].time - teeth[i - 71].time);
			}
			float guarded = 0;
#if HAS_CYCLE_PROFILE
			if (auto d = tc.instantRpm.ignitionProfile.getDelayNt(info, span / 2)) {
				guarded = 2 * d.Value / US_TO_NT_MULTIPLIER;
			}
#endif
			toothTrace << i << ',' << time << ',' << span << ',' << teeth[i + 1].time - teeth[i].time << ','
					   << span * engine->rpmCalculator.oneDegreeUs << ',' << raw << ',' << guarded << ',' << available
					   << '\n';
		}
		float rpm = engine->rpmCalculator.getCachedRpm();
		int beforeState[2] = {};
		unsigned beforeExpired[2] = {};
		for (int c = 0; c < 2; c++) {
			beforeState[c] = static_cast<int>(engine->ignitionEvents.elements[c].state);
			beforeExpired[c] = engine->ignitionEvents.elements[c].expiredTargetCount;
		}
		engine->module<TriggerScheduler>()->onEnginePhase(rpm, info);
		onTriggerEventSparkLogic(info);
		if (plannerTrace.is_open()) {
			for (int c = 0; c < 2; c++) {
				const auto& e = engine->ignitionEvents.elements[c];
				plannerTrace << time << ',' << teeth[i].angle << ',' << c + 1 << ','
							 << engine->cylinders[c].getIgnitionTimingBtdc() << ','
							 << engine->cylinders[c].getSparkAngle(0) << ',' << beforeState[c] << ','
							 << static_cast<int>(e.state) << ',' << engine->ignitionEvents.plannerCycle << ','
							 << e.plannedTdcCycle << ',' << e.plannedCycleValid << ',' << e.plannedSparkAngle << ','
							 << beforeExpired[c] << ',' << e.expiredTargetCount << ','
							 << enginePins.coils[c].getLogicValue() << '\n';
			}
		}
	}
	advanceEvents(teeth.back().time);
	EXPECT_FALSE(hasFirmwareError());
	EXPECT_GT(counts[0], 5);
	EXPECT_GT(counts[1], 5);
	printf("REPLAY %s sparks=%d/%d minDwell=%d/%d fallback=%d\n",
		   name.c_str(),
		   counts[0],
		   counts[1],
		   delays[0],
		   delays[1],
		   fallback);
	std::ofstream physical(
			std::string(root) + "/" + std::getenv("FOME_REPLAY_VERSION") + "-replay/" + name + "_physical.csv");
	physical << "cylinder,end_high,censored_us,hard_guard,missed_charge,contention,expired_target\n";
	for (int c = 0; c < 2; c++) {
		const bool high = enginePins.coils[c].getLogicValue();
		physical << c + 1 << ',' << high << ',' << (high ? getTimeNowUs() - chargeUs[c] : 0);
#if HAS_CYCLE_PROFILE
		const auto& event = engine->ignitionEvents.elements[c];
		physical << ',' << event.hardGuardCount << ',' << event.missedChargeCount << ',' << event.contentionCount << ','
				 << event.expiredTargetCount;
#else
		physical << ",0,0,0,0";
#endif
		physical << '\n';
	}
	engine->onIgnitionEvent = {};
#if HAS_RETARGET
	engine->onIgnitionRetarget = {};
#endif
#if HAS_CYCLE_PROFILE
	engine->onIgnitionTiming = {};
#endif
	engine->engineModules.get<IdleController>().set(nullptr);
}
} // namespace
TEST(IgnitionCycleReplay, FixedAdvanceAndDynamicIdle) {
	const char* replayDir = std::getenv("FOME_REPLAY_DIR");
	const char* replayVersion = std::getenv("FOME_REPLAY_VERSION");
	if (!replayDir || !*replayDir || !replayVersion || !*replayVersion) {
		GTEST_SKIP() << "Optional diagnostic: set nonempty FOME_REPLAY_DIR and FOME_REPLAY_VERSION";
	}
	for (bool enabled : {false, true}) {
#if !HAS_CYCLE_PROFILE
		if (enabled) {
			continue;
		}
#endif
		for (int rpm : {1200, 5000, 10000}) {
			runReplay("uniform" + std::to_string(rpm), rpm, enabled, 0, 0);
			for (int phase = 0; phase < 360; phase += 15) {
				runReplay("real" + std::to_string(rpm), rpm, enabled, phase, 0);
			}
		}
		for (int phase = 0; phase < 360; phase += 15) {
			for (int fast : {0, 1000, 2000, 3000}) {
				runReplay("real1200", 1200, enabled, phase, 0.1142f, fast);
			}
		}
		for (const std::string profile :
			 {"original",
			  "acceleration",
			  "deceleration",
			  "amplitude",
			  "disturbance",
			  "misfire",
			  "step_up",
			  "step_down"}) {
			for (int phase = 0; phase < 360; phase += 15) {
				runReplay(profile, profile == "original" ? 435 : 1200, enabled, phase, 0);
			}
		}
	}
}
