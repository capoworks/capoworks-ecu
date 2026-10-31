/**
 * @file test_ve_analyze_injection_events.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "airmass_injection_state.h"
#include "fuel_schedule.h"
#include "injector_model.h"

#include <algorithm>
#include <cmath>

namespace {
using ve_analyze_fuel_events::FuelEventReason;

constexpr mass_t UnscaledCycleMass = 0.01f;

class ControlledWallModel final : public IWallFuelController {
public:
	bool getEnable() const override {
		return Enabled;
	}

	float getAlpha() const override {
		return Alpha;
	}

	float getBeta() const override {
		return Beta;
	}

	bool Enabled = false;
	float Alpha = 0.5f;
	float Beta = 0.25f;
};

class InjectionEventHarness {
public:
	InjectionEventHarness()
		: eth(engine_type_e::TEST_ENGINE) {
		Sensor::setMockValue(SensorType::Clt, 27);
		Sensor::setMockValue(SensorType::Iat, 27);

		setCylinderCount(4);
		engineConfiguration->fuelAlgorithm = LM_MOCK;
		engineConfiguration->injectionMode = IM_SEQUENTIAL;
		engineConfiguration->crankingInjectionMode = IM_SEQUENTIAL;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->engineState.injectionOffset = 0;
		engine->engineState.shouldUpdateInjectionTiming = true;

		for (size_t i = 0; i < 4; i++) {
			engineConfiguration->cylinderBankSelect[i] = i < 2 ? 0 : 1;
		}

		ON_CALL(primary, getInjectionDuration(::testing::_)).WillByDefault(::testing::Invoke([](float mass) -> float {
			if (!std::isfinite(mass) || mass <= 0) {
				return 0.0f;
			}
			return std::clamp(mass * 1000.0f, 0.02f, 20.0f);
		}));
		ON_CALL(secondary, getInjectionDuration(::testing::_)).WillByDefault(::testing::Invoke([](float mass) -> float {
			if (!std::isfinite(mass) || mass <= 0) {
				return 0.0f;
			}
			return std::clamp(mass * 1000.0f, 0.02f, 20.0f);
		}));
		engine->module<InjectorModelPrimary>().set(&primary);
		engine->module<InjectorModelSecondary>().set(&secondary);
		engine->module<WallFuelController>().set(&wall);
	}

	void seed(uint8_t cylinder, mass_t mass = UnscaledCycleMass, uint32_t generation = 0, bool valid = true) {
		CylinderFuelProvenance provenance;
		provenance.PreAeBaseFuelMass = mass;
		provenance.AppliedAeFuelMass = 0;
		provenance.CalibrationGeneration = generation == 0 ? engine->airmassCalibration.Generation : generation;
		provenance.Valid = valid;
		engine->cylinders[cylinder].setInjectionMassAndProvenance(mass, provenance);
	}

	void seedAll() {
		for (uint8_t i = 0; i < engine->engineState.cylinderCount; i++) {
			seed(i);
		}
	}

	InjectionEvent makeEvent(uint8_t firingIndex, injection_mode_e mode = IM_SEQUENTIAL) {
		engineConfiguration->injectionMode = mode;
		engineConfiguration->crankingInjectionMode = mode;
		engine->triggerCentral.triggerState.syncEnginePhase(2, 0, engine->engineState.engineCycle);

		InjectionEvent event;
		event.setIndex(firingIndex);
		EXPECT_TRUE(event.update());
		event.injectionStartAngle = 5;
		return event;
	}

	void fire(InjectionEvent& event) {
		event.onTriggerTooth({getTimeNowNt(), 0, 0, 0, 10});
	}

	void configureFuelSchedule() {
		for (uint8_t i = 0; i < engine->engineState.cylinderCount; i++) {
			auto& event = engine->injectionEvents.elements[i];
			event.setIndex(i);
			ASSERT_TRUE(event.update());
			event.injectionStartAngle = 5;
		}
		engine->injectionEvents.isReady = true;
	}

	void fireFuelSchedule() {
		for (uint8_t i = 0; i < engine->engineState.cylinderCount; i++) {
			// The completion callback may rebuild the real injection angle between teeth.
			engine->injectionEvents.elements[i].injectionStartAngle = 5;
		}
		engine->injectionEvents.onTriggerTooth({getTimeNowNt(), 0, 0, 0, 10});
	}

	VeAnalyzeFuelEventSnapshot snapshot() {
		chibios_rt::CriticalSectionLocker csl;
		return engine->engineState.snapshotVeAnalyzeFuelEventsLocked(getTimeNowNt());
	}

	void exportOutput() {
		chibios_rt::CriticalSectionLocker csl;
		engine->engineState.exportVeAnalyzeFuelEventsLocked(engine->outputChannels);
	}

	NiceMock<MockInjectorModel2> primary;
	NiceMock<MockInjectorModel2> secondary;
	ControlledWallModel wall;
	EngineTestHelper eth;
};

void establishStagedEvent(InjectionEventHarness& harness) {
	harness.seed(0);
	harness.engine->engineState.injectionStage2Fraction = 0.2f;
	auto event = harness.makeEvent(0);
	harness.fire(event);

	auto evidence = harness.snapshot();
	ASSERT_EQ(1u, evidence.AcceptedEventCount);
	ASSERT_TRUE(evidence.LastEvent.Stage1Supplied);
	ASSERT_TRUE(evidence.LastEvent.Stage2Supplied);
}

enum class UnsupportedCase {
	Split,
	SinglePoint,
	SuppressedStage2,
};

void expectUnsupportedAfterGoodEvidence(UnsupportedCase unsupportedCase) {
	InjectionEventHarness harness;
	establishStagedEvent(harness);
	harness.eth.moveTimeForwardUs(1000);

	const size_t queueBefore = harness.engine->scheduler.size();
	harness.seed(0);
	harness.engine->engineState.injectionStage2Fraction = 0.2f;

	injection_mode_e mode = IM_SEQUENTIAL;
	switch (unsupportedCase) {
		case UnsupportedCase::Split:
			harness.engine->engineState.doSplitInjection = true;
			break;
		case UnsupportedCase::SinglePoint:
			mode = IM_SINGLE_POINT;
			harness.engine->engineState.injectionStage2Fraction = 0;
			break;
		case UnsupportedCase::SuppressedStage2:
			harness.engine->engineState.injectionStage2Fraction = 0.002f;
			break;
	}

	auto attempt = harness.makeEvent(0, mode);
	harness.fire(attempt);

	EXPECT_EQ(queueBefore + 2, harness.engine->scheduler.size())
			<< "the real fueling scheduler must retain their existing pulse behavior";

	auto evidence = harness.snapshot();
	EXPECT_EQ(2u, evidence.AttemptedEventCount);
	EXPECT_EQ(1u, evidence.AcceptedEventCount) << "unsupported evidence cannot add an accepted attempt";
	EXPECT_EQ(0u, evidence.RejectedEventCount) << "the scheduler accepted, but the evidence is incomplete";
	EXPECT_EQ(1u, evidence.InvalidEvidenceEventCount);
	EXPECT_NE(0u, evidence.rejectedEventCount(FuelEventReason::IntegrationUnsupported));
	EXPECT_TRUE(evidence.LastEvent.SupersededByRejection);
	EXPECT_TRUE(evidence.Cylinders[0].SupersededByRejection);

	harness.exportOutput();
	EXPECT_EQ(1u, harness.engine->outputChannels.veAnalyzeFuelEventAcceptedCount);
	EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelCylinderScheduledMg[0]);
	EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelBankScheduledMg[0]);
	EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelEventLastScheduledMg);
}
} // namespace

TEST(VeAnalyzeInjectionEvents, SequentialModeScalesOnceAndSchedulesRealCallbacks) {
	InjectionEventHarness harness;
	harness.seedAll();
	const size_t queueBefore = harness.engine->scheduler.size();

	auto event = harness.makeEvent(1, IM_SEQUENTIAL);
	const uint16_t expectedCylinderMask = static_cast<uint16_t>(1u << 2);
	harness.fire(event);

	auto evidence = harness.snapshot();
	harness.exportOutput();
	EXPECT_EQ(queueBefore + 2, harness.engine->scheduler.size());
	EXPECT_EQ(1u, evidence.AcceptedEventCount);
	EXPECT_EQ(1u, harness.engine->outputChannels.veAnalyzeFuelEventAcceptedCount);
	EXPECT_EQ(expectedCylinderMask, evidence.LastEvent.AffectedCylinderMask);
	EXPECT_EQ(2, evidence.LastEvent.AffectedBankMask);
	EXPECT_FLOAT_EQ(10.0f, evidence.LastEvent.CalculatedBaseMassMilligrams);
	EXPECT_FLOAT_EQ(10.0f, evidence.LastEvent.ScheduledCommandMassMilligrams);
}

TEST(VeAnalyzeInjectionEvents, BatchModeScalesOnceAndMasksItsPhysicalPair) {
	InjectionEventHarness harness;
	harness.seed(0);
	const size_t queueBefore = harness.engine->scheduler.size();

	auto event = harness.makeEvent(0, IM_BATCH);
	harness.fire(event);

	auto evidence = harness.snapshot();
	EXPECT_EQ(queueBefore + 2, harness.engine->scheduler.size());
	EXPECT_EQ(1u, evidence.AcceptedEventCount);
	EXPECT_EQ(0x9u, evidence.LastEvent.AffectedCylinderMask);
	EXPECT_EQ(0x3u, evidence.LastEvent.AffectedBankMask);
	EXPECT_FLOAT_EQ(5.0f, evidence.LastEvent.CalculatedBaseMassMilligrams);
	EXPECT_FLOAT_EQ(5.0f, evidence.LastEvent.ScheduledCommandMassMilligrams);
}

TEST(VeAnalyzeInjectionEvents, SimultaneousModeScalesOnceAndMasksConfiguredEngine) {
	InjectionEventHarness harness;
	harness.seed(0);
	const size_t queueBefore = harness.engine->scheduler.size();

	auto event = harness.makeEvent(0, IM_SIMULTANEOUS);
	harness.fire(event);

	auto evidence = harness.snapshot();
	EXPECT_EQ(queueBefore + 2, harness.engine->scheduler.size());
	EXPECT_EQ(1u, evidence.AcceptedEventCount);
	EXPECT_EQ(0xFu, evidence.LastEvent.AffectedCylinderMask);
	EXPECT_EQ(0x3u, evidence.LastEvent.AffectedBankMask);
	EXPECT_FLOAT_EQ(2.5f, evidence.LastEvent.CalculatedBaseMassMilligrams);
	EXPECT_FLOAT_EQ(2.5f, evidence.LastEvent.ScheduledCommandMassMilligrams);
}

TEST(VeAnalyzeInjectionEvents, CompleteStagedMassRequiresAndReportsBothScheduledPulses) {
	InjectionEventHarness harness;
	harness.seed(0);
	harness.engine->engineState.injectionStage2Fraction = 0.2f;
	const size_t queueBefore = harness.engine->scheduler.size();

	auto event = harness.makeEvent(0);
	harness.fire(event);

	auto evidence = harness.snapshot();
	EXPECT_EQ(queueBefore + 3, harness.engine->scheduler.size());
	EXPECT_EQ(1u, evidence.AcceptedEventCount);
	EXPECT_TRUE(evidence.LastEvent.SchedulerAccepted);
	EXPECT_TRUE(evidence.LastEvent.Stage1Supplied);
	EXPECT_TRUE(evidence.LastEvent.Stage2Supplied);
	EXPECT_NEAR(8.0f, evidence.LastEvent.Stage1MassMilligrams, 1e-5f);
	EXPECT_NEAR(2.0f, evidence.LastEvent.Stage2MassMilligrams, 1e-5f);
	EXPECT_NEAR(10.0f, evidence.LastEvent.Stage1MassMilligrams + evidence.LastEvent.Stage2MassMilligrams, 1e-5f);
	EXPECT_FLOAT_EQ(10.0f, evidence.LastEvent.ScheduledCommandMassMilligrams);
}

TEST(VeAnalyzeInjectionEvents, SplitSinglePointAndSuppressedStage2RemainUnsupported) {
	expectUnsupportedAfterGoodEvidence(UnsupportedCase::Split);
	expectUnsupportedAfterGoodEvidence(UnsupportedCase::SinglePoint);
	expectUnsupportedAfterGoodEvidence(UnsupportedCase::SuppressedStage2);
}

TEST(VeAnalyzeInjectionEvents, TooShortAndZeroPulsesAreRejectedWithoutSchedulerWork) {
	InjectionEventHarness harness;
	const size_t queueBefore = harness.engine->scheduler.size();

	harness.seed(0, 0.00001f);
	auto shortEvent = harness.makeEvent(0);
	harness.fire(shortEvent);

	harness.eth.moveTimeForwardUs(1000);
	harness.seed(0, 0);
	auto zeroEvent = harness.makeEvent(0);
	harness.fire(zeroEvent);

	auto evidence = harness.snapshot();
	EXPECT_EQ(queueBefore, harness.engine->scheduler.size());
	EXPECT_EQ(2u, evidence.AttemptedEventCount);
	EXPECT_EQ(0u, evidence.AcceptedEventCount);
	EXPECT_EQ(2u, evidence.RejectedEventCount);
	EXPECT_NE(0u, evidence.rejectedEventCount(FuelEventReason::SchedulingRejectedRecovery));
	EXPECT_TRUE(evidence.hasReason(FuelEventReason::ZeroCommandedRecovery));
	EXPECT_FLOAT_EQ(0, evidence.LastEvent.ScheduledCommandMassMilligrams);
}

TEST(VeAnalyzeInjectionEvents, FullSchedulerRejectsActualEventAndExportsAttemptedMassOnly) {
	InjectionEventHarness harness;
	for (size_t i = 0; i < 64; i++) {
		harness.engine->scheduler.schedule("fill", nullptr, getTimeNowNt() + US2NT(10000), {+[](void*) {}, nullptr});
	}
	ASSERT_EQ(64u, harness.engine->scheduler.size());

	harness.seed(0);
	auto event = harness.makeEvent(0);
	harness.fire(event);

	auto evidence = harness.snapshot();
	EXPECT_EQ(64u, harness.engine->scheduler.size());
	EXPECT_EQ(1u, evidence.AttemptedEventCount);
	EXPECT_EQ(0u, evidence.AcceptedEventCount);
	EXPECT_EQ(1u, evidence.RejectedEventCount);
	EXPECT_FALSE(evidence.LastEvent.SchedulerAccepted);
	EXPECT_FLOAT_EQ(10.0f, evidence.LastEvent.AttemptedFinalCommandedMassMilligrams);
	EXPECT_FLOAT_EQ(0, evidence.LastEvent.ScheduledCommandMassMilligrams);
	EXPECT_EQ(0u, harness.engine->airmassInjectionState.pendingCallbacks());
}

TEST(VeAnalyzeInjectionEvents, WallClampAndFilmEvidenceUseTheActualAdjustment) {
	InjectionEventHarness harness;
	harness.wall.Enabled = true;
	harness.seed(0);
	auto event = harness.makeEvent(0);

	event.getWallFuel().wallFuel = 0;
	event.getWallFuel().wallFuelCorrection = 99;
	harness.fire(event);
	harness.exportOutput();
	EXPECT_NEAR(13.3333f, harness.engine->outputChannels.veAnalyzeFuelEventLastFinalMg, 0.01f);
	EXPECT_NEAR(3.3333f, harness.engine->outputChannels.veAnalyzeFuelEventLastFilmMg, 0.01f);

	harness.eth.moveTimeForwardUs(1000);
	event.getWallFuel().wallFuel = 1;
	harness.fire(event);
	harness.exportOutput();
	EXPECT_TRUE(harness.engine->outputChannels.veAnalyzeFuelEventLastWallClamp);
	EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelEventLastScheduledMg);
	event.getWallFuel().resetWF();
	EXPECT_FALSE(event.getWallFuel().wasZeroClamped());
	EXPECT_FLOAT_EQ(0, event.getWallFuel().getWallFuel());

	harness.eth.moveTimeForwardUs(1000);
	harness.wall.Enabled = false;
	harness.seed(0);
	event.getWallFuel().wallFuelCorrection = 99;
	harness.fire(event);
	harness.exportOutput();
	EXPECT_FALSE(harness.engine->outputChannels.veAnalyzeFuelEventLastWallClamp);
	EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelEventLastFilmMg);

	harness.eth.moveTimeForwardUs(1000);
	harness.wall.Enabled = true;
	harness.seed(0, NAN);
	harness.fire(event);
	harness.exportOutput();
	EXPECT_FALSE(harness.engine->outputChannels.veAnalyzeFuelEventLastWallClamp);

	event.getWallFuel().wallFuel = 0.25f;
	event.getWallFuel().wallFuelCorrection = 99;
	event.getWallFuel().resetWF();
	EXPECT_FALSE(event.getWallFuel().wasZeroClamped());
	EXPECT_FLOAT_EQ(0, event.getWallFuel().getWallFuel());
}

TEST(VeAnalyzeInjectionEvents, CoverageRequiresFreshConfiguredCylindersAndBanks) {
	{
		InjectionEventHarness harness;
		harness.seedAll();
		auto event = harness.makeEvent(0);
		harness.fire(event);

		auto evidence = harness.snapshot();
		EXPECT_EQ(0x1u, evidence.ObservedCylinderMask);
		EXPECT_EQ(0x1u, evidence.ObservedBankMask);
		EXPECT_EQ(0xFu, evidence.ExpectedCylinderMask);
		EXPECT_EQ(0x3u, evidence.ExpectedBankMask);
		EXPECT_FALSE(evidence.CoverageComplete);
		EXPECT_FALSE(evidence.Ready) << "one fresh cylinder and quiet bank are insufficient";
	}

	{
		InjectionEventHarness harness;
		harness.seedAll();
		harness.configureFuelSchedule();

		for (int cycle = 0; cycle < 18; cycle++) {
			harness.fireFuelSchedule();
			harness.eth.moveTimeForwardAndInvokeEventsUs(100000);
		}

		auto evidence = harness.snapshot();
		EXPECT_EQ(0xFu, evidence.ObservedCylinderMask);
		EXPECT_EQ(0xFu, evidence.FreshCylinderMask);
		EXPECT_EQ(0x3u, evidence.ObservedBankMask);
		EXPECT_EQ(0x3u, evidence.FreshBankMask);
		EXPECT_TRUE(evidence.CoverageComplete);
		EXPECT_TRUE(evidence.Ready);
	}
}

TEST(VeAnalyzeInjectionEvents, GenerationTransitionRetainsCoverageAndProvenanceGatesEvidence) {
	InjectionEventHarness harness;
	harness.seedAll();
	auto initial = harness.makeEvent(0);
	harness.fire(initial);

	auto first = harness.snapshot();
	ASSERT_EQ(0xFu, first.ExpectedCylinderMask);
	ASSERT_EQ(0x3u, first.ExpectedBankMask);

	harness.eth.moveTimeForwardUs(1000);
	harness.engine->airmassCalibration.Generation++;
	const uint32_t nextGeneration = harness.engine->airmassCalibration.Generation;
	harness.seed(2, UnscaledCycleMass, nextGeneration);
	auto transitioned = harness.makeEvent(1);
	harness.fire(transitioned);

	auto transition = harness.snapshot();
	EXPECT_EQ(nextGeneration, transition.PublicationGeneration);
	EXPECT_EQ(0xFu, transition.ExpectedCylinderMask);
	EXPECT_EQ(0x3u, transition.ExpectedBankMask);
	EXPECT_EQ(0x4u, transition.ObservedCylinderMask);
	EXPECT_EQ(0x2u, transition.ObservedBankMask);
	EXPECT_FALSE(transition.CoverageComplete);

	harness.eth.moveTimeForwardUs(1000);
	harness.engine->cylinders[1].setInjectionMass(UnscaledCycleMass);
	auto missing = harness.makeEvent(3);
	harness.fire(missing);
	auto afterMissing = harness.snapshot();
	EXPECT_EQ(nextGeneration, afterMissing.PublicationGeneration);
	EXPECT_NE(0u, afterMissing.rejectedEventCount(FuelEventReason::InvalidGeneration));
	EXPECT_FALSE(afterMissing.Ready);

	harness.eth.moveTimeForwardUs(1000);
	harness.seed(1, UnscaledCycleMass, nextGeneration - 1);
	auto stale = harness.makeEvent(3);
	harness.fire(stale);
	auto afterStale = harness.snapshot();
	EXPECT_EQ(nextGeneration, afterStale.PublicationGeneration);
	EXPECT_NE(0u, afterStale.rejectedEventCount(FuelEventReason::RegressedGeneration));
	EXPECT_FALSE(afterStale.Ready);
}

TEST(VeAnalyzeInjectionEvents, RejectionSupersedesEvidenceAndPollingCannotClearScheduledZero) {
	InjectionEventHarness harness;
	establishStagedEvent(harness);
	harness.eth.moveTimeForwardUs(1000);

	harness.seed(0);
	harness.engine->engineState.injectionStage2Fraction = 0.002f;
	auto unsupported = harness.makeEvent(0);
	harness.fire(unsupported);

	for (int poll = 0; poll < 5; poll++) {
		auto evidence = harness.snapshot();
		EXPECT_TRUE(evidence.LastEvent.SupersededByRejection);
		EXPECT_TRUE(evidence.Cylinders[0].SupersededByRejection);
		EXPECT_FLOAT_EQ(10.0f, evidence.Cylinders[0].ScheduledCommandMassMilligrams);
		EXPECT_FLOAT_EQ(10.0f, evidence.Banks[0].LatestEvent.ScheduledCommandMassMilligrams);
		EXPECT_TRUE(evidence.hasReason(FuelEventReason::IntegrationUnsupported));

		harness.exportOutput();
		EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelCylinderScheduledMg[0]);
		EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelBankScheduledMg[0]);
		EXPECT_FLOAT_EQ(0, harness.engine->outputChannels.veAnalyzeFuelEventLastScheduledMg);
		harness.eth.moveTimeForwardUs(100000);
	}
}

TEST(VeAnalyzeInjectionEvents, OwnerResetSyncLossAndReconstructionRequireFreshCoverage) {
	{
		InjectionEventHarness harness;
		establishStagedEvent(harness);
		harness.exportOutput();
		ASSERT_GT(harness.engine->outputChannels.veAnalyzeFuelEventLastScheduledMg, 0);

		{
			chibios_rt::CriticalSectionLocker csl;
			harness.engine->engineState.resetVeAnalyzeFuelEventsLocked(getTimeNowNt());
		}
		auto afterReset = harness.snapshot();
		EXPECT_EQ(0u, afterReset.ObservedCylinderMask);
		EXPECT_EQ(0u, afterReset.ObservedBankMask);
		EXPECT_EQ(0u, afterReset.AcceptedEventCount);
		EXPECT_FALSE(afterReset.Ready);
		EXPECT_TRUE(afterReset.hasReason(FuelEventReason::ResetRecovery));

		harness.eth.moveTimeForwardUs(1000);
		establishStagedEvent(harness);
		harness.engine->OnTriggerSynchronizationLost();
		auto afterSyncLoss = harness.snapshot();
		EXPECT_EQ(0u, afterSyncLoss.ObservedCylinderMask);
		EXPECT_EQ(0u, afterSyncLoss.AcceptedEventCount);
		EXPECT_FALSE(afterSyncLoss.Ready);
	}

	InjectionEventHarness reconstructed;
	auto evidence = reconstructed.snapshot();
	EXPECT_EQ(0u, evidence.ObservedCylinderMask);
	EXPECT_EQ(0u, evidence.ObservedBankMask);
	EXPECT_EQ(0u, evidence.AttemptedEventCount);
	EXPECT_EQ(0u, evidence.AcceptedEventCount);
	EXPECT_FALSE(evidence.Ready);
}
