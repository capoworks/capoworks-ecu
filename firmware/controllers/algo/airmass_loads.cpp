/**
 * @file airmass_loads.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"

#include "airmass_loads.h"
#include "closed_loop_fuel.h"
#include "fuel_math.h"
#include "lambda_monitor.h"
#ifdef MODULE_CYLINDER_FUEL_TRIM
#include "cylinder_fuel_trim.h"
#endif // MODULE_CYLINDER_FUEL_TRIM

namespace {
using PackedLoad = decltype(output_channels_s::injectionPhaseLoad);

constexpr AirmassConsumerMask consumerBit(AirmassConsumer consumer) {
	return AirmassConsumerMask{1} << static_cast<unsigned>(consumer);
}

static_assert(static_cast<unsigned>(AirmassConsumer::Count) <= 16);
static_assert(std::is_same_v<PackedLoad, std::remove_reference_t<decltype(output_channels_s::ignitionTrimLoad[0])>>);
static_assert(std::is_same_v<PackedLoad, std::remove_reference_t<decltype(output_channels_s::knockGainLoad[0])>>);

bool revisedModel() {
	const auto mode = engineConfiguration->fuelAlgorithm;
	return mode == LM_SPEED_DENSITY || mode == LM_ALPHA_N || mode == LM_REAL_MAF || mode == LM_SD_ALPHA_N;
}

bool cylinderConsumer(AirmassConsumer consumer) {
	return consumer == AirmassConsumer::IgnitionTrim || consumer == AirmassConsumer::KnockGain
#ifdef MODULE_CYLINDER_FUEL_TRIM
		|| consumer == AirmassConsumer::CylinderFuelTrim
#endif // MODULE_CYLINDER_FUEL_TRIM
			;
}

load_override_e selector(AirmassConsumer consumer, size_t index) {
	switch (consumer) {
		case AirmassConsumer::InjectionPhase:
			return config->injectionPhaseLoadSource;
		case AirmassConsumer::IgnitionTrim:
			return index < MAX_CYLINDER_COUNT ? config->ignitionTrimLoadSource[index] : AFR_Invalid;
		case AirmassConsumer::Stft:
			return config->stftLoadSource;
		case AirmassConsumer::Staging:
			return config->stagingLoadSource;
		case AirmassConsumer::LambdaDeviation:
			return config->lambdaDeviationLoadSource;
		case AirmassConsumer::LambdaMonitor:
			return config->lambdaMonitorLoadSource;
		case AirmassConsumer::TrailingSpark:
			return config->trailingSparkLoadSource;
		case AirmassConsumer::IgnitionIat:
			return config->ignitionIatLoadSource;
		case AirmassConsumer::KnockRetard:
			return config->knockRetardLoadSource;
		case AirmassConsumer::KnockGain:
			return index < MAX_CYLINDER_COUNT ? config->knockGainLoadSource[index] : AFR_Invalid;
		case AirmassConsumer::HpfpTarget:
			return config->hpfpTargetLoadSource;
#ifdef MODULE_CYLINDER_FUEL_TRIM
		case AirmassConsumer::CylinderFuelTrim:
			return index < CYLINDER_FUEL_TRIM_COUNT ? config->cylinderFuelTrimLoadSource[index] : AFR_Invalid;
#endif // MODULE_CYLINDER_FUEL_TRIM
		default:
			return AFR_Invalid;
	}
}

bool snapshotCurrent(const AirmassLoadSnapshot& snapshot) {
	return snapshot.Valid && snapshot.ConfigurationVersion == engine->getGlobalConfigurationVersion() &&
		   snapshot.CalibrationGeneration == engine->airmassCalibration.Generation &&
		   snapshot.Strategy == engineConfiguration->fuelAlgorithm &&
		   snapshot.CylinderCount == engine->engineState.cylinderCount &&
		   snapshot.Displacement == engineConfiguration->displacement;
}

bool identityCurrent(const AirmassConsumerLoadIdentity& identity) {
	return identity.ConfigurationVersion == engine->getGlobalConfigurationVersion() &&
		   identity.CalibrationGeneration == engine->airmassCalibration.Generation &&
		   identity.Strategy == engineConfiguration->fuelAlgorithm &&
		   identity.CylinderCount == engine->engineState.cylinderCount &&
		   identity.Displacement == engineConfiguration->displacement &&
		   identity.StateInvalidationEpoch == engine->engineState.airmassLoadsInvalidationEpoch &&
		   identity.PreviousPublicationSerial == engine->engineState.airmassLoadsPublicationSerial;
}

AirmassConsumerLoadIdentity captureIdentity(const AirmassInputs& inputs) {
	AirmassConsumerLoadIdentity identity;
	identity.PublicationEpoch = inputs.PublicationEpoch;
	identity.ConfigurationVersion = inputs.ConfigurationVersion;
	identity.CalibrationGeneration = inputs.CalibrationGeneration;
	identity.Strategy = inputs.ActiveStrategy;
	identity.CylinderCount = inputs.CylinderCount;
	identity.Displacement = inputs.Displacement;
	identity.StateInvalidationEpoch = engine->engineState.airmassLoadsInvalidationEpoch;
	identity.PreviousPublicationSerial = engine->engineState.airmassLoadsPublicationSerial;
	return identity;
}

AirmassConsumerMask enabledConsumerMask(const AirmassInputs& inputs) {
	AirmassConsumerMask result = 0;
	const auto include = [&](AirmassConsumer consumer, bool enabled) {
		if (enabled) {
			result |= consumerBit(consumer);
		}
	};

	include(AirmassConsumer::InjectionPhase, engineConfiguration->isInjectionEnabled);
	include(AirmassConsumer::IgnitionTrim, engineConfiguration->isIgnitionEnabled);
	include(AirmassConsumer::Stft, engineConfiguration->fuelClosedLoopCorrectionEnabled);
	include(AirmassConsumer::Staging,
			engineConfiguration->isInjectionEnabled && engineConfiguration->enableStagedInjection);
	include(AirmassConsumer::LambdaDeviation, engineConfiguration->lambdaProtectionEnable);
	include(AirmassConsumer::LambdaMonitor,
			engineConfiguration->lambdaProtectionEnable || engine->lambdaMonitor.isCut());
	include(AirmassConsumer::TrailingSpark,
			engineConfiguration->isIgnitionEnabled && engineConfiguration->enableTrailingSparks);
	include(AirmassConsumer::IgnitionIat, engineConfiguration->isIgnitionEnabled);
	include(AirmassConsumer::KnockRetard, engineConfiguration->enableSoftwareKnock);
	include(AirmassConsumer::KnockGain, engineConfiguration->enableSoftwareKnock);
	include(AirmassConsumer::HpfpTarget,
			inputs.Rpm >= 60 && enginePins.hpfpValve.isInitialized() && engineConfiguration->hpfpCamLobes > 0 &&
					engineConfiguration->hpfpPumpVolume > 0);
#ifdef MODULE_CYLINDER_FUEL_TRIM
	include(AirmassConsumer::CylinderFuelTrim, engineConfiguration->isInjectionEnabled);
#endif // MODULE_CYLINDER_FUEL_TRIM
	return result;
}

bool stftActive(const AirmassInputs& inputs) {
#if EFI_SHAFT_POSITION_INPUT
	const auto clt = Sensor::get(SensorType::Clt);
	const auto& stft = engineConfiguration->stft;
	return engineConfiguration->fuelClosedLoopCorrectionEnabled && engine->rpmCalculator.isRunning() && clt &&
		   clt.Value >= stft.minClt && engine->fuelComputer.running.timeSinceCrankingInSecs >= stft.startupDelay;
#else
	UNUSED(inputs);
	return false;
#endif
}

bool lambdaProtectionActive(const AirmassInputs& inputs, float monitorLoad) {
	if (!engineConfiguration->lambdaProtectionEnable) {
		return false;
	}
	// With no usable lambda sample there is nothing for protection to compare
	// against this cycle.
	if (!Sensor::get(SensorType::Lambda1).Valid) {
		return false;
	}
	return inputs.Rpm >= engineConfiguration->lambdaProtectionMinRpm && std::isfinite(monitorLoad) &&
		   monitorLoad >= engineConfiguration->lambdaProtectionMinLoad && inputs.Tps &&
		   inputs.Tps.Value > engineConfiguration->lambdaProtectionMinTps &&
		   engine->module<DfcoController>()->getTimeSinceCut() >= engineConfiguration->noFuelTrimAfterDfcoTime &&
		   engine->module<LimpManager>()->getTimeSinceAnyCut() >= 2;
}

bool lambdaTargetRequired(const AirmassInputs& inputs, const AirmassResolvedLoads& loads) {
	const auto monitorLoad = loads.consumer(config->lambdaMonitorLoadSource);
	if (engineConfiguration->isInjectionEnabled ||
		lambdaProtectionActive(inputs, monitorLoad.Valid ? monitorLoad.Value : NAN)) {
		return true;
	}
	if (stftActive(inputs)) {
		for (const auto sensor : {SensorType::Lambda1, SensorType::Lambda2, SensorType::Lambda3, SensorType::Lambda4}) {
			if (Sensor::get(sensor).Valid && shouldUpdateCorrection(sensor)) {
				return true;
			}
		}
	}
	return false;
}

struct CapturedConsumerSelection {
	AirmassConsumerLoadIdentity Identity;
	AirmassConsumerMask EnabledConsumers = 0;
	load_override_e Sources[static_cast<size_t>(AirmassConsumer::Count)][MAX_CYLINDER_COUNT] = {};
};

float strictValue(const AirmassLoad& load) {
	return load.Valid ? load.Value : NAN;
}

float consumerValue(const AirmassResolvedLoads& loads, AirmassConsumer consumer, load_override_e source) {
	if (consumer == AirmassConsumer::HpfpTarget && source == AFR_MAP) {
		// Upstream HPFP keeps zero for a missing measured MAP, unlike the
		// historical 200 kPa substitute used by lambda and ignition tables.
		const auto strict = loads.strict(AFR_MAP);
		return strict.Valid ? strict.Value : 0;
	}
	return loads.consumer(source).Value;
}

float consumerValue(const AirmassLoadSnapshot& snapshot, AirmassConsumer consumer, load_override_e source) {
	if (source > AFR_EffectiveMAP || !(snapshot.ValidSources & (1u << static_cast<unsigned>(source)))) {
		switch (source) {
			case AFR_None:
				if (std::isfinite(snapshot.Native)) {
					return snapshot.Native;
				}
				// Composite and SD native coordinates are effective MAP. A healthy
				// Alpha-N blend can fuel without MAP, but downstream tables retain
				// their historical 200 kPa substitute without changing provenance.
				return snapshot.Strategy == LM_SD_ALPHA_N || snapshot.Strategy == LM_SPEED_DENSITY ? 200 : NAN;
			case AFR_MAP:
			case AFR_EffectiveMAP:
				return consumer == AirmassConsumer::HpfpTarget && source == AFR_MAP ? 0 : 200;
			case AFR_Tps:
			case AFR_AccPedal:
				return 100;
			default:
				return NAN;
		}
	}

	switch (source) {
		case AFR_None:
			return snapshot.Native;
		case AFR_MAP:
			return snapshot.Map;
		case AFR_EffectiveMAP:
			return snapshot.EffectiveMap;
		case AFR_Tps:
			return snapshot.Tps;
		case AFR_AccPedal:
			return snapshot.Pedal;
		case AFR_CylFilling:
			return snapshot.Filling;
		default:
			return NAN;
	}
}

int16_t packCursor(float value) {
	if (!std::isfinite(value)) {
		return 0;
	}
	value = clampF(-3276, value, 3276);
	return static_cast<int16_t>(value * 10.0f);
}

void fillSnapshot(
		AirmassLoadSnapshot& snapshot,
		const AirmassInputs& inputs,
		mass_t finalMass,
		const AirmassResolvedLoads& loads,
		const CapturedConsumerSelection& selection) {
	snapshot.PublicationEpoch = inputs.PublicationEpoch;
	snapshot.ConfigurationVersion = inputs.ConfigurationVersion;
	snapshot.CalibrationGeneration = inputs.CalibrationGeneration;
	snapshot.Strategy = inputs.ActiveStrategy;
	snapshot.CylinderCount = inputs.CylinderCount;
	snapshot.Displacement = inputs.Displacement;

	// Control values keep their historical numeric substitute; strict source
	// validity remains in ValidSources below.
	snapshot.Native = loads.consumer(AFR_None).Value;
	snapshot.Map = strictValue(loads.strict(AFR_MAP));
	snapshot.EffectiveMap = strictValue(loads.strict(AFR_EffectiveMAP));
	snapshot.Tps = strictValue(loads.strict(AFR_Tps));
	snapshot.Pedal = strictValue(loads.strict(AFR_AccPedal));
	snapshot.Filling = strictValue(loads.strict(AFR_CylFilling));
	snapshot.UsesEstimate = loads.strict(AFR_EffectiveMAP).UsesEstimate;
	snapshot.EnabledConsumers = selection.EnabledConsumers;

	// ValidSources is intentionally sourced only from strict resolution.
	for (uint8_t source = 0; source <= AFR_EffectiveMAP; source++) {
		if (loads.strict(static_cast<load_override_e>(source)).Valid) {
			snapshot.ValidSources |= 1u << source;
		}
	}

	// Optional actuator coordinates do not participate in air-charge validity.
	snapshot.Valid = std::isfinite(finalMass) && finalMass >= 0 && loads.strict(AFR_CylFilling).Valid;
	snapshot.LambdaTargetRequired = lambdaTargetRequired(inputs, loads);

	const auto usesFallback = [&](load_override_e source) {
		return !loads.strict(source).Valid && std::isfinite(loads.consumer(source).Value);
	};
	snapshot.RequiredFallbackUsed = (snapshot.LambdaTargetRequired && usesFallback(inputs.LambdaOverride)) ||
									((snapshot.EnabledConsumers & consumerBit(AirmassConsumer::IgnitionTrim)) &&
									 usesFallback(inputs.IgnitionOverride));
	for (size_t i = 0; i < static_cast<size_t>(AirmassConsumer::Count); i++) {
		const auto consumer = static_cast<AirmassConsumer>(i);
		if (!(snapshot.EnabledConsumers & consumerBit(consumer))) {
			continue;
		}
		const size_t count = cylinderConsumer(consumer) ? inputs.CylinderCount : 1;
		for (size_t cylinder = 0; cylinder < count && cylinder < MAX_CYLINDER_COUNT; cylinder++) {
			snapshot.RequiredFallbackUsed |= usesFallback(selection.Sources[i][cylinder]);
		}
	}
}

void publishCursors(const AirmassConsumerLoadCandidate& candidate) {
	auto& out = engine->outputChannels;
	for (size_t i = 0; i < AIRMASS_PACKED_CONSUMER_COUNT; i++) {
		const auto consumer = static_cast<AirmassConsumer>(i);
		const size_t count = cylinderConsumer(consumer) ? MAX_CYLINDER_COUNT : 1;
		for (size_t cylinder = 0; cylinder < count && cylinder < MAX_CYLINDER_COUNT; cylinder++) {
			const auto raw = candidate.DisplayCursors[i][cylinder];
			const float cursor = raw * 0.1f;
			switch (consumer) {
				case AirmassConsumer::InjectionPhase:
					out.injectionPhaseLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::IgnitionTrim:
					out.ignitionTrimLoad[cylinder] = PackedLoad(cursor);
					break;
				case AirmassConsumer::Stft:
					out.stftLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::Staging:
					out.stagingLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::LambdaDeviation:
					out.lambdaDeviationLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::LambdaMonitor:
					out.lambdaMonitorLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::TrailingSpark:
					out.trailingSparkLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::IgnitionIat:
					out.ignitionIatLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::KnockRetard:
					out.knockRetardLoad = PackedLoad(cursor);
					break;
				case AirmassConsumer::KnockGain:
					out.knockGainLoad[cylinder] = PackedLoad(cursor);
					break;
				case AirmassConsumer::HpfpTarget:
					out.hpfpTargetLoad = PackedLoad(cursor);
					break;
				default:
					break;
			}
		}
	}
}

void readLegacySource(
		float (&loads)[AFR_EffectiveMAP + 1], float& hpfpMap, load_override_e source, float legacyDefault) {
	switch (source) {
		case AFR_None:
			loads[AFR_None] = legacyDefault;
			break;
		case AFR_MAP:
			loads[AFR_MAP] = Sensor::get(SensorType::Map).value_or(200);
			break;
		case AFR_Tps:
			loads[AFR_Tps] = Sensor::get(SensorType::Tps1).value_or(100);
			break;
		case AFR_AccPedal:
			loads[AFR_AccPedal] = Sensor::get(SensorType::AcceleratorPedal).value_or(100);
			break;
		case AFR_CylFilling:
			loads[AFR_CylFilling] = engine->fuelComputer.normalizedCylinderFilling;
			break;
		case AFR_EffectiveMAP:
			loads[AFR_EffectiveMAP] = Sensor::get(SensorType::Map).value_or(200);
			break;
		case AFR_Invalid:
			break;
	}

	if (source == AFR_MAP) {
		hpfpMap = Sensor::getOrZero(SensorType::Map);
	} else if (source == AFR_EffectiveMAP) {
		hpfpMap = loads[AFR_EffectiveMAP];
	}
}
} // namespace

AirmassConsumerLoadCandidate prepareAirmassConsumerLoads(const AirmassInputs& inputs, mass_t finalMass) {
	AirmassConsumerLoadCandidate candidate;
	if (!inputs.HasPublicationContext || !revisedModel() || !std::isfinite(inputs.CylinderCount) ||
		inputs.CylinderCount < 1 || inputs.CylinderCount > MAX_CYLINDER_COUNT ||
		inputs.CylinderCount != static_cast<float>(static_cast<uint8_t>(inputs.CylinderCount))) {
		return candidate;
	}

#ifdef MODULE_CYLINDER_FUEL_TRIM
	if (engineConfiguration->isInjectionEnabled && inputs.CylinderCount > CYLINDER_FUEL_TRIM_COUNT) {
		return candidate;
	}
#endif // MODULE_CYLINDER_FUEL_TRIM

	CapturedConsumerSelection selection;
	{
		chibios_rt::CriticalSectionLocker csl;
		candidate.Identity = captureIdentity(inputs);
		selection.Identity = candidate.Identity;
		selection.EnabledConsumers = enabledConsumerMask(inputs);
		for (size_t i = 0; i < static_cast<size_t>(AirmassConsumer::Count); i++) {
			for (size_t cylinder = 0; cylinder < MAX_CYLINDER_COUNT; cylinder++) {
				selection.Sources[i][cylinder] = selector(static_cast<AirmassConsumer>(i), cylinder);
				candidate.Sources[i][cylinder] = selection.Sources[i][cylinder];
			}
		}
	}

	// Resolution uses only the captured inputs. No sensor is read here.
	const AirmassResolvedLoads loads(inputs, finalMass);
	fillSnapshot(candidate.Snapshot, inputs, finalMass, loads, selection);

	for (size_t i = 0; i < AIRMASS_PACKED_CONSUMER_COUNT; i++) {
		const auto consumer = static_cast<AirmassConsumer>(i);
		if (!(candidate.Snapshot.EnabledConsumers & consumerBit(consumer))) {
			continue;
		}
		const size_t count = cylinderConsumer(consumer) ? inputs.CylinderCount : 1;
		for (size_t cylinder = 0; cylinder < count && cylinder < MAX_CYLINDER_COUNT; cylinder++) {
			const auto source = selection.Sources[i][cylinder];
			candidate.DisplayCursors[i][cylinder] = packCursor(consumerValue(loads, consumer, source));
		}
	}

	candidate.Prepared = true;
	return candidate;
}

bool isAirmassConsumerLoadCandidateCurrent(
		const AirmassConsumerLoadCandidate& candidate, AirmassInjectionState::LockedAccess& admission) {
	return candidate.Prepared && admission.isCalculationCurrent(candidate.Identity.PublicationEpoch) &&
		   identityCurrent(candidate.Identity);
}

bool publishAirmassConsumerLoads(
		const AirmassConsumerLoadCandidate& candidate, AirmassInjectionState::LockedAccess& admission) {
	if (!candidate.Snapshot.Valid || !isAirmassConsumerLoadCandidateCurrent(candidate, admission)) {
		return false;
	}

	engine->engineState.airmassLoads = candidate.Snapshot;
	engine->engineState.airmassLoadsPublicationSerial++;
	publishCursors(candidate);
	return true;
}

void clearAirmassConsumerCursorFields(output_channels_s& outputs) {
	outputs.injectionPhaseLoad = PackedLoad{};
	for (auto& value : outputs.ignitionTrimLoad) {
		value = PackedLoad{};
	}
	outputs.stftLoad = PackedLoad{};
	outputs.stagingLoad = PackedLoad{};
	outputs.lambdaDeviationLoad = PackedLoad{};
	outputs.lambdaMonitorLoad = PackedLoad{};
	outputs.trailingSparkLoad = PackedLoad{};
	outputs.ignitionIatLoad = PackedLoad{};
	outputs.knockRetardLoad = PackedLoad{};
	for (auto& value : outputs.knockGainLoad) {
		value = PackedLoad{};
	}
	outputs.hpfpTargetLoad = PackedLoad{};

	outputs.blendedSdMass = 0;
	outputs.blendedAlphaNMass = 0;
	outputs.blendedRequestedAuthority = 0;
	outputs.blendedEffectiveAuthority = 0;
	outputs.blendedSdLoad = 0;
	outputs.blendedAlphaNLoad = 0;
	outputs.blendedSdVe = 0;
	outputs.blendedAlphaNVe = 0;
	outputs.blendedCorrection = 0;
	outputs.blendedFault = static_cast<uint8_t>(AirmassInjectionFault::None);
	outputs.blendedFlags = 0;
	outputs.blendedLambdaLoad = 0;
	outputs.blendedAuthorityLoad = 0;
	outputs.blendedVeAnalyzeEndpoint = 0;
}

void invalidateAirmassConsumerLoadsLocked() {
	auto& state = engine->engineState;
	state.airmassLoads = {};
	state.airmassLoadsInvalidationEpoch++;
	state.airmassLoadsPublicationSerial++;
	clearAirmassConsumerCursorFields(engine->outputChannels);
	// A stop, fault or actual configuration invalidation must clear both the
	// published factors and their model-coordinate cursors on this owner.
#ifdef MODULE_CYLINDER_FUEL_TRIM
	resetCylinderFuelTrim(*engine);
#endif // MODULE_CYLINDER_FUEL_TRIM
}

void invalidateAirmassConsumerLoads() {
	chibios_rt::CriticalSectionLocker csl;
	invalidateAirmassConsumerLoadsLocked();
}

float getAirmassSelectedLoad(load_override_e source, float legacyDefault) {
	chibios_rt::CriticalSectionLocker csl;
	const auto& snapshot = engine->engineState.airmassLoads;
	if (snapshotCurrent(snapshot)) {
		return consumerValue(snapshot, AirmassConsumer::Count, source);
	}
	if (revisedModel()) {
		return NAN;
	}
	float loads[AFR_EffectiveMAP + 1]{};
	float hpfpMap = NAN;
	readLegacySource(loads, hpfpMap, source, legacyDefault);
	return source <= AFR_EffectiveMAP ? loads[source] : NAN;
}

float getAirmassConsumerLoad(AirmassConsumer consumer, size_t index) {
	chibios_rt::CriticalSectionLocker csl;
	if (index >= MAX_CYLINDER_COUNT || consumer >= AirmassConsumer::Count) {
		return NAN;
	}
	if (cylinderConsumer(consumer) && index >= engine->engineState.cylinderCount) {
		return NAN;
	}

	const auto source = selector(consumer, index);
	const auto& snapshot = engine->engineState.airmassLoads;
	if (snapshotCurrent(snapshot)) {
		return consumerValue(snapshot, consumer, source);
	}
	if (revisedModel()) {
		return NAN;
	}

	float loads[AFR_EffectiveMAP + 1]{};
	float hpfpMap = NAN;
	readLegacySource(loads, hpfpMap, source, getFuelingLoad());
	if (consumer == AirmassConsumer::HpfpTarget && source == AFR_MAP) {
		return hpfpMap;
	}
	return source <= AFR_EffectiveMAP ? loads[source] : NAN;
}

expected<float> getEffectiveAirmassMap() {
	chibios_rt::CriticalSectionLocker csl;
	const auto& snapshot = engine->engineState.airmassLoads;
	if (!snapshotCurrent(snapshot) || !(snapshot.ValidSources & (1u << static_cast<unsigned>(AFR_EffectiveMAP)))) {
		return unexpected;
	}
	return snapshot.EffectiveMap;
}

bool isAirmassLambdaTargetRequired() {
	chibios_rt::CriticalSectionLocker csl;
	const auto& snapshot = engine->engineState.airmassLoads;
	// Missing publication must never be interpreted as permission to skip validation.
	return !snapshotCurrent(snapshot) || snapshot.LambdaTargetRequired;
}

bool hasAirmassLoadFallback() {
	chibios_rt::CriticalSectionLocker csl;
	const auto& snapshot = engine->engineState.airmassLoads;
	return snapshotCurrent(snapshot) && snapshot.RequiredFallbackUsed;
}

AirmassConsumerLoadContext::AirmassConsumerLoadContext(AirmassConsumer first, AirmassConsumer second)
	: AirmassConsumerLoadContext(first, second, getFuelingLoad(), getFuelingLoad()) {}

AirmassConsumerLoadContext::AirmassConsumerLoadContext(AirmassConsumer consumer, float legacyDefault)
	: AirmassConsumerLoadContext(consumer, AirmassConsumer::Count, legacyDefault, legacyDefault) {}

AirmassConsumerLoadContext::AirmassConsumerLoadContext(
		AirmassConsumer first, AirmassConsumer second, float firstLegacyDefault, float secondLegacyDefault)
	: m_first(first)
	, m_second(second) {
	if (first == AirmassConsumer::Count && second == AirmassConsumer::Count) {
		return;
	}

	{
		chibios_rt::CriticalSectionLocker csl;
		m_identity.PublicationEpoch = engine->airmassInjectionState.locked(csl).publicationEpoch();
		m_identity.ConfigurationVersion = engine->getGlobalConfigurationVersion();
		m_identity.CalibrationGeneration = engine->airmassCalibration.Generation;
		m_identity.Strategy = engineConfiguration->fuelAlgorithm;
		m_identity.CylinderCount = engine->engineState.cylinderCount;
		m_identity.Displacement = engineConfiguration->displacement;
		m_identity.StateInvalidationEpoch = engine->engineState.airmassLoadsInvalidationEpoch;
		m_identity.PreviousPublicationSerial = engine->engineState.airmassLoadsPublicationSerial;

		const auto& snapshot = engine->engineState.airmassLoads;
		m_hasSnapshot = snapshotCurrent(snapshot);
		m_legacy = !revisedModel();
		if (m_hasSnapshot) {
			m_validSources = snapshot.ValidSources;
			m_loads[AFR_None] = snapshot.Native;
			m_loads[AFR_MAP] = snapshot.Map;
			m_loads[AFR_EffectiveMAP] = snapshot.EffectiveMap;
			m_loads[AFR_Tps] = snapshot.Tps;
			m_loads[AFR_AccPedal] = snapshot.Pedal;
			m_loads[AFR_CylFilling] = snapshot.Filling;
			m_hpfpMap = snapshot.ValidSources & (1u << AFR_MAP) ? snapshot.Map : 0;
		}
		for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
			if (first != AirmassConsumer::Count) {
				m_sources[0][i] = selector(first, i);
			}
			if (second != AirmassConsumer::Count) {
				m_sources[1][i] = selector(second, i);
			}
		}
	}

	if (!m_hasSnapshot && m_legacy) {
		float ignoredHpfpMap = NAN;
		for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
			if (first != AirmassConsumer::Count) {
				readLegacySource(m_loads, ignoredHpfpMap, m_sources[0][i], firstLegacyDefault);
			}
			if (second != AirmassConsumer::Count) {
				readLegacySource(m_loads, ignoredHpfpMap, m_sources[1][i], secondLegacyDefault);
			}
			if (m_sources[0][i] == AFR_MAP || m_sources[1][i] == AFR_MAP) {
				m_hpfpMap = Sensor::getOrZero(SensorType::Map);
			}
		}
	}
}

AirmassConsumerLoadContext::AirmassConsumerLoadContext(const AirmassConsumerLoadCandidate& candidate)
	: m_first(AirmassConsumer::Count)
	, m_second(AirmassConsumer::Count)
	, m_candidate(&candidate) {
	m_identity = candidate.Identity;
	m_hasSnapshot = candidate.Prepared && candidate.Snapshot.Valid;
}

float AirmassConsumerLoadContext::get(AirmassConsumer consumer, size_t index) const {
	if (index >= MAX_CYLINDER_COUNT || consumer >= AirmassConsumer::Count) {
		return NAN;
	}
	if (cylinderConsumer(consumer) && index >= m_identity.CylinderCount) {
		return NAN;
	}
	if (m_candidate) {
		if (!m_hasSnapshot) {
			return NAN;
		}
		const auto source = m_candidate->Sources[static_cast<size_t>(consumer)][index];
		return consumerValue(m_candidate->Snapshot, consumer, source);
	}
	if (!m_hasSnapshot && !m_legacy) {
		return NAN;
	}

	size_t group;
	if (consumer == m_first) {
		group = 0;
	} else if (consumer == m_second) {
		group = 1;
	} else {
		return NAN;
	}

	const auto source = m_sources[group][index];
	if (consumer == AirmassConsumer::HpfpTarget && source == AFR_MAP) {
		return m_hpfpMap;
	}
	if (m_legacy && !m_hasSnapshot) {
		return source <= AFR_EffectiveMAP ? m_loads[source] : NAN;
	}
	AirmassLoadSnapshot snapshot;
	snapshot.Native = m_loads[AFR_None];
	snapshot.Map = m_loads[AFR_MAP];
	snapshot.EffectiveMap = m_loads[AFR_EffectiveMAP];
	snapshot.Tps = m_loads[AFR_Tps];
	snapshot.Pedal = m_loads[AFR_AccPedal];
	snapshot.Filling = m_loads[AFR_CylFilling];
	snapshot.ValidSources = m_validSources;
	snapshot.Valid = m_hasSnapshot || m_legacy;
	return consumerValue(snapshot, consumer, source);
}

float AirmassConsumerLoadContext::getSelectedLoad(load_override_e source) const {
	if (!m_candidate || !m_hasSnapshot) {
		return NAN;
	}
	return consumerValue(m_candidate->Snapshot, AirmassConsumer::Count, source);
}

bool AirmassConsumerLoadContext::isCurrent() const {
	chibios_rt::CriticalSectionLocker csl;
	auto admission = engine->airmassInjectionState.locked(csl);
	return admission.isCalculationCurrent(m_identity.PublicationEpoch) && identityCurrent(m_identity);
}
