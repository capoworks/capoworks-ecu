#include "pch.h"

#include "airmass.h"
#include "gppwm_channel.h"
#include "idle_thread.h"
#include "speed_density_base.h"

AirmassVeModelBase::AirmassVeModelBase(const ValueProvider3D* veTable, engine_load_mode_e model)
	: m_veTable(veTable)
	, m_model(model) {}

template <typename T, size_t N>
static bool isAxisValid(const T (&axis)[N], float maximum, float minimum = 0) {
	if (axis[0] < minimum || axis[N - 1] > maximum) {
		return false;
	}
	for (size_t i = 1; i < N; i++) {
		if (axis[i] <= axis[i - 1]) {
			return false;
		}
	}
	return true;
}

bool isMapEstimateAxesValid() {
	return isAxisValid(config->mapEstimateTpsBins, 100) && isAxisValid(config->mapEstimateRpmBins, 18000);
}

bool isMapEstimateConfigurationValid() {
	if (!isMapEstimateAxesValid()) {
		return false;
	}
	for (const auto& row : config->mapEstimateTable) {
		for (float value : row) {
			if (!std::isfinite(value) || value < 0 || value > 600) {
				return false;
			}
		}
	}
	return true;
}

bool isAirmassModelConfigurationValid(engine_load_mode_e model) {
	switch (model) {
		case LM_SPEED_DENSITY:
			return isAxisValid(config->veLoadBins, 1000) && isAxisValid(config->veRpmBins, 18000);
		case LM_ALPHA_N:
			return isAxisValid(config->alphaNTpsBins, 100) && isAxisValid(config->alphaNRpmBins, 18000);
		case LM_REAL_MAF:
			return isAxisValid(config->mafTrimLoadBins, 1000) && isAxisValid(config->mafTrimRpmBins, 18000);
		default:
			return true;
	}
}

namespace {
bool scanCalibration(AirmassCalibration calibration) {
	switch (calibration) {
		case AirmassCalibration::SpeedDensity:
			return isAirmassModelConfigurationValid(LM_SPEED_DENSITY);
		case AirmassCalibration::AlphaN:
			return isAirmassModelConfigurationValid(LM_ALPHA_N);
		case AirmassCalibration::MafTrim:
			return isAirmassModelConfigurationValid(LM_REAL_MAF);
		case AirmassCalibration::MapEstimate:
			return isMapEstimateConfigurationValid();
		case AirmassCalibration::Idle:
			return isAxisValid(config->idleVeLoadBins, 1000) && isAxisValid(config->idleVeRpmBins, 18000);
		default: {
			const auto index =
					static_cast<uint8_t>(calibration) - static_cast<uint8_t>(AirmassCalibration::Correction0);
			if (index >= VE_BLEND_COUNT) {
				return false;
			}
			const auto& correction = config->veBlends[index];
			return isAxisValid(correction.loadBins, 1000) && isAxisValid(correction.rpmBins, 18000) &&
				   isAxisValid(correction.blendBins, 1000, -1000);
		}
	}
}
} // namespace

static_assert(
		static_cast<uint8_t>(AirmassCalibration::Correction0) + VE_BLEND_COUNT <= 32,
		"Airmass calibration bits must fit in the cache");

void invalidateAirmassCalibration() {
	chibios_rt::CriticalSectionLocker csl;
	auto& cache = engine->airmassCalibration;
	++cache.Generation;
	cache.Known = 0;
}

bool isCapturedAirmassCalibrationValid(const AirmassInputs& inputs, AirmassCalibration calibration) {
	if (!inputs.LiveCalibration) {
		return scanCalibration(calibration);
	}

	const uint32_t bit = 1u << static_cast<uint8_t>(calibration);
	{
		chibios_rt::CriticalSectionLocker csl;
		auto& cache = engine->airmassCalibration;
		if (inputs.CalibrationGeneration != cache.Generation ||
			inputs.ConfigurationVersion != engine->getGlobalConfigurationVersion()) {
			return false;
		}
		if (cache.ConfigurationVersion != inputs.ConfigurationVersion) {
			cache.Known = 0;
			cache.ConfigurationVersion = inputs.ConfigurationVersion;
		}
		if (cache.Known & bit) {
			return cache.Valid & bit;
		}
	}

	// Table scans stay outside the critical section. A write must invalidate the
	// generation before changing axes, so an obsolete scan cannot be cached.
	const bool valid = scanCalibration(calibration);
#if EFI_UNIT_TEST
	if (engine->onAirmassCalibrationScanned) {
		engine->onAirmassCalibrationScanned();
	}
#endif

	chibios_rt::CriticalSectionLocker csl;
	auto& cache = engine->airmassCalibration;
	if (inputs.CalibrationGeneration != cache.Generation ||
		inputs.ConfigurationVersion != engine->getGlobalConfigurationVersion()) {
		return false;
	}
#if EFI_UNIT_TEST
	++cache.Scans;
#endif
	cache.Valid = (cache.Valid & ~bit) | (valid ? bit : 0);
	cache.Known |= bit;
	return valid;
}

bool isCapturedAirmassModelValid(const AirmassInputs& inputs, engine_load_mode_e model) {
	switch (model) {
		case LM_SPEED_DENSITY:
			return isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::SpeedDensity);
		case LM_ALPHA_N:
			return isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::AlphaN);
		case LM_REAL_MAF:
			return isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::MafTrim);
		default:
			return true;
	}
}

bool isAirmassConfigurationValid() {
	return isAirmassModelConfigurationValid(engineConfiguration->fuelAlgorithm);
}

bool validateAirmassConfiguration() {
	if (!isAirmassConfigurationValid()) {
		firmwareError(ObdCode::CUSTOM_ERR_ASSERT, "Invalid active airmass model configuration");
		return false;
	}
	return true;
}

static void captureTemperature(AirmassInputs& inputs) {
	inputs.TemperatureK = 0;
	inputs.TemperatureValid = false;
	inputs.TemperatureFallback = false;

	switch (inputs.TemperatureSource) {
		case AirmassTemperatureSource::Tcharge:
			inputs.TemperatureK = engine->engineState.sd.tChargeK;
			inputs.TemperatureValid = std::isfinite(inputs.TemperatureK) && inputs.TemperatureK > 0;
			break;
		case AirmassTemperatureSource::Iat:
			if (inputs.Iat && std::isfinite(inputs.Iat.Value)) {
				inputs.TemperatureK = convertCelsiusToKelvin(inputs.Iat.Value);
				inputs.TemperatureValid = inputs.TemperatureK > 0;
			}
			break;
		default:
			return;
	}

	if (inputs.TemperatureValid) {
		return;
	}

	// Preserve the selected source in diagnostics while retaining the existing
	// air-temperature fallback policy for standalone fueling.
	if (inputs.Iat && std::isfinite(inputs.Iat.Value)) {
		inputs.TemperatureK = convertCelsiusToKelvin(inputs.Iat.Value);
		if (inputs.TemperatureK > 0) {
			inputs.TemperatureValid = true;
			inputs.TemperatureFallback = true;
			return;
		}
	}

	inputs.TemperatureK = convertCelsiusToKelvin(20);
	inputs.TemperatureValid = true;
	inputs.TemperatureFallback = true;
}

expected<float> normalizeAirmassPercent(const AirmassInputs& inputs, expected<float> sensor) {
	if (!sensor || !std::isfinite(sensor.Value) || !std::isfinite(inputs.TpsToleranceMin) ||
		!std::isfinite(inputs.TpsToleranceMax) || inputs.TpsToleranceMin > inputs.TpsToleranceMax ||
		sensor.Value < inputs.TpsToleranceMin || sensor.Value > inputs.TpsToleranceMax) {
		return unexpected;
	}
	return clampF(0, sensor.Value, 100);
}

namespace {
bool isMapCoordinateValid(float map) {
	return std::isfinite(map) && map >= 0 && map <= 1000;
}

bool isMapCoordinateValid(const expected<float>& map) {
	return map.Valid && isMapCoordinateValid(map.Value);
}

bool isMapCoordinateValid(const MapEvaluation& map) {
	return map.Valid && isMapCoordinateValid(map.Map);
}

AirmassLoad resolveNativeModelLoad(const AirmassInputs& inputs) {
	AirmassLoad result;
	switch (inputs.ActiveStrategy) {
		case LM_SPEED_DENSITY:
			result.Value = inputs.NativeMap.Map;
			result.Source = AirmassLoadSource::EffectiveMap;
			result.Unit = AirmassLoadUnit::Kpa;
			result.Valid = isMapCoordinateValid(inputs.NativeMap);
			result.UsesEstimate = inputs.NativeMap.UsesEstimate;
			return result;
		case LM_ALPHA_N: {
			const auto tps = normalizeAirmassPercent(inputs, inputs.Tps);
			result.Source = AirmassLoadSource::Tps;
			result.Valid = tps.Valid;
			if (tps) {
				result.Value = tps.Value;
			}
			return result;
		}
		case LM_REAL_MAF:
			result.Source = AirmassLoadSource::CylinderFilling;
			result.Valid =
					inputs.NativeLoad.Valid && std::isfinite(inputs.NativeLoad.Value) && inputs.NativeLoad.Value >= 0;
			if (result.Valid) {
				result.Value = inputs.NativeLoad.Value;
			}
			return result;
		default:
			return result;
	}
}

AirmassLoad resolveCylinderFilling(const AirmassInputs& inputs, mass_t finalMass) {
	AirmassLoad result;
	result.Source = AirmassLoadSource::CylinderFilling;
	if (!std::isfinite(finalMass) || finalMass < 0 || !std::isfinite(inputs.Displacement) || inputs.Displacement <= 0 ||
		!std::isfinite(inputs.CylinderCount) || inputs.CylinderCount <= 0) {
		return result;
	}

	const float standardCharge = idealGasLaw(inputs.Displacement / inputs.CylinderCount, 101.325f, 293.15f);
	if (!std::isfinite(standardCharge) || standardCharge <= 0) {
		return result;
	}

	result.Value = 100 * finalMass / standardCharge;
	result.Valid = std::isfinite(result.Value) && result.Value >= 0;
	return result;
}

AirmassLoad applyConsumerFallback(AirmassLoad result, bool massValid) {
	if (result.Valid || !massValid) {
		return result;
	}

	switch (result.Source) {
		case AirmassLoadSource::EffectiveMap:
		case AirmassLoadSource::MeasuredMap:
			result.Value = 200;
			break;
		case AirmassLoadSource::Tps:
		case AirmassLoadSource::Pedal:
			result.Value = 100;
			break;
		default:
			return result;
	}

	result.Valid = true;
	result.UsesEstimate = false;
	return result;
}
} // namespace

AirmassLoad resolveAirmassLoad(const AirmassInputs& inputs, mass_t finalMass, load_override_e selector) {
	AirmassLoad result;
	switch (selector) {
		case AFR_None:
			return resolveNativeModelLoad(inputs);
		case AFR_EffectiveMAP:
			result.Value = inputs.NativeMap.Map;
			result.Source = AirmassLoadSource::EffectiveMap;
			result.Unit = AirmassLoadUnit::Kpa;
			result.Valid = isMapCoordinateValid(inputs.NativeMap);
			result.UsesEstimate = inputs.NativeMap.UsesEstimate;
			return result;
		case AFR_MAP:
			result.Source = AirmassLoadSource::MeasuredMap;
			result.Unit = AirmassLoadUnit::Kpa;
			result.Valid = isMapCoordinateValid(inputs.MeasuredMap);
			if (result.Valid) {
				result.Value = inputs.MeasuredMap.Value;
			}
			return result;
		case AFR_Tps: {
			const auto tps = normalizeAirmassPercent(inputs, inputs.Tps);
			result.Source = AirmassLoadSource::Tps;
			result.Valid = tps.Valid;
			if (tps) {
				result.Value = tps.Value;
			}
			return result;
		}
		case AFR_AccPedal: {
			const auto pedal = normalizeAirmassPercent(inputs, inputs.Pedal);
			result.Source = AirmassLoadSource::Pedal;
			result.Valid = pedal.Valid;
			if (pedal) {
				result.Value = pedal.Value;
			}
			return result;
		}
		case AFR_CylFilling:
			return resolveCylinderFilling(inputs, finalMass);
		default:
			return result;
	}
}

AirmassLoad resolveAirmassConsumerLoad(const AirmassInputs& inputs, mass_t finalMass, load_override_e selector) {
	return applyConsumerFallback(
			resolveAirmassLoad(inputs, finalMass, selector), std::isfinite(finalMass) && finalMass >= 0);
}

AirmassResolvedLoads::AirmassResolvedLoads(const AirmassInputs& inputs, mass_t finalMass)
	: m_massValid(std::isfinite(finalMass) && finalMass >= 0) {
	for (uint8_t source = AFR_None; source <= AFR_EffectiveMAP; source++) {
		m_loads[source] = resolveAirmassLoad(inputs, finalMass, static_cast<load_override_e>(source));
	}
}

AirmassLoad AirmassResolvedLoads::strict(load_override_e source) const {
	const auto index = static_cast<uint8_t>(source);
	return index <= AFR_EffectiveMAP ? m_loads[index] : AirmassLoad{};
}

AirmassLoad AirmassResolvedLoads::consumer(load_override_e source) const {
	return applyConsumerFallback(strict(source), m_massValid);
}

void resolveCapturedMap(AirmassInputs& inputs, const ValueProvider3D* estimateTable) {
	auto& result = inputs.NativeMap;
	result = {};
	const bool measuredValid = inputs.MeasuredMap.Valid && std::isfinite(inputs.MeasuredMap.Value) &&
							   inputs.MeasuredMap.Value >= 0 && inputs.MeasuredMap.Value <= 1000;
	result.Map = measuredValid ? inputs.MeasuredMap.Value : 0;
	result.Valid = measuredValid;
	result.Fallback = !measuredValid;

	if (!inputs.UseMapEstimateTable || (measuredValid && !inputs.MapEstimateTransient)) {
		return;
	}
	if (!std::isfinite(inputs.Rpm) || inputs.Rpm < 0 ||
		(inputs.LiveCalibration ? !isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::MapEstimate)
								: (!estimateTable && !isMapEstimateConfigurationValid()))) {
		return;
	}

	const auto normalizedTps = normalizeAirmassPercent(inputs, inputs.Tps);
	const float estimateTps = normalizedTps.value_or(0);
	if (estimateTable) {
		result.FallbackMap = estimateTable->getValue(inputs.Rpm, estimateTps);
	} else {
		result.FallbackMap = interpolate3d(
				config->mapEstimateTable,
				config->mapEstimateTpsBins,
				estimateTps,
				config->mapEstimateRpmBins,
				inputs.Rpm);
	}
	result.HasValue = true;
	const bool estimateValid =
			std::isfinite(result.FallbackMap) && result.FallbackMap >= 0 && result.FallbackMap <= 600;
	result.UsesEstimate =
			estimateValid && (!measuredValid || (inputs.MapEstimateTransient && result.Map < result.FallbackMap));
	if (result.UsesEstimate) {
		result.Map = result.FallbackMap;
	}
	result.Valid = measuredValid || estimateValid;
	result.Fallback = !measuredValid || !estimateValid || !normalizedTps;
}

void captureAirmassInputs(
		float rpm, AirmassInputs& inputs, const ValueProvider3D* estimateTable, bool resolveMap, bool liveCalibration) {
	inputs = {};
	{
		chibios_rt::CriticalSectionLocker csl;
		inputs.PublicationEpoch = engine->airmassInjectionState.locked(csl).publicationEpoch();
		inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
		inputs.ActiveStrategy = engineConfiguration->fuelAlgorithm;
		inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
		inputs.Displacement = engineConfiguration->displacement;
		inputs.CylinderCount = engine->engineState.cylinderCount;
	}
	inputs.HasPublicationContext = true;
	inputs.LiveCalibration = liveCalibration;

	inputs.Rpm = rpm;
	inputs.MeasuredMap = Sensor::get(SensorType::Map);
	inputs.Tps = Sensor::get(SensorType::Tps1);
	inputs.Iat = Sensor::get(SensorType::Iat);
	inputs.BarometricPressure = Sensor::get(SensorType::BarometricPressure);
	inputs.DriverThrottleIntent = Sensor::get(SensorType::DriverThrottleIntent);
	inputs.Pedal = Sensor::get(SensorType::AcceleratorPedal);
	inputs.Maf = Sensor::get(SensorType::Maf);
	inputs.HasSecondaryMaf = Sensor::hasSensor(SensorType::Maf2);
	if (inputs.HasSecondaryMaf) {
		inputs.Maf2 = Sensor::get(SensorType::Maf2);
	}
	inputs.FuelingLoad = getFuelingLoad();
	inputs.IgnitionLoad = getIgnitionLoad();
	inputs.LambdaOverride = engineConfiguration->afrOverrideMode;
	inputs.IgnitionOverride = engineConfiguration->ignOverrideMode;

	inputs.TpsToleranceMin = engineConfiguration->tpsErrorDetectionTooLow;
	inputs.TpsToleranceMax = engineConfiguration->tpsErrorDetectionTooHigh;
	inputs.TemperatureSource = config->airmassTemperatureSource;
	inputs.UseMapEstimateTable = config->useMapEstimateTable;
	inputs.MapEstimateTransient = inputs.UseMapEstimateTable && engineConfiguration->useMapEstimateDuringTransient &&
								  engine->module<TpsAccelEnrichment>()->isAboveAccelThreshold;
	inputs.AlphaNMultiplyMap = config->alphaNMultiplyMap;
	inputs.AlphaNBaroCompensation = config->alphaNBaroCompensation;
	inputs.AlphaNBaroReferencePressure = config->alphaNBaroReferencePressure;
	inputs.IdleLoadSource = config->idleVeLoadSource;
	inputs.UseSeparateVeForIdle = engineConfiguration->useSeparateVeForIdle;
	inputs.IdlePidDeactivationTpsThreshold = engineConfiguration->idlePidDeactivationTpsThreshold;
	inputs.VeOverrideMode = engineConfiguration->veOverrideMode;
	inputs.TwoStroke = engineConfiguration->twoStroke;
#if EFI_IDLE_CONTROL
	inputs.IdleActive = engine->module<IdleController>()->isIdlingOrTaper();
#endif
	captureTemperature(inputs);

	if (resolveMap) {
		resolveCapturedMap(inputs, estimateTable);
	}
}

void evaluateAirmassTemperature(const AirmassInputs& inputs, AirmassDiagnostics& diagnostics) {
	diagnostics.TemperatureK = inputs.TemperatureK;
	diagnostics.TemperatureValid = inputs.TemperatureValid;
	diagnostics.TemperatureFallback = inputs.TemperatureFallback;
	diagnostics.TemperatureSource = inputs.TemperatureSource;
}

void evaluateAirmassTemperature(AirmassDiagnostics& diagnostics) {
	AirmassInputs inputs;
	captureAirmassInputs(0, inputs, nullptr, false);
	evaluateAirmassTemperature(inputs, diagnostics);
}

MapEvaluation evaluateAirmassMap(float rpm, const ValueProvider3D* estimateTable) {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, estimateTable);
	return inputs.NativeMap;
}

void publishAirmassDiagnostics(const AirmassDiagnostics& diagnostics) {
	engine->outputChannels.airmassTemperature = diagnostics.TemperatureValid ? diagnostics.TemperatureK - KELV : 0;
	engine->outputChannels.airmassTemperatureSourceUsed = static_cast<uint8_t>(diagnostics.TemperatureSource);
	engine->outputChannels.alphaNBaroCoefficient = diagnostics.BaroCoefficient;
	engine->outputChannels.airmassPressureFlags =
			(diagnostics.TemperatureFallback ? 1 : 0) | (diagnostics.BaroFallback ? 2 : 0) |
			(diagnostics.Map.Fallback ? 4 : 0) | (diagnostics.Map.UsesEstimate ? 8 : 0);
	if (diagnostics.Map.HasValue) {
		engine->outputChannels.fallbackMap = diagnostics.Map.FallbackMap;
	}
}

static expected<float> resolveIdleVeLoad(const AirmassInputs& inputs, percent_t nativeLoad) {
	switch (inputs.IdleLoadSource) {
		case IdleVeLoadSource::ModelDefault:
			return nativeLoad;
		case IdleVeLoadSource::MeasuredMap: {
			return inputs.MeasuredMap.Valid && std::isfinite(inputs.MeasuredMap.Value) &&
								   inputs.MeasuredMap.Value >= 0 && inputs.MeasuredMap.Value <= 1000
						 ? expected<float>(inputs.MeasuredMap.Value)
						 : expected<float>(unexpected);
		}
		case IdleVeLoadSource::Tps:
			return normalizeAirmassPercent(inputs, inputs.Tps);
		case IdleVeLoadSource::EffectiveMap:
			return inputs.NativeMap.Valid && std::isfinite(inputs.NativeMap.Map) && inputs.NativeMap.Map >= 0 &&
								   inputs.NativeMap.Map <= 1000
						 ? expected<float>(inputs.NativeMap.Map)
						 : expected<float>(unexpected);
		default:
			return unexpected;
	}
}

static expected<float> readCapturedBlendChannel(const AirmassInputs& inputs, gppwm_channel_e channel) {
	switch (channel) {
		case GPPWM_Zero:
			return 0;
		case GPPWM_Rpm:
			return std::isfinite(inputs.Rpm) ? expected<float>(inputs.Rpm) : expected<float>(unexpected);
		case GPPWM_Tps:
			return normalizeAirmassPercent(inputs, inputs.Tps);
		case GPPWM_Map:
			return inputs.MeasuredMap.Valid && std::isfinite(inputs.MeasuredMap.Value) &&
								   inputs.MeasuredMap.Value >= 0 && inputs.MeasuredMap.Value <= 1000
						 ? inputs.MeasuredMap
						 : expected<float>(unexpected);
		case GPPWM_FuelLoad:
			return inputs.FuelingLoad && std::isfinite(inputs.FuelingLoad.Value) ? inputs.FuelingLoad
																				 : expected<float>(unexpected);
		case GPPWM_IgnLoad:
			return inputs.IgnitionLoad && std::isfinite(inputs.IgnitionLoad.Value) ? inputs.IgnitionLoad
																				   : expected<float>(unexpected);
		case GPPWM_BaroPressure:
			return inputs.BarometricPressure && std::isfinite(inputs.BarometricPressure.Value)
						 ? inputs.BarometricPressure
						 : expected<float>(unexpected);
		case GPPWM_Iat:
			return inputs.Iat && std::isfinite(inputs.Iat.Value) ? inputs.Iat : expected<float>(unexpected);
		case GPPWM_AccelPedal:
			return normalizeAirmassPercent(inputs, inputs.Pedal);
		default:
			return readGppwmChannel(channel);
	}
}

static BlendResult
calculateCapturedBlend(const AirmassInputs& inputs, const blend_table_s& cfg, float nativeLoad, bool& fallback) {
	if (cfg.blendParameter == GPPWM_Zero) {
		return {};
	}

	const auto parameter = readCapturedBlendChannel(inputs, cfg.blendParameter);
	if (!parameter || !std::isfinite(parameter.Value)) {
		fallback = true;
		return {};
	}

	float load = nativeLoad;
	if (cfg.yAxisOverride != GPPWM_Zero) {
		const auto yAxis = readCapturedBlendChannel(inputs, cfg.yAxisOverride);
		if (!yAxis || !std::isfinite(yAxis.Value)) {
			fallback = true;
			return {};
		}
		load = yAxis.Value;
	}
	const float tableValue = interpolate3d(cfg.table, cfg.loadBins, load, cfg.rpmBins, inputs.Rpm);
	const float blendFactor = interpolate2d(parameter.Value, cfg.blendBins, cfg.blendValues);
	return {parameter.Value, blendFactor, 0.01f * blendFactor * tableValue, load};
}

VeDiagnostics
AirmassVeModelBase::evaluateRawVe(const AirmassInputs& inputs, float load, VeDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}
	VeDiagnostics result;
	const bool calibrationValid = m_veTable || (inputs.LiveCalibration ? isCapturedAirmassModelValid(inputs, m_model)
																	   : isAirmassModelConfigurationValid(m_model));
	if (!calibrationValid || !std::isfinite(inputs.Rpm) || !std::isfinite(load)) {
		return result;
	}

	float ve = m_veTable ? m_veTable->getValue(inputs.Rpm, load) : getVeImpl(inputs.Rpm, load);
	if (!std::isfinite(ve) || ve < 0) {
		return result;
	}

	float idleVeLoad = load;
#if EFI_IDLE_CONTROL
	// Use only the captured throttle intent and idle state. Model evaluation
	// never re-reads these inputs after the snapshot is complete.
	if (inputs.IdleActive && inputs.UseSeparateVeForIdle) {
		const auto idleLoad = resolveIdleVeLoad(inputs, load);
		const bool idleAxesValid =
				inputs.LiveCalibration
						? isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Idle)
						: (isAxisValid(config->idleVeLoadBins, 1000) && isAxisValid(config->idleVeRpmBins, 18000));
		const float idleThreshold = inputs.IdlePidDeactivationTpsThreshold;
		if (!inputs.DriverThrottleIntent || !idleLoad || !idleAxesValid || !std::isfinite(idleThreshold) ||
			idleThreshold <= 0 || !std::isfinite(inputs.DriverThrottleIntent.Value)) {
			result.IdleFallback = true;
		} else {
			idleVeLoad = idleLoad.Value;
			percent_t idleVe = interpolate3d(
					config->idleVeTable, config->idleVeLoadBins, idleVeLoad, config->idleVeRpmBins, inputs.Rpm);
			if (!std::isfinite(idleVe) || idleVe < 0) {
				result.IdleFallback = true;
			} else {
				ve = interpolateClamped(
						idleThreshold / 2, idleVe, idleThreshold, ve, inputs.DriverThrottleIntent.Value);
			}
		}
	}
#endif // EFI_IDLE_CONTROL

	result.Ve = ve;
	result.Load = load;
	result.IdleLoad = idleVeLoad;
	result.HasValue = true;
	result.Valid = std::isfinite(ve) && ve >= 0;
	if (diagnostics) {
		*diagnostics = result;
	}
	return result;
}

VeDiagnostics AirmassVeModelBase::evaluateVe(
		const AirmassInputs& inputs,
		float load,
		bool postState,
		VeTableType consumer,
		VeDiagnostics* diagnostics) const {
	auto result = evaluateRawVe(inputs, load, diagnostics);
	if (!result.HasValue) {
		return result;
	}

	for (size_t i = 0; i < efi::size(config->veBlends); i++) {
		const auto& blend = config->veBlends[i];
		if (blend.veTableSelect != consumer || blend.blendParameter == GPPWM_Zero) {
			continue;
		}

		const auto calibration =
				static_cast<AirmassCalibration>(static_cast<uint8_t>(AirmassCalibration::Correction0) + i);
		const bool axesValid = inputs.LiveCalibration ? isCapturedAirmassCalibrationValid(inputs, calibration)
													  : scanCalibration(calibration);
		BlendResult blendResult{};
		if (axesValid) {
			bool blendFallback = false;
			blendResult = calculateCapturedBlend(inputs, blend, load, blendFallback);
			result.CorrectionFallback = result.CorrectionFallback || blendFallback;
			const float multiplier = (100 + blendResult.Value) * 0.01f;
			if (!std::isfinite(blendResult.Value) || !std::isfinite(multiplier) || multiplier < 0) {
				result.CorrectionFallback = true;
			} else if (blendResult.Value != 0 && !blendFallback) {
				result.Ve *= multiplier;
			}
		} else {
			result.CorrectionFallback = true;
		}

		if (postState) {
			engine->outputChannels.veBlendParameter[i] = blendResult.BlendParameter;
			engine->outputChannels.veBlendBias[i] = blendResult.Bias;
			engine->outputChannels.veBlendOutput[i] = blendResult.Value;
			engine->outputChannels.veBlendYAxis[i] = blendResult.TableYAxis;
		}
	}

	result.Valid = std::isfinite(result.Ve) && result.Ve >= 0;
	if (postState) {
		engine->engineState.currentVe = result.Ve;
		engine->engineState.veTableYAxis = load;
		engine->engineState.idleVeTableYAxis = result.IdleLoad;
	}
	if (diagnostics) {
		*diagnostics = result;
	}
	return result;
}

VeDiagnostics AirmassVeModelBase::evaluateVe(
		float rpm, float load, bool postState, VeTableType consumer, VeDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	return evaluateVe(inputs, load, postState, consumer, diagnostics);
}

float AirmassVeModelBase::getVe(float rpm, float load, bool postState, VeTableType consumer) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	switch (inputs.VeOverrideMode) {
		case VE_None:
			break;
		case VE_MAP:
			load = inputs.MeasuredMap.value_or(0);
			break;
		case VE_TPS:
			load = inputs.Tps.value_or(0);
			break;
		default:
			load = 0;
			break;
	}
	return evaluateVe(inputs, load, postState, consumer, nullptr).Ve * PERCENT_DIV;
}

float AirmassVeModelBase::getVeImpl(float rpm, percent_t load) const {
	return interpolate3d(config->veTable, config->veLoadBins, load, config->veRpmBins, rpm);
}
