#include "pch.h"
#include "speed_density_airmass.h"

expected<AirmassResult> SpeedDensityAirmass::getAirmass(float rpm, bool postState) {
	ScopePerf perf(PE::GetSpeedDensityFuel);

	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	const auto evaluation = evaluateAirmass(inputs, postState, nullptr);
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

expected<AirmassResult> SpeedDensityAirmass::getAirmass(float rpm, float map, bool postState) {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	inputs.NativeMap.Map = map;
	inputs.NativeMap.Valid = std::isfinite(map) && map >= 0 && map <= 1000;
	inputs.NativeMap.Fallback = !inputs.NativeMap.Valid;
	const auto evaluation = evaluateAirmass(inputs, postState, nullptr);
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

AirmassEvaluation SpeedDensityAirmass::getAirmassForFuel(float rpm) const {
	ScopePerf perf(PE::GetSpeedDensityFuel);

	AirmassInputs inputs;
	captureInputs(rpm, inputs, true);
	return evaluateAirmass(inputs, true, nullptr);
}

AirmassEvaluation SpeedDensityAirmass::evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	return evaluateAirmass(inputs, false, diagnostics);
}

AirmassEvaluation SpeedDensityAirmass::evaluateAirmass(float rpm, float map, AirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	inputs.NativeMap.Map = map;
	inputs.NativeMap.Valid = std::isfinite(map) && map >= 0 && map <= 1000;
	inputs.NativeMap.Fallback = !inputs.NativeMap.Valid;
	return evaluateAirmass(inputs, false, diagnostics);
}

AirmassEvaluation
SpeedDensityAirmass::evaluateAirmass(const AirmassInputs& inputs, AirmassDiagnostics* diagnostics) const {
	return evaluateAirmass(inputs, false, diagnostics);
}

AirmassEvaluation SpeedDensityAirmass::evaluateAirmass(
		const AirmassInputs& inputs, bool postState, AirmassDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}
	AirmassDiagnostics localDiagnostics;
	AirmassDiagnostics& resultDiagnostics = diagnostics ? *diagnostics : localDiagnostics;
	AirmassEvaluation evaluation;
	resultDiagnostics.Map = inputs.NativeMap;
	evaluateAirmassTemperature(inputs, resultDiagnostics);

	if (!inputs.NativeMap.Valid || !std::isfinite(inputs.NativeMap.Map) || inputs.NativeMap.Map < 0 ||
		inputs.NativeMap.Map > 1000 || !resultDiagnostics.TemperatureValid || !std::isfinite(inputs.Rpm) ||
		inputs.Rpm <= 0 || !std::isfinite(inputs.Displacement) || inputs.Displacement <= 0 ||
		!std::isfinite(inputs.CylinderCount) || inputs.CylinderCount <= 0) {
		return evaluation;
	}

	const auto ve =
			evaluateVe(inputs, inputs.NativeMap.Map, postState, VeTableType::SpeedDensity, &resultDiagnostics.Ve);
	const mass_t airmass = getAirmassImpl(
			ve.Ve * PERCENT_DIV,
			inputs.NativeMap.Map,
			resultDiagnostics.TemperatureK,
			inputs.Displacement,
			inputs.CylinderCount);
	evaluation.Result = {
			airmass,
			inputs.NativeMap.Map,
	};
	evaluation.Degraded = resultDiagnostics.TemperatureFallback || resultDiagnostics.Map.Fallback || ve.IdleFallback ||
						  ve.CorrectionFallback;
	evaluation.Valid = ve.Valid && std::isfinite(airmass) && airmass >= 0;
	if (!evaluation.Valid) {
		evaluation.Result.CylinderAirmass = 0;
	} else {
		// This standalone model owns the whole trim mass and indexes on the same
		// captured MAP coordinate used by its physics.
		evaluation.TrimSdActive = true;
		evaluation.TrimSdWeightedMass = airmass;
		evaluation.TrimSdLoadKpa = inputs.NativeMap.Map;
	}
	if (postState) {
		publishAirmassDiagnostics(resultDiagnostics);
	}
	return evaluation;
}

float SpeedDensityAirmass::getAirflow(float rpm, float map, bool postState) {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	inputs.NativeMap.Map = map;
	inputs.NativeMap.Valid = std::isfinite(map) && map >= 0 && map <= 1000;
	inputs.NativeMap.Fallback = !inputs.NativeMap.Valid;
	const auto evaluation = evaluateAirmass(inputs, postState, nullptr);
	if (!evaluation.Valid || evaluation.Result.CylinderAirmass < 0) {
		return 0;
	}

	float massPerCycle = evaluation.Result.CylinderAirmass * inputs.CylinderCount;

	if (!inputs.TwoStroke) {
		// 4 stroke engines only do a half cycle per rev
		massPerCycle = massPerCycle / 2;
	}

	// g/s
	return massPerCycle * inputs.Rpm / 60;
}

float SpeedDensityAirmass::getMap(float rpm, bool postState) const {
	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	const auto& evaluation = inputs.NativeMap;
	if (postState && evaluation.HasValue) {
		engine->outputChannels.fallbackMap = evaluation.FallbackMap;
	}
	return evaluation.Valid ? evaluation.Map : NAN;
}

MapEvaluation SpeedDensityAirmass::evaluateMap(float rpm) const {
	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	return inputs.NativeMap;
}

float SpeedDensityAirmass::getVeImpl(float rpm, percent_t load) const {
	return interpolate3d(config->veTable, config->veLoadBins, load, config->veRpmBins, rpm);
}

AirmassEvaluation
SpeedDensityAirmass::evaluateRawAirmass(const AirmassInputs& inputs, RawAirmassDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}

	AirmassEvaluation evaluation;
	if (!inputs.NativeMap.Valid || !std::isfinite(inputs.NativeMap.Map) || inputs.NativeMap.Map < 0 ||
		inputs.NativeMap.Map > 1000 || !inputs.TemperatureValid || !std::isfinite(inputs.TemperatureK) ||
		inputs.TemperatureK <= 0 || !std::isfinite(inputs.Rpm) || inputs.Rpm <= 0 ||
		!std::isfinite(inputs.Displacement) || inputs.Displacement <= 0 || !std::isfinite(inputs.CylinderCount) ||
		inputs.CylinderCount <= 0) {
		return evaluation;
	}

	const auto ve = evaluateRawVe(inputs, inputs.NativeMap.Map, diagnostics);
	const mass_t airmass = getAirmassImpl(
			ve.Ve * PERCENT_DIV, inputs.NativeMap.Map, inputs.TemperatureK, inputs.Displacement, inputs.CylinderCount);
	evaluation.Result = {airmass, inputs.NativeMap.Map};
	evaluation.Degraded = inputs.TemperatureFallback || inputs.NativeMap.Fallback || ve.IdleFallback;
	evaluation.Valid = ve.Valid && std::isfinite(airmass) && airmass >= 0;
	if (!evaluation.Valid) {
		evaluation.Result.CylinderAirmass = 0;
	} else {
		evaluation.TrimSdActive = true;
		evaluation.TrimSdWeightedMass = airmass;
		evaluation.TrimSdLoadKpa = inputs.NativeMap.Map;
	}
	return evaluation;
}

void SpeedDensityAirmass::captureInputs(float rpm, AirmassInputs& inputs, bool liveCalibration) const {
	captureAirmassInputs(rpm, inputs, m_mapEstimationTable, true, liveCalibration);
}
