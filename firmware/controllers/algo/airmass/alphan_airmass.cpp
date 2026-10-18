#include "pch.h"

#include "alphan_airmass.h"

namespace {
expected<float>
evaluateAlphaNPressure(const AirmassInputs& inputs, AlphaNPressurePolicy policy, float& coefficient, bool& fallback) {
	coefficient = 1;
	fallback = false;

	if (policy == AlphaNPressurePolicy::EffectiveMap) {
		fallback = inputs.NativeMap.Fallback;
		return inputs.NativeMap.Valid && std::isfinite(inputs.NativeMap.Map) && inputs.NativeMap.Map >= 0 &&
							   inputs.NativeMap.Map <= 1000
					 ? expected<float>(inputs.NativeMap.Map)
					 : expected<float>(unexpected);
	}

	if (!inputs.AlphaNBaroCompensation) {
		return 101.325f;
	}

	const float reference = inputs.AlphaNBaroReferencePressure;
	if (!std::isfinite(reference) || reference <= 0 || reference > 200 || !inputs.BarometricPressure ||
		!std::isfinite(inputs.BarometricPressure.Value) || inputs.BarometricPressure.Value <= 0 ||
		inputs.BarometricPressure.Value > 200) {
		fallback = true;
		return 101.325f;
	}

	coefficient = inputs.BarometricPressure.Value / reference;
	if (!std::isfinite(coefficient) || coefficient <= 0) {
		coefficient = 1;
		fallback = true;
	}
	return 101.325f * coefficient;
}

bool validAlphaNInputs(const AirmassInputs& inputs) {
	return std::isfinite(inputs.Rpm) && inputs.Rpm > 0 && normalizeAirmassPercent(inputs, inputs.Tps) &&
		   inputs.TemperatureValid && std::isfinite(inputs.TemperatureK) && inputs.TemperatureK > 0 &&
		   std::isfinite(inputs.Displacement) && inputs.Displacement > 0 && std::isfinite(inputs.CylinderCount) &&
		   inputs.CylinderCount > 0;
}
} // namespace

void AlphaNAirmass::captureInputs(
		float rpm, AlphaNPressurePolicy policy, AirmassInputs& inputs, bool liveCalibration) const {
	captureAirmassInputs(rpm, inputs, nullptr, false, liveCalibration);
	if (policy == AlphaNPressurePolicy::EffectiveMap) {
		resolveCapturedMap(inputs);
	}
}

AirmassEvaluation
AlphaNAirmass::evaluateAirmass(float rpm, AlphaNPressurePolicy policy, AirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureInputs(rpm, policy, inputs);
	return evaluateAirmass(inputs, policy, false, diagnostics);
}

AirmassEvaluation AlphaNAirmass::evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	const auto policy =
			inputs.AlphaNMultiplyMap ? AlphaNPressurePolicy::EffectiveMap : AlphaNPressurePolicy::PureReference;
	if (policy == AlphaNPressurePolicy::EffectiveMap) {
		resolveCapturedMap(inputs);
	}
	return evaluateAirmass(inputs, policy, false, diagnostics);
}

expected<AirmassResult> AlphaNAirmass::getAirmass(float rpm, bool postState) {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	const auto policy =
			inputs.AlphaNMultiplyMap ? AlphaNPressurePolicy::EffectiveMap : AlphaNPressurePolicy::PureReference;
	if (policy == AlphaNPressurePolicy::EffectiveMap) {
		resolveCapturedMap(inputs);
	}
	const auto evaluation = evaluateAirmass(inputs, policy, postState, nullptr);
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

AirmassEvaluation AlphaNAirmass::getAirmassForFuel(float rpm) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false, true);
	const auto policy =
			inputs.AlphaNMultiplyMap ? AlphaNPressurePolicy::EffectiveMap : AlphaNPressurePolicy::PureReference;
	if (policy == AlphaNPressurePolicy::EffectiveMap) {
		resolveCapturedMap(inputs);
	}
	return evaluateAirmass(inputs, policy, true, nullptr);
}

AirmassEvaluation AlphaNAirmass::evaluateAirmass(
		const AirmassInputs& inputs, AlphaNPressurePolicy policy, AirmassDiagnostics* diagnostics) const {
	return evaluateAirmass(inputs, policy, false, diagnostics);
}

AirmassEvaluation AlphaNAirmass::evaluateAirmass(
		const AirmassInputs& inputs,
		AlphaNPressurePolicy policy,
		bool postState,
		AirmassDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}
	AirmassDiagnostics localDiagnostics;
	AirmassDiagnostics& resultDiagnostics = diagnostics ? *diagnostics : localDiagnostics;
	AirmassEvaluation evaluation;
	resultDiagnostics.Map = inputs.NativeMap;
	evaluateAirmassTemperature(inputs, resultDiagnostics);

	const auto tps = normalizeAirmassPercent(inputs, inputs.Tps);
	if (!validAlphaNInputs(inputs) || !tps) {
		return evaluation;
	}

	float coefficient;
	bool pressureFallback;
	const auto pressure = evaluateAlphaNPressure(inputs, policy, coefficient, pressureFallback);
	resultDiagnostics.BaroCoefficient = coefficient;
	resultDiagnostics.BaroFallback = pressureFallback;
	if (!pressure) {
		return evaluation;
	}

	const auto ve = evaluateVe(inputs, tps.Value, postState, VeTableType::AlphaN, &resultDiagnostics.Ve);
	const mass_t airmass = getAirmassImpl(
			ve.Ve * PERCENT_DIV,
			pressure.Value,
			resultDiagnostics.TemperatureK,
			inputs.Displacement,
			inputs.CylinderCount);
	evaluation.Result = {airmass, tps.Value};
	evaluation.Degraded = resultDiagnostics.TemperatureFallback || pressureFallback || resultDiagnostics.Map.Fallback ||
						  ve.IdleFallback || ve.CorrectionFallback;
	evaluation.Valid = ve.Valid && std::isfinite(airmass) && airmass >= 0;
	if (!evaluation.Valid) {
		evaluation.Result.CylinderAirmass = 0;
	} else {
		// Alpha-N trim evidence uses the captured normalized TPS regardless of
		// whether the selected pressure policy also multiplies by MAP.
		evaluation.TrimAlphaNActive = true;
		evaluation.TrimAlphaNWeightedMass = airmass;
		evaluation.TrimAlphaNTps = tps.Value;
	}
	if (postState) {
		publishAirmassDiagnostics(resultDiagnostics);
	}
	return evaluation;
}

float AlphaNAirmass::getVeImpl(float rpm, percent_t load) const {
	return interpolate3d(config->alphaNTable, config->alphaNTpsBins, load, config->alphaNRpmBins, rpm);
}

AirmassEvaluation AlphaNAirmass::evaluateRawAirmass(
		const AirmassInputs& inputs, RawAirmassDiagnostics* diagnostics, AlphaNPressurePolicy pressurePolicy) const {
	if (diagnostics) {
		*diagnostics = {};
	}

	AirmassEvaluation evaluation;
	const auto tps = normalizeAirmassPercent(inputs, inputs.Tps);
	if (!validAlphaNInputs(inputs) || !tps) {
		return evaluation;
	}

	float coefficient;
	bool pressureFallback;
	const auto pressure = evaluateAlphaNPressure(inputs, pressurePolicy, coefficient, pressureFallback);
	if (diagnostics) {
		diagnostics->BaroCoefficient = coefficient;
		diagnostics->BaroFallback = pressureFallback;
	}
	if (!pressure) {
		return evaluation;
	}

	const auto ve = evaluateRawVe(inputs, tps.Value, diagnostics);
	const mass_t airmass = getAirmassImpl(
			ve.Ve * PERCENT_DIV, pressure.Value, inputs.TemperatureK, inputs.Displacement, inputs.CylinderCount);
	evaluation.Result = {airmass, tps.Value};
	evaluation.Degraded = inputs.TemperatureFallback || pressureFallback || ve.IdleFallback ||
						  (pressurePolicy == AlphaNPressurePolicy::EffectiveMap && inputs.NativeMap.Fallback);
	evaluation.Valid = ve.Valid && std::isfinite(airmass) && airmass >= 0;
	if (!evaluation.Valid) {
		evaluation.Result.CylinderAirmass = 0;
	} else {
		evaluation.TrimAlphaNActive = true;
		evaluation.TrimAlphaNWeightedMass = airmass;
		evaluation.TrimAlphaNTps = tps.Value;
	}
	return evaluation;
}
