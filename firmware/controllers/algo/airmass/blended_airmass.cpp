/**
 * @file blended_airmass.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"

#include "alphan_airmass.h"
#include "blended_airmass.h"
#include "speed_density_airmass.h"

bool isBlendedAirmassConfigurationValid() {
	return std::isfinite(engineConfiguration->displacement) && engineConfiguration->displacement > 0 &&
		   engine->engineState.cylinderCount > 0 &&
		   isAirmassBlendAuthorityCalibrationValid(config->airmassBlendLoadSource) &&
		   isAirmassModelConfigurationValid(LM_SPEED_DENSITY) && isAirmassModelConfigurationValid(LM_ALPHA_N);
}

namespace {
expected<float> getAuthorityLoad(const AirmassInputs& inputs) {
	switch (inputs.BlendLoadSource) {
		case AirmassBlendLoadSource::Tps:
			return normalizeAirmassPercent(inputs, inputs.Tps);
		case AirmassBlendLoadSource::MeasuredMap:
			if (inputs.MeasuredMap.Valid && std::isfinite(inputs.MeasuredMap.Value) && inputs.MeasuredMap.Value >= 0 &&
				inputs.MeasuredMap.Value <= 1000) {
				return inputs.MeasuredMap.Value;
			}
			return unexpected;
		default:
			return unexpected;
	}
}

float interpolateAuthority(float load, float rpm) {
	const auto row = priv::getBin(load, config->airmassBlendLoadBins);
	const auto column = priv::getBin(rpm, config->airmassBlendRpmBins);
	const auto& table = config->airmassBlendTable;
	// Difference form preserves flat endpoint stencils exactly.
	const auto interpolate = [](float low, float high, float fraction) { return low + (high - low) * fraction; };
	const float left = interpolate(table[row.Idx][column.Idx], table[row.Idx + 1][column.Idx], row.Frac);
	const float right = interpolate(table[row.Idx][column.Idx + 1], table[row.Idx + 1][column.Idx + 1], row.Frac);
	return interpolate(left, right, column.Frac);
}

AirmassInjectionFault
branchFault(const AirmassInputs& inputs, engine_load_mode_e model, const RawAirmassDiagnostics& diagnostics) {
	const bool calibrationValid = inputs.LiveCalibration ? isCapturedAirmassModelValid(inputs, model)
														 : isAirmassModelConfigurationValid(model);
	if (!calibrationValid) {
		return AirmassInjectionFault::Configuration;
	}
	return diagnostics.HasValue ? AirmassInjectionFault::Result : AirmassInjectionFault::Sensor;
}

bool isDegraded(uint16_t flags) {
	constexpr uint16_t degradedFlags = BlendedTemperatureFallback | BlendedBranchFallback |
									   BlendedAuthorityUnavailable | BlendedBaroFallback | BlendedIdleFallback |
									   BlendedCorrectionFallback | BlendedMapFallback;
	return (flags & degradedFlags) != 0;
}
} // namespace

void BlendedAirmass::captureInputs(float rpm, AirmassInputs& inputs, bool liveCalibration) const {
	m_sd.captureInputs(rpm, inputs, liveCalibration);
	inputs.NativeLoad = inputs.NativeMap.Valid && std::isfinite(inputs.NativeMap.Map)
							  ? expected<float>(inputs.NativeMap.Map)
							  : expected<float>(unexpected);
}

expected<AirmassResult> BlendedAirmass::getAirmass(float rpm, bool postState) {
	UNUSED(postState);
	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	const auto evaluation = evaluateAirmass(inputs, nullptr).Airmass;
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

BlendedAirmassEvaluation BlendedAirmass::evaluateAirmass(float rpm, BlendedAirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureInputs(rpm, inputs);
	return evaluateAirmass(inputs, diagnostics);
}

BlendedAirmassEvaluation
BlendedAirmass::evaluateAirmass(const AirmassInputs& inputs, BlendedAirmassDiagnostics* diagnostics) const {
	BlendedAirmassEvaluation evaluation;
	uint16_t flags = 0;

	AirmassInputs snapshot = inputs;
	snapshot.ActiveStrategy = LM_SD_ALPHA_N;
	snapshot.NativeLoad = snapshot.NativeMap.Valid && std::isfinite(snapshot.NativeMap.Map)
								? expected<float>(snapshot.NativeMap.Map)
								: expected<float>(unexpected);

	if (snapshot.TemperatureFallback) {
		flags |= BlendedTemperatureFallback;
	}
	if (snapshot.NativeMap.HasValue) {
		flags |= BlendedEstimateEvaluated;
	}
	if (snapshot.NativeMap.UsesEstimate) {
		flags |= BlendedMapEstimateUsed;
	}

	if (diagnostics) {
		*diagnostics = {};
		diagnostics->Inputs = snapshot;
		diagnostics->AuthorityLoadSource = snapshot.BlendLoadSource;
		diagnostics->Map = snapshot.NativeMap;
		diagnostics->TemperatureK = snapshot.TemperatureK;
		diagnostics->TemperatureValid = snapshot.TemperatureValid;
		diagnostics->TemperatureFallback = snapshot.TemperatureFallback;
		diagnostics->TemperatureSource = snapshot.TemperatureSource;
	}

	const auto finish = [&](AirmassInjectionFault fault) {
		evaluation.Fault = fault;
		if (diagnostics) {
			diagnostics->Fault = fault;
			diagnostics->Flags = flags;
		}
		return evaluation;
	};

	if (!std::isfinite(snapshot.Rpm) || snapshot.Rpm <= 0) {
		return finish(AirmassInjectionFault::Sensor);
	}

	const auto authorityLoad = getAuthorityLoad(snapshot);
	if (diagnostics) {
		if (authorityLoad) {
			diagnostics->AuthorityLoad = authorityLoad.Value;
		}
	}

	const bool authorityCalibrationValid =
			snapshot.LiveCalibration ? isCapturedAirmassCalibrationValid(snapshot, AirmassCalibration::Authority)
									 : isAirmassBlendAuthorityCalibrationValid(snapshot.BlendLoadSource);
	const bool authorityAvailable = authorityCalibrationValid && authorityLoad.Valid;
	const bool authorityLoadValid = authorityAvailable && std::isfinite(authorityLoad.Value);
	if (diagnostics) {
		diagnostics->AuthorityLoadValid = authorityLoadValid;
	}
	if (authorityLoadValid) {
		flags |= BlendedAuthorityLoadValid;
	}
	if (!authorityAvailable) {
		flags |= BlendedAuthorityUnavailable;
	}

	float requestedAuthority = 0;
	if (authorityAvailable) {
		requestedAuthority = interpolateAuthority(authorityLoad.Value, snapshot.Rpm);
		if (!std::isfinite(requestedAuthority)) {
			return finish(AirmassInjectionFault::Configuration);
		}
		requestedAuthority = clampF(0, requestedAuthority, 100);
	}
	if (diagnostics) {
		diagnostics->RequestedAuthority = requestedAuthority;
	}

	AirmassEvaluation sd;
	AirmassEvaluation alphaN;
	bool sdEvaluated = false;
	bool alphaNEvaluated = false;
	AirmassInjectionFault activeBranchFault =
			authorityCalibrationValid ? AirmassInjectionFault::Sensor : AirmassInjectionFault::Configuration;

	const auto evaluateBranch = [&](bool speedDensity) {
		RawAirmassDiagnostics raw;
		auto value = speedDensity ? m_sd.evaluateRawAirmass(snapshot, &raw)
								  : m_alphaN.evaluateRawAirmass(snapshot, &raw, AlphaNPressurePolicy::PureReference);
		flags |= speedDensity ? BlendedSdEvaluated : BlendedAlphaNEvaluated;
		if (value.Valid) {
			flags |= speedDensity ? BlendedSdValid : BlendedAlphaNValid;
		} else if (activeBranchFault != AirmassInjectionFault::Configuration) {
			activeBranchFault = branchFault(snapshot, speedDensity ? LM_SPEED_DENSITY : LM_ALPHA_N, raw);
		}
		flags |= raw.IdleFallback ? BlendedIdleFallback : 0;
		flags |= raw.BaroFallback ? BlendedBaroFallback : 0;
		if (diagnostics) {
			if (speedDensity) {
				diagnostics->Sd = raw;
				diagnostics->SdMass = value.Result.CylinderAirmass;
			} else {
				diagnostics->AlphaN = raw;
				diagnostics->AlphaNMass = value.Result.CylinderAirmass;
				diagnostics->BaroCoefficient = raw.BaroCoefficient;
			}
		}
		(speedDensity ? sdEvaluated : alphaNEvaluated) = true;
		(speedDensity ? sd : alphaN) = value;
	};

	float authority = requestedAuthority;
	if (authority < 100) {
		evaluateBranch(true);
	}
	if (authority > 0) {
		evaluateBranch(false);
	}

	bool fallback = !authorityAvailable;
	if ((authority < 100 && !sd.Valid) || (authority > 0 && !alphaN.Valid)) {
		// A failed contributing branch may be replaced only by the complete,
		// healthy alternate result. Never mix a numeric payload from a failure.
		if (!sdEvaluated) {
			evaluateBranch(true);
		}
		if (!alphaNEvaluated) {
			evaluateBranch(false);
		}
		if (!sd.Valid && !alphaN.Valid) {
			evaluation.NativeLoad = resolveAirmassLoad(snapshot, 0, AFR_None);
			return finish(activeBranchFault);
		}
		authority = sd.Valid ? 0 : 100;
		fallback = true;
	}

	if (fallback) {
		flags |= BlendedBranchFallback;
	}
	if (authority < 100 && sd.Valid && snapshot.NativeMap.Fallback) {
		flags |= BlendedMapFallback;
	}
	const float weight = authority * 0.01f;
	evaluation.NativeLoad = resolveAirmassLoad(snapshot, 0, AFR_None);
	if (diagnostics) {
		diagnostics->EffectiveAuthority = authority;
		diagnostics->SdWeight = authority < 100 ? 1 - weight : 0;
		diagnostics->AlphaNWeight = authority > 0 ? weight : 0;
	}

	const float rawMass = authority == 0 ? sd.Result.CylinderAirmass
						: authority == 100
								? alphaN.Result.CylinderAirmass
								: (1 - weight) * sd.Result.CylinderAirmass + weight * alphaN.Result.CylinderAirmass;

	auto corrections = diagnostics ? evaluateAirmassCorrections(snapshot, &diagnostics->Corrections)
								   : evaluateAirmassCorrections(snapshot);
	if (corrections.Fallback) {
		flags |= BlendedCorrectionFallback;
	}
	if (!corrections.Valid) {
		return finish(AirmassInjectionFault::Correction);
	}

	const float finalMass = rawMass * corrections.Multiplier;
	if (diagnostics) {
		diagnostics->RawMass = rawMass;
		diagnostics->CorrectionMultiplier = corrections.Multiplier;
		diagnostics->FinalMass = finalMass;
	}
	if (!std::isfinite(finalMass) || finalMass < 0) {
		return finish(AirmassInjectionFault::Result);
	}

	AirmassResolvedLoads loads(snapshot, finalMass);
	const auto filling = loads.strict(AFR_CylFilling);
	if (!filling.Valid) {
		return finish(AirmassInjectionFault::Result);
	}

	evaluation.NormalizedFilling = filling.Value;
	evaluation.LambdaLoadStrict = loads.strict(snapshot.LambdaOverride);
	evaluation.IgnitionLoadStrict = loads.strict(snapshot.IgnitionOverride);
	evaluation.LambdaLoad = loads.consumer(snapshot.LambdaOverride);
	evaluation.IgnitionLoad = loads.consumer(snapshot.IgnitionOverride);
	evaluation.Airmass.Result = {
			finalMass,
			snapshot.NativeMap.Valid ? snapshot.NativeMap.Map : 0,
	};
	evaluation.Airmass.Valid = true;
	evaluation.Airmass.Degraded = isDegraded(flags);

	if (evaluation.Airmass.Degraded) {
		flags |= BlendedCalculationValid;
		evaluation.Fault = fallback ? activeBranchFault : AirmassInjectionFault::None;
		if (!fallback) {
			if (flags & BlendedCorrectionFallback) {
				evaluation.Fault = AirmassInjectionFault::Correction;
			} else if (
					flags &
					(BlendedTemperatureFallback | BlendedBaroFallback | BlendedIdleFallback | BlendedMapFallback)) {
				evaluation.Fault = AirmassInjectionFault::Sensor;
			} else if (flags & BlendedAuthorityUnavailable) {
				evaluation.Fault = activeBranchFault;
			}
		}
	} else {
		flags |= BlendedCalculationValid;
		evaluation.Fault = AirmassInjectionFault::None;
	}

	if (diagnostics) {
		diagnostics->Fault = evaluation.Fault;
		diagnostics->Flags = flags;
	}
	return evaluation;
}
