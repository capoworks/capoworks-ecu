/**
 * @file	fuel_math.cpp
 * @brief	Fuel amount calculation logic
 *
 *
 * @date May 27, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pch.h"

#include "airmass.h"
#include "alphan_airmass.h"
#include "blended_airmass.h"
#include "maf_airmass.h"
#include "speed_density_airmass.h"
#include "fuel_math.h"
#include "fuel_computer.h"
#include "injector_model.h"
#include "speed_density.h"
#include "speed_density_base.h"
#ifdef MODULE_CYLINDER_FUEL_TRIM
#include "cylinder_fuel_trim.h"
#endif // MODULE_CYLINDER_FUEL_TRIM
#if EFI_LUA
#include "lua_hooks.h"
#endif // EFI_LUA

static mapEstimate_Map3D_t mapEstimationTable;

#if EFI_ENGINE_CONTROL

struct CrankingFuelResult {
	float FuelMass = 0;
	float CoolantTemperatureCoefficient = 0;
	float TpsCoefficient = 0;
	float DurationCoefficient = 0;
};

static CrankingFuelResult calculateCrankingFuel3(float baseFuel, uint32_t revolutionCounterSinceStart) {
	float baseCrankingFuel;
	if (engineConfiguration->useRunningMathForCranking) {
		baseCrankingFuel = baseFuel;
	} else {
		// parameter is in milligrams, convert to grams
		baseCrankingFuel = engineConfiguration->cranking.baseFuel * 0.001f;
	}

	// Cranking fuel changes over time
	float durationCoefficient =
			interpolate2d(revolutionCounterSinceStart, config->crankingCycleBins, config->crankingCycleCoef);

	/**
	 * Cranking fuel is different depending on engine coolant temperature
	 * If the sensor is failed, use 20 deg C
	 */
	auto clt = Sensor::get(SensorType::Clt).value_or(20);
	auto e0Mult = interpolate2d(clt, config->crankingFuelBins, config->crankingFuelCoef);

	bool alreadyWarned = false;
	if (e0Mult <= 0.1f) {
		warning(ObdCode::CUSTOM_ERR_ZERO_E0_MULT, "zero e0 multiplier");
		alreadyWarned = true;
	}

	float coolantTemperatureCoefficient;
	if (engineConfiguration->flexCranking && Sensor::hasSensor(SensorType::FuelEthanolPercent)) {
		auto e85Mult = interpolate2d(clt, config->crankingFuelBins, config->crankingFuelCoefE100);

		if (e85Mult <= 0.1f) {
			warning(ObdCode::CUSTOM_ERR_ZERO_E85_MULT, "zero e85 multiplier");
			alreadyWarned = true;
		}

		auto flex = getFlexEthanolPercent();

		coolantTemperatureCoefficient = interpolateClamped(0, e0Mult, 85, e85Mult, flex);
	} else {
		coolantTemperatureCoefficient = e0Mult;
	}

	auto tps = Sensor::get(SensorType::DriverThrottleIntent);
	float tpsCoefficient = tps.Valid ? interpolate2d(tps.Value, config->crankingTpsBins, config->crankingTpsCoef)
									 : 1; // in case of failed TPS, don't correct.

	floatms_t crankingFuel = baseCrankingFuel * durationCoefficient * coolantTemperatureCoefficient * tpsCoefficient;

	// don't re-warn for zero fuel when we already warned for a more specific problem
	if (!alreadyWarned && crankingFuel <= 0) {
		warning(ObdCode::CUSTOM_ERR_ZERO_CRANKING_FUEL, "Cranking fuel value %f", crankingFuel);
	}

	return {
			crankingFuel,
			coolantTemperatureCoefficient,
			tpsCoefficient,
			durationCoefficient,
	};
}

float getCrankingFuel3(float baseFuel, uint32_t revolutionCounterSinceStart) {
	const auto result = calculateCrankingFuel3(baseFuel, revolutionCounterSinceStart);
	engine->engineState.crankingFuel.coolantTemperatureCoefficient = result.CoolantTemperatureCoefficient;
	engine->engineState.crankingFuel.tpsCoefficient = result.TpsCoefficient;
	engine->engineState.crankingFuel.durationCoefficient = result.DurationCoefficient;
	engine->engineState.crankingFuel.fuel = result.FuelMass * 1000;
	return result.FuelMass;
}

struct RunningFuelResult {
	float FuelMass = 0;
	float BaseFuelMilligrams = 0;
	float Correction = 0;
};

static RunningFuelResult invalidRunningFuelResult() {
	return {
			NAN,
			0,
			NAN,
	};
}

static RunningFuelResult calculateRunningFuel(float baseFuel) {
	ScopePerf perf(PE::GetRunningFuel);

	float iatCorrection = engine->fuelComputer.running.intakeTemperatureCoefficient;
	float cltCorrection = engine->fuelComputer.running.coolantTemperatureCoefficient;
	float postCrankingFuelCorrection = engine->fuelComputer.running.postCrankingFuelCorrection;
	float baroCorrection = engine->engineState.baroCorrection;

	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(iatCorrection), "NaN iatCorrection", invalidRunningFuelResult());
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(cltCorrection), "NaN cltCorrection", invalidRunningFuelResult());
	efiAssert(
			ObdCode::CUSTOM_ERR_ASSERT,
			!std::isnan(postCrankingFuelCorrection),
			"NaN postCrankingFuelCorrection",
			invalidRunningFuelResult());

	float correction = baroCorrection * iatCorrection * cltCorrection * postCrankingFuelCorrection;

#if EFI_ANTILAG_SYSTEM
	correction *= (1 + engine->antilagController.fuelALSCorrection / 100);
#endif /* EFI_ANTILAG_SYSTEM */

#if EFI_LAUNCH_CONTROL
	correction *= engine->launchController.getFuelCoefficient();
#endif

	float runningFuel = baseFuel * correction;

	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(runningFuel), "NaN runningFuel", invalidRunningFuelResult());

	return {
			runningFuel,
			baseFuel * 1000,
			correction,
	};
}

float getRunningFuel(float baseFuel) {
	const auto result = calculateRunningFuel(baseFuel);

	if (!std::isfinite(result.FuelMass) || !std::isfinite(result.Correction)) {
		return 0;
	}

	// Keep the public helper's historical publication behavior for non-transactional callers.
	engine->fuelComputer.running.baseFuel = result.BaseFuelMilligrams;
	engine->fuelComputer.totalFuelCorrection = result.Correction;
	engine->fuelComputer.running.fuel = result.FuelMass * 1000;

	return result.FuelMass;
}

static SpeedDensityAirmass sdAirmass(nullptr, mapEstimationTable);
static MafAirmass mafAirmass;
static AlphaNAirmass alphaNAirmass;
static BlendedAirmass blendedAirmass(sdAirmass, alphaNAirmass);

AirmassModelBase* getAirmassModel(engine_load_mode_e mode) {
	switch (mode) {
		case LM_SPEED_DENSITY:
			return &sdAirmass;
		case LM_REAL_MAF:
			return &mafAirmass;
		case LM_ALPHA_N:
			return &alphaNAirmass;
		case LM_SD_ALPHA_N:
			return &blendedAirmass;
#if EFI_LUA
		case LM_LUA:
			return &(getLuaAirmassModel());
#endif
#if EFI_UNIT_TEST
		case LM_MOCK:
			return engine->mockAirmassModel;
#endif
		default:
			firmwareError(ObdCode::CUSTOM_ERR_ASSERT, "Invalid airmass mode %d", engineConfiguration->fuelAlgorithm);
			return nullptr;
	}
}

static bool isRevisedFuelStrategy(engine_load_mode_e strategy) {
	return strategy == LM_SPEED_DENSITY || strategy == LM_ALPHA_N || strategy == LM_REAL_MAF ||
		   strategy == LM_SD_ALPHA_N;
}

bool isFuelCaptureCurrentLocked(const FuelAirmassCapture& capture, AirmassInjectionState::LockedAccess& admission) {
	if (!admission.isCalculationCurrent(capture.Token)) {
		return false;
	}

	if (capture.Strategy != engineConfiguration->fuelAlgorithm ||
		capture.ConfigurationVersion != engine->getGlobalConfigurationVersion() ||
		capture.CalibrationGeneration != engine->airmassCalibration.Generation ||
		capture.Displacement != engineConfiguration->displacement ||
		capture.CylinderCount != engine->engineState.cylinderCount) {
		return false;
	}

	if (!capture.Standalone) {
		return true;
	}

	const auto& inputs = capture.Inputs;
	return inputs.HasPublicationContext && inputs.PublicationEpoch == capture.Token &&
		   inputs.ConfigurationVersion == capture.ConfigurationVersion &&
		   inputs.CalibrationGeneration == capture.CalibrationGeneration && inputs.ActiveStrategy == capture.Strategy &&
		   inputs.CylinderCount == capture.CylinderCount && inputs.Displacement == capture.Displacement &&
		   (capture.Strategy != LM_SD_ALPHA_N || inputs.BlendLoadSource == config->airmassBlendLoadSource);
}

bool isFuelCaptureCurrent(const FuelAirmassCapture& capture) {
	chibios_rt::CriticalSectionLocker csl;
	auto admission = engine->airmassInjectionState.locked(csl);
	return isFuelCaptureCurrentLocked(capture, admission);
}

bool captureFuelAirmassInputs(
		float rpm, AirmassInjectionState::CalculationToken token, FuelAirmassCapture& capture, bool requireCurrent) {
	capture = {};
	capture.Token = token;

	{
		chibios_rt::CriticalSectionLocker csl;
		auto admission = engine->airmassInjectionState.locked(csl);
		capture.Strategy = engineConfiguration->fuelAlgorithm;
		capture.ConfigurationVersion = engine->getGlobalConfigurationVersion();
		capture.CalibrationGeneration = engine->airmassCalibration.Generation;
		capture.Displacement = engineConfiguration->displacement;
		capture.CylinderCount = engine->engineState.cylinderCount;
		capture.Standalone = isRevisedFuelStrategy(capture.Strategy);
		capture.Current = admission.isCalculationCurrent(token);
	}

	if (requireCurrent && !capture.Current) {
		return false;
	}

	if (!capture.Standalone) {
		capture.Current = true;
		return true;
	}

	// Capture all live model inputs after the identity capture, never through
	// the legacy post-state wrappers used by non-fueling consumers.
	switch (capture.Strategy) {
		case LM_SPEED_DENSITY:
			sdAirmass.captureInputs(rpm, capture.Inputs, true);
			break;
		case LM_ALPHA_N:
			captureAirmassInputs(rpm, capture.Inputs, nullptr, false, true);
			if (capture.Inputs.AlphaNMultiplyMap) {
				resolveCapturedMap(capture.Inputs);
			}
			break;
		case LM_REAL_MAF:
			captureAirmassInputs(rpm, capture.Inputs, nullptr, false, true);
			break;
		case LM_SD_ALPHA_N:
			captureAirmassInputs(rpm, capture.Inputs, nullptr, true, true);
			break;
		default:
			break;
	}

	capture.Current = requireCurrent ? isFuelCaptureCurrent(capture) : true;
	return capture.Current;
}

static AirmassEvaluation getCapturedAirmassForFuel(const AirmassInputs& inputs, AirmassDiagnostics& diagnostics) {
	switch (inputs.ActiveStrategy) {
		case LM_SPEED_DENSITY:
			return sdAirmass.evaluateAirmass(inputs, &diagnostics);
		case LM_ALPHA_N: {
			const auto policy =
					inputs.AlphaNMultiplyMap ? AlphaNPressurePolicy::EffectiveMap : AlphaNPressurePolicy::PureReference;
			return alphaNAirmass.evaluateAirmass(inputs, policy, &diagnostics);
		}
		case LM_REAL_MAF:
			return mafAirmass.evaluateAirmass(inputs, &diagnostics);
		default:
			return {};
	}
}

static float getStandardAirCharge(float displacement, float cylinderCount) {
	if (!std::isfinite(displacement) || displacement <= 0 || !std::isfinite(cylinderCount) || cylinderCount <= 0) {
		return 0;
	}

	return idealGasLaw(displacement / cylinderCount, 101.325f, 273.15f + 20.0f);
}

static bool isBlendedVeAnalyzeResult(
		const BlendedAirmassEvaluation& evaluation,
		const BlendedAirmassDiagnostics& diagnostics,
		const AirmassInputs& inputs,
		AirmassVeAnalyzeEndpoint endpoint) {
	if (!evaluation.Airmass.Valid || endpoint == AirmassVeAnalyzeEndpoint::None ||
		!(diagnostics.Flags & BlendedCalculationValid) ||
		(diagnostics.Flags & (BlendedTemperatureFallback | BlendedBranchFallback | BlendedAuthorityUnavailable |
							  BlendedBaroFallback | BlendedIdleFallback | BlendedCorrectionFallback))) {
		return false;
	}
	if (inputs.UseSeparateVeForIdle && inputs.IdleActive) {
		return false;
	}

	if (endpoint == AirmassVeAnalyzeEndpoint::SpeedDensity) {
		return diagnostics.EffectiveAuthority == 0 && !(diagnostics.Flags & BlendedMapFallback) &&
			   (diagnostics.Flags & BlendedSdValid) && diagnostics.Sd.Valid;
	}

	// Alpha-N does not depend on MAP. A strict MAP failure may still use the
	// historical native-load substitute downstream without invalidating this
	// endpoint's own mass.
	return diagnostics.EffectiveAuthority == 100 && (diagnostics.Flags & BlendedAlphaNValid) &&
		   diagnostics.AlphaN.Valid;
}

static uint16_t veAnalyzeReasonFromFault(AirmassInjectionFault fault) {
	switch (fault) {
		case AirmassInjectionFault::Configuration:
			return airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::Configuration);
		case AirmassInjectionFault::Sensor:
			return airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::Sensor);
		default:
			return airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::CalculationResult);
	}
}

static AirmassVeAnalyzeEndpoint getBlendedVeAnalyzeCandidateEndpoint(
		const BlendedAirmassDiagnostics& diagnostics, AirmassVeAnalyzeEndpoint qualifiedEndpoint) {
	if (qualifiedEndpoint != AirmassVeAnalyzeEndpoint::None) {
		return qualifiedEndpoint;
	}
	if ((diagnostics.Flags & BlendedAuthorityUnavailable) || !(diagnostics.Flags & BlendedAuthorityLoadValid)) {
		return AirmassVeAnalyzeEndpoint::None;
	}
	if (diagnostics.RequestedAuthority <= 0) {
		return AirmassVeAnalyzeEndpoint::SpeedDensity;
	}
	if (diagnostics.RequestedAuthority >= 100) {
		return AirmassVeAnalyzeEndpoint::AlphaN;
	}
	return AirmassVeAnalyzeEndpoint::None;
}

static uint16_t getBlendedVeAnalyzeRejectReasons(
		const BlendedAirmassEvaluation& evaluation,
		const BlendedAirmassDiagnostics& diagnostics,
		const AirmassInputs& inputs,
		AirmassVeAnalyzeEndpoint endpoint) {
	if (isBlendedVeAnalyzeResult(evaluation, diagnostics, inputs, endpoint)) {
		// Strict native-load fallback is downstream evidence, not a reason to
		// reject an otherwise qualified endpoint.
		return 0;
	}

	uint16_t reasons = 0;
	const auto add = [&reasons](AirmassVeAnalyzeRejectReason reason) { reasons |= airmassVeAnalyzeReason(reason); };

	if (diagnostics.Fault != AirmassInjectionFault::None) {
		reasons |= veAnalyzeReasonFromFault(diagnostics.Fault);
	}

	uint16_t fallbackFlags = BlendedTemperatureFallback | BlendedBranchFallback | BlendedAuthorityUnavailable |
							 BlendedBaroFallback | BlendedIdleFallback | BlendedCorrectionFallback;
	// Alpha-N qualification never depends on the strict MAP/native-load
	// substitute. Keep that fallback as evidence only for endpoints that use it.
	if (endpoint != AirmassVeAnalyzeEndpoint::AlphaN) {
		fallbackFlags |= BlendedMapFallback;
	}
	if (diagnostics.Flags & fallbackFlags) {
		add(AirmassVeAnalyzeRejectReason::Fallback);
	}
	if ((inputs.UseSeparateVeForIdle && inputs.IdleActive) || (diagnostics.Flags & BlendedIdleFallback)) {
		add(AirmassVeAnalyzeRejectReason::IdleVe);
	}

	if (endpoint == AirmassVeAnalyzeEndpoint::None) {
		// A valid, available authority load with no endpoint comes from a mixed
		// authority table. Keep this distinct from unavailable input/calibration.
		if (!(diagnostics.Flags & BlendedAuthorityUnavailable) && (diagnostics.Flags & BlendedAuthorityLoadValid)) {
			add(AirmassVeAnalyzeRejectReason::NonUniformAuthority);
		}
	} else if (endpoint == AirmassVeAnalyzeEndpoint::SpeedDensity) {
		if (!(diagnostics.Flags & BlendedSdValid) || (diagnostics.Flags & BlendedMapFallback)) {
			if (diagnostics.Flags & BlendedMapFallback) {
				add(AirmassVeAnalyzeRejectReason::Fallback);
			} else {
				add(AirmassVeAnalyzeRejectReason::Sensor);
			}
		}
	} else if (!(diagnostics.Flags & BlendedAlphaNValid)) {
		add(AirmassVeAnalyzeRejectReason::Sensor);
	}

	if (reasons == 0) {
		add(AirmassVeAnalyzeRejectReason::CalculationResult);
	}
	return reasons;
}

FuelPublicationCandidate
calculateFuelPublication(float rpm, bool isCranking, const FuelAirmassCapture& capture, bool requireCurrent) {
	ScopePerf perf(PE::GetBaseFuel);

	FuelPublicationCandidate candidate;
	if (!capture.Current || (requireCurrent && !isFuelCaptureCurrent(capture))) {
		candidate.Stale = true;
		candidate.VeAnalyzeRejectReasons = airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::StaleCalculation);
		return candidate;
	}

	const bool modelTrimRequired =
			engineConfiguration->isInjectionEnabled && (!isCranking || engineConfiguration->useRunningMathForCranking);
#ifdef MODULE_CYLINDER_FUEL_TRIM
	if (capture.Standalone && engineConfiguration->isInjectionEnabled &&
		capture.CylinderCount > CYLINDER_FUEL_TRIM_COUNT) {
		candidate.Fault = AirmassInjectionFault::Configuration;
		return candidate;
	}
#endif // MODULE_CYLINDER_FUEL_TRIM

	AirmassResult airmass;
	if (capture.Standalone) {
		if (capture.Strategy != LM_SD_ALPHA_N && !isAirmassModelConfigurationValid(capture.Strategy)) {
			candidate.Fault = AirmassInjectionFault::Configuration;
			return candidate;
		}

		if (capture.Strategy == LM_SD_ALPHA_N) {
			const auto evaluation = blendedAirmass.evaluateAirmass(capture.Inputs, &candidate.BlendedDiagnostics);
			candidate.HasBlendedDiagnostics = true;
			if (requireCurrent && !isFuelCaptureCurrent(capture)) {
				candidate.Stale = true;
				candidate.VeAnalyzeRejectReasons =
						airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::StaleCalculation);
				return candidate;
			}
			const auto qualifiedEndpoint = getCapturedAirmassAuthorityEndpoint(capture.Inputs);
			if (requireCurrent && !isFuelCaptureCurrent(capture)) {
				candidate.Stale = true;
				candidate.VeAnalyzeRejectReasons =
						airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::StaleCalculation);
				return candidate;
			}
			const auto candidateEndpoint =
					getBlendedVeAnalyzeCandidateEndpoint(candidate.BlendedDiagnostics, qualifiedEndpoint);
			candidate.VeAnalyzeCandidateEndpoint = static_cast<uint8_t>(candidateEndpoint);
			candidate.VeAnalyzeRejectReasons = getBlendedVeAnalyzeRejectReasons(
					evaluation, candidate.BlendedDiagnostics, capture.Inputs, qualifiedEndpoint);
			if (!evaluation.Airmass.Valid) {
				candidate.Fault = evaluation.Fault;
				return candidate;
			}

			airmass = evaluation.Airmass.Result;
			candidate.Degraded = evaluation.Airmass.Degraded;
			candidate.Fault = evaluation.Fault;
			candidate.NormalizedCylinderFilling = evaluation.NormalizedFilling;
			if (isBlendedVeAnalyzeResult(evaluation, candidate.BlendedDiagnostics, capture.Inputs, qualifiedEndpoint)) {
				candidate.VeAnalyzeEndpoint = static_cast<uint8_t>(qualifiedEndpoint);
			}
		} else {
			const auto evaluation = getCapturedAirmassForFuel(capture.Inputs, candidate.Diagnostics);
			if (requireCurrent && !isFuelCaptureCurrent(capture)) {
				candidate.Stale = true;
				candidate.VeAnalyzeRejectReasons =
						airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::StaleCalculation);
				return candidate;
			}
			if (!evaluation.Valid) {
				candidate.Fault = isAirmassModelConfigurationValid(capture.Strategy)
										? AirmassInjectionFault::Result
										: AirmassInjectionFault::Configuration;
				return candidate;
			}

			airmass = evaluation.Result;
			candidate.Degraded = evaluation.Degraded;
			candidate.Fault = evaluation.Degraded ? AirmassInjectionFault::Sensor : AirmassInjectionFault::None;
		}
	} else {
		auto model = getAirmassModel(capture.Strategy);
		if (model == nullptr) {
			candidate.Fault = AirmassInjectionFault::Configuration;
			return candidate;
		}
		airmass = model->getAirmass(rpm, true).value_or(AirmassResult{});
		candidate.Fault = AirmassInjectionFault::None;
	}

	FuelComputer candidateFuelComputer = engine->fuelComputer;
	const float standardAirCharge = capture.Standalone
										  ? getStandardAirCharge(capture.Displacement, capture.CylinderCount)
										  : getStandardAirCharge();
	if (!std::isfinite(standardAirCharge) || standardAirCharge <= 0) {
		candidate.Fault = AirmassInjectionFault::Configuration;
		return candidate;
	}

	candidate.SdAirMassInOneCylinder = airmass.CylinderAirmass;
	if (capture.Strategy != LM_SD_ALPHA_N) {
		candidate.NormalizedCylinderFilling = 100 * airmass.CylinderAirmass / standardAirCharge;
	}
	candidateFuelComputer.sdAirMassInOneCylinder = candidate.SdAirMassInOneCylinder;
	candidateFuelComputer.normalizedCylinderFilling = candidate.NormalizedCylinderFilling;

	if (capture.Standalone) {
		AirmassInputs consumerInputs = capture.Inputs;
		if (capture.Strategy == LM_REAL_MAF) {
			// MAF's native table coordinate is the pre-trim EngineLoadPercent,
			// while cylinder filling uses the final corrected air mass.
			consumerInputs.NativeLoad = airmass.EngineLoadPercent;
		}
		if (capture.Strategy == LM_ALPHA_N || capture.Strategy == LM_REAL_MAF) {
			// Consumer snapshots need an effective MAP independently of the
			// Alpha-N multiplication policy. Resolve a copy so the model's
			// native coordinate and fuel validity remain untouched.
			resolveCapturedMap(consumerInputs);
		}
		candidate.ConsumerLoads = prepareAirmassConsumerLoads(consumerInputs, airmass.CylinderAirmass);
		if (!candidate.ConsumerLoads.Prepared || !candidate.ConsumerLoads.Snapshot.Valid) {
			candidate.Fault = AirmassInjectionFault::Result;
			return candidate;
		}
		if (candidate.ConsumerLoads.Snapshot.RequiredFallbackUsed && candidate.Fault == AirmassInjectionFault::None) {
			candidate.Degraded = true;
			candidate.Fault = AirmassInjectionFault::Load;
		}
	}

	const AirmassConsumerLoadContext selectedLoads(candidate.ConsumerLoads);
	if (capture.Standalone) {
		candidate.FuelingLoad = selectedLoads.getSelectedLoad(AFR_None);
		// Ignition is optional for fuel validity. When enabled, its captured
		// source is required; a missing sensor receives the upstream substitute.
		candidate.IgnitionLoad = engineConfiguration->isIgnitionEnabled
									   ? selectedLoads.getSelectedLoad(capture.Inputs.IgnitionOverride)
									   : 0;
	} else {
		// Preserve legacy model numerics and live fallback behavior.
		candidate.FuelingLoad = airmass.EngineLoadPercent;
		candidate.IgnitionLoad =
				candidateFuelComputer.getLoadOverride(airmass.EngineLoadPercent, engineConfiguration->ignOverrideMode);
	}
	if (!std::isfinite(candidate.SdAirMassInOneCylinder) || candidate.SdAirMassInOneCylinder < 0 ||
		!std::isfinite(candidate.NormalizedCylinderFilling) || !std::isfinite(candidate.FuelingLoad) ||
		(engineConfiguration->isIgnitionEnabled && !std::isfinite(candidate.IgnitionLoad))) {
		candidate.Fault = AirmassInjectionFault::Result;
		return candidate;
	}

#ifdef MODULE_CYLINDER_FUEL_TRIM
	// Lua and mock models are intentionally non-standalone: they expose no
	// immutable publication context, so trimming them would require live sensor
	// reads after capture. They remain neutral; all captured table models above
	// (Speed Density, Alpha-N, MAF and blended SD/Alpha-N) are trimmed.
	if (modelTrimRequired && capture.Standalone) {
		if (!prepareCylinderFuelTrim(*engine, capture.Inputs, candidate.ConsumerLoads, candidate.CylinderFuelTrim)) {
			candidate.Fault = AirmassInjectionFault::Configuration;
			return candidate;
		}
		if (requireCurrent && !isFuelCaptureCurrent(capture)) {
			candidate.Stale = true;
			return candidate;
		}
		candidate.ApplyCylinderFuelTrim = true;
	}
#else
	UNUSED(modelTrimRequired);
#endif // MODULE_CYLINDER_FUEL_TRIM

	const float gramPerCycle = airmass.CylinderAirmass * capture.CylinderCount;
	const float gramPerMs = rpm == 0 ? 0 : gramPerCycle / getEngineCycleDuration(rpm);
	candidate.AirflowEstimate = gramPerMs * 3600000 /* milliseconds per hour */ / 1000 /* grams per kg */;

	const bool lambdaTargetRequired = capture.Standalone ? candidate.ConsumerLoads.Snapshot.LambdaTargetRequired : true;
	const bool publishFuelOutputs = !capture.Standalone || engineConfiguration->isInjectionEnabled;
	float baseFuelMass = 0;
	if (lambdaTargetRequired) {
		if (capture.Standalone) {
			const float lambdaLoad = selectedLoads.getSelectedLoad(capture.Inputs.LambdaOverride);
			if (!std::isfinite(lambdaLoad)) {
				candidate.Fault = AirmassInjectionFault::Load;
				return candidate;
			}
			candidate.ResolvedLambdaLoad = lambdaLoad;
			baseFuelMass = candidateFuelComputer.getCycleFuelWithResolvedLoad(airmass.CylinderAirmass, rpm, lambdaLoad);
		} else {
			baseFuelMass = candidateFuelComputer.getCycleFuel(airmass.CylinderAirmass, rpm, airmass.EngineLoadPercent);
		}
		if (!std::isfinite(baseFuelMass) || baseFuelMass < 0) {
			candidate.Fault = AirmassInjectionFault::Correction;
			return candidate;
		}
	} else if (capture.Standalone) {
		candidateFuelComputer.publishResolvedLambdaLoad(0);
		candidateFuelComputer.afrTableYAxis = 0;
		candidateFuelComputer.targetLambda = 0;
		candidateFuelComputer.targetAFR = 0;
		candidateFuelComputer.stoichiometricRatio = 0;
	}

	if (publishFuelOutputs && lambdaTargetRequired) {
		const float globalCorrection = engineConfiguration->globalFuelCorrection;
		if (!std::isfinite(globalCorrection) || globalCorrection < 0) {
			candidate.Fault = AirmassInjectionFault::Correction;
			return candidate;
		}
		baseFuelMass *= globalCorrection;
		if (!std::isfinite(baseFuelMass) || baseFuelMass < 0) {
			candidate.Fault = AirmassInjectionFault::Correction;
			return candidate;
		}
	} else {
		baseFuelMass = 0;
	}

	candidate.BaseFuelMass = baseFuelMass;
	candidate.AfrTableYAxis = candidateFuelComputer.afrTableYAxis;
	candidate.TargetLambda = candidateFuelComputer.targetLambda;
	candidate.TargetAFR = candidateFuelComputer.targetAFR;
	candidate.StoichiometricRatio = candidateFuelComputer.stoichiometricRatio;

	float cycleFuelMass = 0;
	if (capture.Standalone && !engineConfiguration->isInjectionEnabled) {
		// A valid air mass remains usable for ignition and diagnostics without
		// requiring an AFR target that no injector will consume.
		candidate.CrankingCoolantTemperatureCoefficient = 0;
		candidate.CrankingTpsCoefficient = 0;
		candidate.CrankingDurationCoefficient = 0;
		candidate.CrankingFuel = 0;
		candidate.RunningBaseFuel = 0;
		candidate.RunningFuel = 0;
		candidate.TotalFuelCorrection = 0;
	} else if (isCranking) {
		const auto cranking =
				calculateCrankingFuel3(baseFuelMass, engine->rpmCalculator.getRevolutionCounterSinceStart());
		candidate.CrankingCoolantTemperatureCoefficient = cranking.CoolantTemperatureCoefficient;
		candidate.CrankingTpsCoefficient = cranking.TpsCoefficient;
		candidate.CrankingDurationCoefficient = cranking.DurationCoefficient;
		candidate.CrankingFuel = cranking.FuelMass;
		cycleFuelMass = cranking.FuelMass;
	} else {
		const auto running = calculateRunningFuel(baseFuelMass);
		candidate.RunningBaseFuel = running.BaseFuelMilligrams;
		candidate.RunningFuel = running.FuelMass;
		candidate.TotalFuelCorrection = running.Correction;
		cycleFuelMass = running.FuelMass;
	}

	if (!std::isfinite(cycleFuelMass) || cycleFuelMass < 0) {
		candidate.Fault = AirmassInjectionFault::Correction;
		return candidate;
	}

	candidate.ModeledCycleFuelMass = cycleFuelMass;
	const bool fuelCut = engine->module<DfcoController>()->cutFuel();
	candidate.FuelCut = fuelCut;
	if (fuelCut) {
		cycleFuelMass = 0;
		candidate.ModeledCycleFuelMass = 0;
	}

	if (publishFuelOutputs) {
		engine->module<InjectorModelPrimary>()->prepare();
		if (engineConfiguration->enableStagedInjection) {
			engine->module<InjectorModelSecondary>()->prepare();
		}
	}

	const floatms_t tpsAccelEnrich = publishFuelOutputs ? engine->module<TpsAccelEnrichment>()->getTpsEnrichment() : 0;
	const float tpsFuelMass =
			publishFuelOutputs ? engine->module<InjectorModelPrimary>()->getFuelMassForDuration(tpsAccelEnrich) : 0;
	candidate.TpsAccelEnrich = tpsAccelEnrich;

	if (!std::isfinite(cycleFuelMass) || cycleFuelMass < 0 || !std::isfinite(tpsAccelEnrich) || tpsAccelEnrich < 0 ||
		!std::isfinite(tpsFuelMass) || tpsFuelMass < 0) {
		candidate.Fault = AirmassInjectionFault::Correction;
		return candidate;
	}

	candidate.AdditiveCycleFuelMass = publishFuelOutputs && !fuelCut ? tpsFuelMass : 0;
	candidate.CycleInjectionMass =
			publishFuelOutputs ? candidate.ModeledCycleFuelMass + candidate.AdditiveCycleFuelMass : 0;
	if (!std::isfinite(candidate.CycleInjectionMass) || candidate.CycleInjectionMass < 0) {
		candidate.Fault = AirmassInjectionFault::Result;
		return candidate;
	}

	candidate.Valid = true;
	return candidate;
}

void publishBlendedAirmassDiagnostics(const FuelPublicationCandidate& candidate) {
	if (!candidate.HasBlendedDiagnostics || candidate.Stale) {
		return;
	}

	const auto& diagnostics = candidate.BlendedDiagnostics;
	auto& output = engine->outputChannels;
	auto& state = engine->engineState;
	state.observeVeAnalyzeCandidate(
			candidate.VeAnalyzeCandidateEndpoint,
			candidate.VeAnalyzeRejectReasons,
			candidate.BlendedDiagnostics.Inputs.CalibrationGeneration,
			candidate.BlendedDiagnostics.Inputs.PublicationEpoch);
	if (!candidate.Valid) {
		// A current rejection remains visible even when the model produced no
		// publishable mass. Do not touch any accepted model outputs.
		state.exportVeAnalyzeDiagnostics(output);
		return;
	}

	if (!state.veAnalyzeSessionStarted) {
		state.veAnalyzeSessionStarted = true;
	}
	if (candidate.VeAnalyzeEndpoint == 0) {
		state.veAnalyzeSessionInvalid = true;
		state.veAnalyzeEndpoint = 0;
	} else if (!state.veAnalyzeSessionInvalid) {
		state.veAnalyzeEndpoint = candidate.VeAnalyzeEndpoint;
	}

	const auto finiteOrZero = [](float value) { return std::isfinite(value) ? value : 0; };
	const auto alphaNLoad = normalizeAirmassPercent(diagnostics.Inputs, diagnostics.Inputs.Tps).value_or(0);

	output.blendedSdMass = finiteOrZero(diagnostics.SdMass);
	output.blendedAlphaNMass = finiteOrZero(diagnostics.AlphaNMass);
	output.blendedRequestedAuthority = diagnostics.RequestedAuthority;
	output.blendedEffectiveAuthority = diagnostics.EffectiveAuthority;
	output.blendedSdLoad = diagnostics.Sd.HasValue ? diagnostics.Inputs.NativeMap.Map : 0;
	output.blendedAlphaNLoad = diagnostics.AlphaN.HasValue ? alphaNLoad : 0;
	output.blendedSdVe = diagnostics.Sd.HasValue && diagnostics.Sd.Valid ? diagnostics.Sd.TableValue : 0;
	output.blendedAlphaNVe =
			diagnostics.AlphaN.HasValue && diagnostics.AlphaN.Valid ? diagnostics.AlphaN.TableValue : 0;
	output.blendedCorrection = finiteOrZero(diagnostics.CorrectionMultiplier);
	output.blendedFault = static_cast<uint8_t>(diagnostics.Fault);
	output.blendedFlags = diagnostics.Flags;
	output.blendedLambdaLoad = finiteOrZero(candidate.ResolvedLambdaLoad);
	output.blendedAuthorityLoad = diagnostics.AuthorityLoadValid ? diagnostics.AuthorityLoad : 0;
	state.exportVeAnalyzeDiagnostics(output);

	output.airmassTemperature = diagnostics.TemperatureValid ? diagnostics.TemperatureK - KELV : 0;
	output.airmassTemperatureSourceUsed = static_cast<uint8_t>(diagnostics.TemperatureSource);
	output.alphaNBaroCoefficient = diagnostics.BaroCoefficient;
	output.airmassPressureFlags =
			(diagnostics.TemperatureFallback ? 1 : 0) | (diagnostics.Flags & BlendedBaroFallback ? 2 : 0) |
			(diagnostics.Flags & BlendedMapFallback ? 4 : 0) | (diagnostics.Flags & BlendedMapEstimateUsed ? 8 : 0);
	if (diagnostics.Map.HasValue) {
		output.fallbackMap = diagnostics.Map.FallbackMap;
	}

	for (size_t i = 0; i < VE_BLEND_COUNT; i++) {
		const auto& blend = diagnostics.Corrections.Blends[i];
		output.veBlendParameter[i] = blend.BlendParameter;
		output.veBlendBias[i] = blend.Bias;
		output.veBlendOutput[i] = blend.Value;
		output.veBlendYAxis[i] = blend.TableYAxis;
	}

	// A composite calculation has two model-native VE coordinates. Consumers
	// use the dedicated branch cursors instead of either legacy scalar.
	state.currentVe = 0;
	state.veTableYAxis = 0;
	if (diagnostics.Sd.IdleWeight > 0) {
		state.idleVeTableYAxis = diagnostics.Sd.IdleLoad;
	} else if (diagnostics.AlphaN.IdleWeight > 0) {
		state.idleVeTableYAxis = diagnostics.AlphaN.IdleLoad;
	}
}

angle_t getInjectionOffset(float rpm, float load) {
	if (!std::isfinite(rpm)) {
		return 0; // error already reported
	}

	if (!std::isfinite(load)) {
		return 0; // error already reported
	}

	angle_t value = interpolate3d(config->injectionPhase, config->injPhaseLoadBins, load, config->injPhaseRpmBins, rpm);

	if (!std::isfinite(value)) {
		// we could be here while resetting configuration for example
		// huh? what? when do we have RPM while resetting configuration? is that CI edge case? shall we fix CI?
		warning(ObdCode::CUSTOM_ERR_6569, "phase map not ready");
		return 0;
	}

	angle_t result = value;
	wrapAngle(result, "inj offset#2", ObdCode::CUSTOM_ERR_6553);
	return result;
}

/**
 * Number of injections using each injector per engine cycle
 * @see getNumberOfSparks
 */
int getNumberOfInjections(injection_mode_e mode, size_t cylinderCount) {
	switch (mode) {
		case IM_SIMULTANEOUS:
		case IM_SINGLE_POINT:
			return static_cast<int>(cylinderCount);
		case IM_BATCH:
			return 2;
		case IM_SEQUENTIAL:
			return 1;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_INVALID_INJECTION_MODE, "Unexpected injection_mode_e %d", mode);
			return 1;
	}
}

int getNumberOfInjections(injection_mode_e mode) {
	return getNumberOfInjections(mode, engine->engineState.cylinderCount);
}

float getInjectionModeDurationMultiplier(injection_mode_e mode, size_t cylinderCount) {
	switch (mode) {
		case IM_SIMULTANEOUS: {
			if (cylinderCount == 0) {
				// we can end up here during configuration reset
				return 0;
			}

			return 1.0f / cylinderCount;
		}
		case IM_SEQUENTIAL:
		case IM_SINGLE_POINT:
			return 1;
		case IM_BATCH:
			return 0.5f;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_INVALID_INJECTION_MODE, "Unexpected injection_mode_e %d", mode);
			return 0;
	}
}

float getInjectionModeDurationMultiplier(injection_mode_e mode) {
	return getInjectionModeDurationMultiplier(mode, engine->engineState.cylinderCount);
}

percent_t getInjectorDutyCycle(float rpm) {
	auto mode = getCurrentInjectionMode();
	floatms_t totalInjectiorAmountPerCycle = engine->engineState.injectionDuration * getNumberOfInjections(mode);
	floatms_t engineCycleDuration = getEngineCycleDuration(rpm);
	return 100 * totalInjectiorAmountPerCycle / engineCycleDuration;
}

percent_t getInjectorDutyCycleStage2(float rpm) {
	auto mode = getCurrentInjectionMode();
	floatms_t totalInjectiorAmountPerCycle = engine->engineState.injectionDurationStage2 * getNumberOfInjections(mode);
	floatms_t engineCycleDuration = getEngineCycleDuration(rpm);
	return 100 * totalInjectiorAmountPerCycle / engineCycleDuration;
}

/**
 * @returns	Mass of each individual fuel injection, in grams
 *     in case of single point injection mode the amount of fuel into all cylinders, otherwise the amount for one
 * cylinder
 */
float getCycleInjectionMass(float rpm, bool isCranking) {
	ScopePerf perf(PE::GetInjectionDuration);
#if EFI_SHAFT_POSITION_INPUT
	FuelAirmassCapture capture;
	captureFuelAirmassInputs(rpm, engine->airmassInjectionState.publicationEpoch(), capture, false);
	const auto candidate = calculateFuelPublication(rpm, isCranking, capture, false);
	return candidate.Valid && !candidate.Stale ? candidate.CycleInjectionMass : 0;
#else
	UNUSED(rpm);
	UNUSED(isCranking);
	return 0;
#endif
}

/**
 * @brief	Initialize fuel map data structure
 * @note this method has nothing to do with fuel map VALUES - it's job
 * is to prepare the fuel map data structure for 3d interpolation
 */
void initFuelMap() {
	mapEstimationTable.init(config->mapEstimateTable, config->mapEstimateTpsBins, config->mapEstimateRpmBins);
}

/**
 * @brief Engine warm-up fuel correction.
 */
float getCltFuelCorrection() {
	const auto clt = Sensor::get(SensorType::Clt);

	if (!clt) {
		return 1; // this error should be already reported somewhere else, let's just handle it
	}

	return interpolate2d(clt.Value, config->cltFuelCorrBins, config->cltFuelCorr);
}

float getIatFuelCorrection() {
	const auto iat = Sensor::get(SensorType::Iat);

	if (!iat) {
		return 1; // this error should be already reported somewhere else, let's just handle it
	}

	return interpolate2d(iat.Value, config->iatFuelCorrBins, config->iatFuelCorr);
}

float getBaroCorrection() {
	if (Sensor::hasSensor(SensorType::BarometricPressure)) {
		// Default to 1atm if failed
		float pressure = Sensor::get(SensorType::BarometricPressure).value_or(101.325f);

		float correction = interpolate3d(
				config->baroCorrTable,
				config->baroCorrPressureBins,
				pressure,
				config->baroCorrRpmBins,
				Sensor::getOrZero(SensorType::Rpm));

		if (std::isnan(correction) || correction < 0.01) {
			warning(ObdCode::OBD_Barometric_Press_Circ_Range_Perf, "Invalid baro correction %f", correction);
			return 1;
		}

		return correction;
	} else {
		return 1;
	}
}

percent_t getFuelALSCorrection(float rpm) {
#if EFI_ANTILAG_SYSTEM
	if (engine->antilagController.isAntilagCondition) {
		float throttleIntent = Sensor::getOrZero(SensorType::DriverThrottleIntent);
		auto AlsFuelAdd = interpolate3d(
				config->ALSFuelAdjustment,
				config->alsFuelAdjustmentLoadBins,
				throttleIntent,
				config->alsFuelAdjustmentrpmBins,
				rpm);
		return AlsFuelAdd;
	} else
#endif /* EFI_ANTILAG_SYSTEM */
	{
		return 0;
	}
}

#if EFI_ENGINE_CONTROL
/**
 * @return Duration of fuel injection while craning
 */
float getCrankingFuel(float baseFuel) {
	return getCrankingFuel3(baseFuel, engine->rpmCalculator.getRevolutionCounterSinceStart());
}

/**
 * Standard cylinder air charge - 100% VE at standard temperature, grams per cylinder
 *
 * Should we bother caching 'getStandardAirCharge' result or can we afford to run the math every time we calculate fuel?
 */
float getStandardAirCharge() {
	float totalDisplacement = engineConfiguration->displacement;
	float cylDisplacement = totalDisplacement / engine->engineState.cylinderCount;

	// Calculation of 100% VE air mass in g/cyl - 1 cylinder filling at 1.204/L
	// 101.325kpa, 20C
	return idealGasLaw(cylDisplacement, 101.325f, 273.15f + 20.0f);
}

static Hysteresis stage2Hysteresis;

float getStage2InjectionFraction(float rpm, float load) {
	if (!engineConfiguration->enableStagedInjection || !std::isfinite(rpm) || !std::isfinite(load)) {
		return 0;
	}

	float frac = 0.01f * interpolate3d(
								 config->injectorStagingTable,
								 config->injectorStagingLoadBins,
								 load,
								 config->injectorStagingRpmBins,
								 rpm);

	// don't allow very small fraction, with some hysteresis
	if (!stage2Hysteresis.test(frac, 0.1, 0.03)) {
		return 0;
	}

	// Clamp to 90%
	if (frac > 0.9) {
		frac = 0.9;
	}

	return frac;
}

#endif
#endif
