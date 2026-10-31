/*
 * engine2.cpp
 *
 * @date Jan 5, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

// todo: move this code to more proper locations

#include "pch.h"

#include "speed_density.h"
#include "fuel_math.h"
#include "airmass_loads.h"
#ifdef MODULE_CYLINDER_FUEL_TRIM
#include "cylinder_fuel_trim.h"
#endif // MODULE_CYLINDER_FUEL_TRIM
#include "closed_loop_fuel.h"
#include "launch_control.h"
#include "injector_model.h"
#include "tunerstudio.h"
#include "gitversion.h"

#if !EFI_UNIT_TEST
#include "status_loop.h"
#endif

WarningCodeState::WarningCodeState() {
	clear();
}

void WarningCodeState::clear() {
	warningCounter = 0;
	lastErrorCode = ObdCode::None;
	recentWarnings.clear();
}

void WarningCodeState::addWarningCode(ObdCode code) {
	warningCounter++;
	lastErrorCode = code;

	warning_t* existing = recentWarnings.find(code);

	if (!existing) {
		chibios_rt::CriticalSectionLocker csl;

		// Add the code to the list
		existing = recentWarnings.add(warning_t(code));
	}

	if (existing) {
		// Reset the timer on the code to now
		existing->LastTriggered.reset();
	}

	// Reset the "any warning" timer too
	timeSinceLastWarning.reset();
}

/**
 * @param forIndicator if we want to retrieving value for TS indicator, this case a minimal period is applued
 */
bool WarningCodeState::isWarningNow() const {
	int period = maxI(3, engineConfiguration->warningPeriod);

	return !timeSinceLastWarning.hasElapsedSec(period);
}

// Check whether a particular warning is active
bool WarningCodeState::isWarningNow(ObdCode code) const {
	warning_t* warn = recentWarnings.find(code);

	// No warning found at all
	if (!warn) {
		return false;
	}

	// If the warning is old, it is not active
	return !warn->LastTriggered.hasElapsedSec(maxI(3, engineConfiguration->warningPeriod));
}

EngineState::EngineState() {
	const efitick_t nowNt = getTimeNowNt();
	timeSinceLastTChargeK.reset(nowNt);
	resetVeAnalyzeSampleGate(nowNt);
}

void EngineState::periodicFastCallback() {
	ScopePerf perf(PE::EngineStatePeriodicFastCallback);

#if EFI_ENGINE_CONTROL
	efitick_t nowNt = getTimeNowNt();
	bool isCranking = engine->rpmCalculator.isCranking();
	float rpm = Sensor::getOrZero(SensorType::Rpm);
	const auto fuelCalculation = engine->airmassInjectionState.beginCalculation(
			engineConfiguration->fuelAlgorithm, rpm, engine->getGlobalConfigurationVersion());
	// The prior completed fuel publication remains live, but this calculation
	// is not valid until every candidate output has been committed below.
	airmassCalculationValid = false;

	if (isCranking) {
		crankingTimer.reset(nowNt);
	}

	engine->fuelComputer.running.timeSinceCrankingInSecs = crankingTimer.getElapsedSeconds(nowNt);

	engine->ignitionState.updateDwell(rpm, isCranking);

	// todo: move this into slow callback, no reason for IAT corr to be here
	engine->fuelComputer.running.intakeTemperatureCoefficient = getIatFuelCorrection();
	// todo: move this into slow callback, no reason for CLT corr to be here
	engine->fuelComputer.running.coolantTemperatureCoefficient = getCltFuelCorrection();

	engine->module<DfcoController>()->update();

	// post-cranking fuel enrichment.
	if (engineConfiguration->postCrankingFuelUseTable) {
		float postCrankingCorr = interpolate3d(
				config->postCrankingEnrichTable,
				config->postCrankingEnrichTempBins,
				Sensor::getOrZero(SensorType::Clt),
				config->postCrankingEnrichRuntimeBins,
				engine->fuelComputer.running.timeSinceCrankingInSecs);

		engine->fuelComputer.running.postCrankingFuelCorrection = std::clamp<float>(postCrankingCorr, 1, 5);
	} else {
		// for compatibility reasons, apply only if the factor is greater than unity (only allow adding fuel)
		if (engineConfiguration->postCrankingFactor > 1.0f) {
			// use interpolation for correction taper
			engine->fuelComputer.running.postCrankingFuelCorrection = interpolateClamped(
					0.0f,
					engineConfiguration->postCrankingFactor,
					engineConfiguration->postCrankingDurationSec,
					1.0f,
					engine->fuelComputer.running.timeSinceCrankingInSecs);
		} else {
			engine->fuelComputer.running.postCrankingFuelCorrection = 1.0f;
		}
	}

	baroCorrection = getBaroCorrection();

	auto tps = Sensor::get(SensorType::Tps1);
	updateTChargeK(rpm, tps.value_or(0));

	FuelAirmassCapture fuelCapture;
	captureFuelAirmassInputs(rpm, fuelCalculation, fuelCapture);
	auto fuelCandidate = calculateFuelPublication(rpm, isCranking, fuelCapture);
	const bool hasCurrentConsumerContext = fuelCapture.Standalone && fuelCandidate.Valid && !fuelCandidate.Stale &&
										   fuelCandidate.ConsumerLoads.Prepared;
	const AirmassConsumerLoadContext currentLoads(fuelCandidate.ConsumerLoads);
	const auto currentConsumerLoad = [&](AirmassConsumer consumer, float legacyLoad, size_t cylinder = 0) {
		if (hasCurrentConsumerContext) {
			return currentLoads.get(consumer, cylinder);
		}
		if (!fuelCapture.Standalone) {
			const AirmassConsumerLoadContext legacyLoads(consumer, legacyLoad);
			return legacyLoads.get(consumer, cylinder);
		}
		return NAN;
	};

	const float luaFuelMult = engine->engineState.lua.fuelMult;
	const float luaFuelAdd = engine->engineState.lua.fuelAdd;
	float cycleFuelMass = 0;
	float modeledCycleFuelMass = 0;
	float additiveCycleFuelMass = 0;
	const bool injectionActive = fuelCandidate.Valid && !fuelCandidate.Stale &&
								 (!fuelCapture.Standalone || engineConfiguration->isInjectionEnabled) &&
								 !fuelCandidate.FuelCut;
	if (injectionActive) {
		modeledCycleFuelMass = fuelCandidate.ModeledCycleFuelMass;
		additiveCycleFuelMass = fuelCandidate.AdditiveCycleFuelMass;
		cycleFuelMass = (modeledCycleFuelMass + additiveCycleFuelMass) * luaFuelMult + luaFuelAdd;
	}
	ClosedLoopFuelResult clResult;
	if (fuelCandidate.Valid && !fuelCandidate.Stale) {
		const float stftLoad = currentConsumerLoad(AirmassConsumer::Stft, fuelCandidate.FuelingLoad);
		clResult = fuelClosedLoopCorrection(stftLoad, fuelCandidate.TargetLambda);
	}

	const float nextFuelLoad = fuelCandidate.Valid ? fuelCandidate.FuelingLoad : 0;
	const float nextIgnitionLoad = fuelCandidate.Valid ? fuelCandidate.IgnitionLoad : 0;
	float lambdaMonitorLoad = nextFuelLoad;
	float lambdaDeviationLoad = nextFuelLoad;
	if (hasCurrentConsumerContext) {
		const bool monitorNeeded = engineConfiguration->lambdaProtectionEnable || engine->lambdaMonitor.isCut();
		lambdaMonitorLoad = monitorNeeded ? currentLoads.get(AirmassConsumer::LambdaMonitor) : 0;
		lambdaDeviationLoad = engineConfiguration->lambdaProtectionEnable
									? currentLoads.get(AirmassConsumer::LambdaDeviation)
									: lambdaMonitorLoad;
	} else if (fuelCapture.Standalone) {
		// A failed or stale physical candidate is unavailable, not a low load.
		lambdaMonitorLoad = NAN;
		lambdaDeviationLoad = NAN;
	} else {
		lambdaMonitorLoad = currentConsumerLoad(AirmassConsumer::LambdaMonitor, nextFuelLoad);
		lambdaDeviationLoad = currentConsumerLoad(AirmassConsumer::LambdaDeviation, nextFuelLoad);
	}
	engine->lambdaMonitor.update(rpm, lambdaMonitorLoad, lambdaDeviationLoad, fuelCandidate.TargetLambda);

	float ignitionIatLoad = nextIgnitionLoad;
	if (!fuelCapture.Standalone || engineConfiguration->isIgnitionEnabled) {
		const float selected = currentConsumerLoad(AirmassConsumer::IgnitionIat, nextIgnitionLoad);
		if (std::isfinite(selected)) {
			ignitionIatLoad = selected;
		}
	}
	engine->ignitionState.updateAdvanceCorrections(ignitionIatLoad);
	float untrimmedAdvance =
			engine->ignitionState.getAdvance(rpm, nextIgnitionLoad, isCranking) * engine->ignitionState.luaTimingMult +
			engine->ignitionState.luaTimingAdd;

	// Torque-reduction controller: pull timing and arm cylinder cut to bring delivered
	// torque down to the requested reduction.
	untrimmedAdvance -= engine->torqueReductionController.update();

	// that's weird logic. also seems broken for two stroke?
	const float nextIgnitionAdvance =
			(float)(untrimmedAdvance > FOUR_STROKE_CYCLE_DURATION / 2 ? untrimmedAdvance - FOUR_STROKE_CYCLE_DURATION
																	  : untrimmedAdvance);

	PreparedTable3DInterpolation ignitionTrimInterpolation(
			config->ignTrimLoadBins, nextIgnitionLoad, config->ignTrimRpmBins, rpm);

	const size_t calculationCylinderCount = fuelCapture.CylinderCount;
#ifdef MODULE_CYLINDER_FUEL_TRIM
	const bool applyCylinderFuelTrim =
			fuelCandidate.ApplyCylinderFuelTrim &&
			isCylinderFuelTrimCandidateValid(fuelCandidate.CylinderFuelTrim, calculationCylinderCount);
#endif // MODULE_CYLINDER_FUEL_TRIM
	bool fuelPublicationValid = fuelCandidate.Valid && !fuelCandidate.Stale && std::isfinite(cycleFuelMass) &&
								cycleFuelMass >= 0 && std::isfinite(luaFuelMult) && std::isfinite(luaFuelAdd) &&
								std::isfinite(rpm) && (rpm > 0 || (!fuelCapture.Standalone && rpm == 0)) &&
								calculationCylinderCount > 0 && calculationCylinderCount <= MAX_CYLINDER_COUNT;
#ifdef MODULE_CYLINDER_FUEL_TRIM
	fuelPublicationValid &= !fuelCandidate.ApplyCylinderFuelTrim || applyCylinderFuelTrim;
#endif // MODULE_CYLINDER_FUEL_TRIM

	for (size_t i = 0; i < STFT_BANK_COUNT; i++) {
		fuelPublicationValid &= std::isfinite(clResult.banks[i]) && clResult.banks[i] >= 0;
	}

	float cylinderFuelMasses[MAX_CYLINDER_COUNT] = {};
	float cylinderTiming[MAX_CYLINDER_COUNT] = {};
	for (size_t i = 0; i < calculationCylinderCount && i < MAX_CYLINDER_COUNT; i++) {
		const uint8_t bankIndex = engineConfiguration->cylinderBankSelect[i];
		float cylinderTrim = 1;
#ifdef MODULE_CYLINDER_FUEL_TRIM
		if (applyCylinderFuelTrim) {
			cylinderTrim = fuelCandidate.CylinderFuelTrim.Factor[i];
		}
#endif // MODULE_CYLINDER_FUEL_TRIM
		const float bankTrim = bankIndex < STFT_BANK_COUNT ? clResult.banks[bankIndex] : 0;
		const float unbankedFuel =
				injectionActive
						? (modeledCycleFuelMass * cylinderTrim + additiveCycleFuelMass) * luaFuelMult + luaFuelAdd
						: 0;
		const float cylinderMass = unbankedFuel * bankTrim;
		if (bankIndex >= STFT_BANK_COUNT || !std::isfinite(bankTrim) || bankTrim < 0 || !std::isfinite(cylinderTrim) ||
			cylinderTrim < 0 || !std::isfinite(cylinderMass) || cylinderMass < 0) {
			fuelPublicationValid = false;
		} else {
			cylinderFuelMasses[i] = cylinderMass;
		}

		angle_t ignitionTrim = 0;
		if (!fuelCapture.Standalone || engineConfiguration->isIgnitionEnabled) {
			const float load = currentConsumerLoad(AirmassConsumer::IgnitionTrim, nextIgnitionLoad, i);
			if (std::isfinite(load)) {
				PreparedTable3DInterpolation selectedInterpolation(
						config->ignTrimLoadBins, load, config->ignTrimRpmBins, rpm);
				ignitionTrim = getCylinderIgnitionTrim(i, selectedInterpolation);
			}
		}
		cylinderTiming[i] = untrimmedAdvance + ignitionTrim;
	}

	const auto injectionMode = getCurrentInjectionMode();
	float nextStage2Fraction = 0;
	float nextInjectionDuration = 0;
	float nextInjectionDurationStage2 = 0;
	float nextInjectionOffset = 0;
	bool nextShouldUpdateInjectionTiming = false;
	if (fuelPublicationValid) {
		const float injectionFuelMass =
				cycleFuelMass * getInjectionModeDurationMultiplier(injectionMode, calculationCylinderCount);
		const float stagingLoad = currentConsumerLoad(AirmassConsumer::Staging, fuelCandidate.AfrTableYAxis);
		nextStage2Fraction =
				engineConfiguration->enableStagedInjection ? getStage2InjectionFraction(rpm, stagingLoad) : 0;
		const float stage2InjectionMass = injectionFuelMass * nextStage2Fraction;
		const float stage1InjectionMass = injectionFuelMass - stage2InjectionMass;

		nextInjectionDuration = engine->module<InjectorModelPrimary>()->getInjectionDuration(stage1InjectionMass);
		nextInjectionDurationStage2 =
				engineConfiguration->enableStagedInjection
						? engine->module<InjectorModelSecondary>()->getInjectionDuration(stage2InjectionMass)
						: 0;
		nextInjectionOffset =
				getInjectionOffset(rpm, currentConsumerLoad(AirmassConsumer::InjectionPhase, nextFuelLoad));

		const float totalInjectionTime =
				nextInjectionDuration * getNumberOfInjections(injectionMode, calculationCylinderCount);
		const float cycleDuration = getEngineCycleDuration(rpm);
		nextShouldUpdateInjectionTiming = 100 * totalInjectionTime / cycleDuration < 90;

		fuelPublicationValid &=
				std::isfinite(injectionFuelMass) && injectionFuelMass >= 0 && std::isfinite(nextStage2Fraction) &&
				nextStage2Fraction >= 0 && nextStage2Fraction <= 1 && std::isfinite(stage1InjectionMass) &&
				stage1InjectionMass >= 0 && std::isfinite(stage2InjectionMass) && stage2InjectionMass >= 0 &&
				std::isfinite(nextInjectionDuration) && nextInjectionDuration >= 0 &&
				std::isfinite(nextInjectionDurationStage2) && nextInjectionDurationStage2 >= 0 &&
				std::isfinite(nextInjectionOffset) &&
				((!fuelCapture.Standalone && rpm == 0) || (std::isfinite(cycleDuration) && cycleDuration > 0));
	}

	fuelCandidate.Stage2Fraction = nextStage2Fraction;
	fuelCandidate.InjectionDuration = nextInjectionDuration;
	fuelCandidate.InjectionDurationStage2 = nextInjectionDurationStage2;
	fuelCandidate.InjectionOffset = nextInjectionOffset;
	fuelCandidate.ShouldUpdateInjectionTiming = nextShouldUpdateInjectionTiming;
	for (size_t i = 0; i < calculationCylinderCount && i < MAX_CYLINDER_COUNT; i++) {
		fuelCandidate.CylinderFuelMasses[i] = cylinderFuelMasses[i];
	}

	bool publicationCurrent = false;
	{
		// Commit or clear every dependent fuel value under the same lock that
		// invalidates stale calculations and completes admission.
		chibios_rt::CriticalSectionLocker csl;
		auto admission = engine->airmassInjectionState.locked(csl);
		publicationCurrent = isFuelCaptureCurrentLocked(fuelCapture, admission);
		if (publicationCurrent) {
			fuelPublicationValid &= calculationCylinderCount == engine->engineState.cylinderCount;
#ifdef MODULE_CYLINDER_FUEL_TRIM
			if (fuelCandidate.ApplyCylinderFuelTrim) {
				fuelPublicationValid &=
						isCylinderFuelTrimCandidateValid(fuelCandidate.CylinderFuelTrim, calculationCylinderCount);
				if (fuelCandidate.CylinderFuelTrim.CalibrationGeneration != fuelCapture.CalibrationGeneration ||
					fuelCandidate.CylinderFuelTrim.ConfigurationVersion != fuelCapture.ConfigurationVersion) {
					publicationCurrent = false;
				}
			}
#endif // MODULE_CYLINDER_FUEL_TRIM
			if (fuelCapture.Standalone) {
				const bool loadShapeValid =
						fuelCandidate.ConsumerLoads.Prepared && fuelCandidate.ConsumerLoads.Snapshot.Valid;
				fuelPublicationValid &= loadShapeValid;
				if (loadShapeValid && !isAirmassConsumerLoadCandidateCurrent(fuelCandidate.ConsumerLoads, admission)) {
					// The fuel candidate itself may still be current while its load
					// identity was superseded. Treat that as a stale race, not as an
					// actual current fault that may clear newer fuel.
					publicationCurrent = false;
				}
			}
			if (publicationCurrent && fuelPublicationValid && fuelCapture.Standalone &&
				!publishAirmassConsumerLoads(fuelCandidate.ConsumerLoads, admission)) {
				// A newer publication won the race. Leave its loads and fuel intact.
				publicationCurrent = false;
			}
		}
		if (!publicationCurrent && fuelCapture.Standalone && fuelCapture.Strategy == LM_SD_ALPHA_N) {
			// Keep discarded calculations in their own evidence domain. They do
			// not own qualification faults or any accepted fuel output.
			engine->engineState.recordVeAnalyzeDiscardedCalculationLocked(fuelCapture.CalibrationGeneration);
		}
		if (publicationCurrent) {
			if (fuelPublicationValid) {
#ifdef MODULE_CYLINDER_FUEL_TRIM
				if (fuelCandidate.ApplyCylinderFuelTrim) {
					publishCylinderFuelTrim(*engine, fuelCandidate.CylinderFuelTrim);
				} else {
					// Disabled injection, fixed cranking and non-standalone models
					// are neutral. Clear any factors from the prior publication.
					resetCylinderFuelTrim(*engine);
				}
#endif // MODULE_CYLINDER_FUEL_TRIM
				airmassCalculationValid = true;
				baseFuel = fuelCandidate.BaseFuelMass;
				tpsAccelEnrich = fuelCandidate.TpsAccelEnrich;
				injectionStage2Fraction = fuelCandidate.Stage2Fraction;
				injectionDuration = fuelCandidate.InjectionDuration;
				injectionDurationStage2 = fuelCandidate.InjectionDurationStage2;
				injectionOffset = fuelCandidate.InjectionOffset;
				shouldUpdateInjectionTiming = fuelCandidate.ShouldUpdateInjectionTiming;

				engine->fuelComputer.sdAirMassInOneCylinder = fuelCandidate.SdAirMassInOneCylinder;
				engine->fuelComputer.normalizedCylinderFilling = fuelCandidate.NormalizedCylinderFilling;
				engine->engineState.fuelingLoad = fuelCandidate.FuelingLoad;
				engine->engineState.ignitionLoad = fuelCandidate.IgnitionLoad;
				engine->engineState.airflowEstimate = fuelCandidate.AirflowEstimate;
				engine->fuelComputer.afrTableYAxis = fuelCandidate.AfrTableYAxis;
				engine->fuelComputer.targetLambda = fuelCandidate.TargetLambda;
				engine->fuelComputer.targetAFR = fuelCandidate.TargetAFR;
				engine->fuelComputer.stoichiometricRatio = fuelCandidate.StoichiometricRatio;
				engine->fuelComputer.publishResolvedLambdaLoad(fuelCandidate.ResolvedLambdaLoad);
				engine->engineState.currentVe =
						fuelCandidate.Diagnostics.Ve.HasValue ? fuelCandidate.Diagnostics.Ve.Ve : 0;
				engine->engineState.veTableYAxis =
						fuelCandidate.Diagnostics.Ve.HasValue ? fuelCandidate.Diagnostics.Ve.Load : 0;
				engine->engineState.idleVeTableYAxis =
						fuelCandidate.Diagnostics.Ve.HasValue ? fuelCandidate.Diagnostics.Ve.IdleLoad : 0;

				if (isCranking) {
					engine->engineState.crankingFuel.coolantTemperatureCoefficient =
							fuelCandidate.CrankingCoolantTemperatureCoefficient;
					engine->engineState.crankingFuel.tpsCoefficient = fuelCandidate.CrankingTpsCoefficient;
					engine->engineState.crankingFuel.durationCoefficient = fuelCandidate.CrankingDurationCoefficient;
					engine->engineState.crankingFuel.fuel = fuelCandidate.CrankingFuel * 1000;
				} else {
					engine->fuelComputer.running.baseFuel = fuelCandidate.RunningBaseFuel;
					engine->fuelComputer.totalFuelCorrection = fuelCandidate.TotalFuelCorrection;
					engine->fuelComputer.running.fuel = fuelCandidate.RunningFuel * 1000;
				}

				for (size_t i = 0; i < STFT_BANK_COUNT; i++) {
					engine->stftCorrection[i] = clResult.banks[i];
				}
				for (size_t i = 0; i < calculationCylinderCount; i++) {
					engine->cylinders[i].setInjectionMass(fuelCandidate.CylinderFuelMasses[i]);
				}
			} else {
				// A current failed calculation closes admission and clears every
				// fuel output together; a stale one never reaches this branch.
				airmassCalculationValid = false;
				baseFuel = 0;
				tpsAccelEnrich = 0;
				injectionStage2Fraction = 0;
				injectionDuration = 0;
				injectionDurationStage2 = 0;
				injectionOffset = 0;
				shouldUpdateInjectionTiming = false;

				engine->fuelComputer.sdAirMassInOneCylinder = 0;
				engine->fuelComputer.normalizedCylinderFilling = 0;
				engine->engineState.fuelingLoad = 0;
				engine->engineState.ignitionLoad = 0;
				engine->engineState.airflowEstimate = 0;
				engine->fuelComputer.afrTableYAxis = 0;
				engine->fuelComputer.targetLambda = 0;
				engine->fuelComputer.targetAFR = 0;
				engine->fuelComputer.stoichiometricRatio = 0;
				engine->fuelComputer.publishResolvedLambdaLoad(0);
				engine->engineState.currentVe = 0;
				engine->engineState.veTableYAxis = 0;
				engine->engineState.idleVeTableYAxis = 0;
				engine->fuelComputer.running.baseFuel = 0;
				engine->fuelComputer.running.fuel = 0;
				engine->fuelComputer.totalFuelCorrection = 0;
				engine->engineState.crankingFuel.fuel = 0;
				for (size_t i = 0; i < STFT_BANK_COUNT; i++) {
					engine->stftCorrection[i] = 1;
				}
				for (auto& cylinder : engine->cylinders) {
					cylinder.setInjectionMass(0);
				}
#ifdef MODULE_CYLINDER_FUEL_TRIM
				resetCylinderFuelTrim(*engine);
#endif // MODULE_CYLINDER_FUEL_TRIM
				if (fuelCapture.Standalone) {
					invalidateAirmassConsumerLoadsLocked();
				}
			}

			engine->outputChannels.ignitionAdvance = nextIgnitionAdvance;
			for (size_t i = 0; i < calculationCylinderCount && i < MAX_CYLINDER_COUNT; i++) {
				engine->cylinders[i].setIgnitionTimingBtdc(cylinderTiming[i]);
			}
			if (fuelCapture.Standalone) {
				if (fuelCandidate.HasBlendedDiagnostics) {
					publishBlendedAirmassDiagnostics(fuelCandidate);
				} else {
					publishAirmassDiagnostics(fuelCandidate.Diagnostics);
				}
			}
			// Every physical fallback status feeds VE Analyze admission.
			admission.completeCalculation(fuelCalculation, fuelPublicationValid, fuelCandidate.Fault);
		}
	}

	if (publicationCurrent) {
		if (!fuelCapture.Standalone) {
			// Preserve the legacy computation even when spark outputs are disabled.
			const float trailingLoad = currentConsumerLoad(AirmassConsumer::TrailingSpark, nextFuelLoad);
			trailingSparkAngle = std::isfinite(trailingLoad) ? interpolate3d(
																	   config->trailingIgnitionTable,
																	   config->trailingIgnitionLoadBins,
																	   trailingLoad,
																	   config->trailingIgnitionRpmBins,
																	   rpm)
															 : 0;
			multispark.count = getMultiSparkCount(rpm);
		} else {
			const float trailingLoad = currentConsumerLoad(AirmassConsumer::TrailingSpark, 0);
			trailingSparkAngle = engineConfiguration->isIgnitionEnabled && engineConfiguration->enableTrailingSparks &&
												 std::isfinite(trailingLoad)
									   ? interpolate3d(
												 config->trailingIgnitionTable,
												 config->trailingIgnitionLoadBins,
												 trailingLoad,
												 config->trailingIgnitionRpmBins,
												 rpm)
									   : 0;
			multispark.count = engineConfiguration->isIgnitionEnabled ? getMultiSparkCount(rpm) : 0;
		}
	}

#if EFI_LAUNCH_CONTROL
	engine->launchController.update();
#endif // EFI_LAUNCH_CONTROL

#if EFI_ANTILAG_SYSTEM
	engine->antilagController.update();
#endif // EFI_ANTILAG_SYSTEM
#endif // EFI_ENGINE_CONTROL
}

void EngineState::updateTChargeK(float rpm, float tps) {
#if EFI_ENGINE_CONTROL
	float newTCharge = engine->fuelComputer.getTCharge(rpm, tps);
	if (!std::isnan(newTCharge)) {
		// control the rate of change or just fill with the initial value
		efitick_t nowNt = getTimeNowNt();
		float secsPassed = timeSinceLastTChargeK.getElapsedSeconds(nowNt);
		sd.tCharge = (sd.tChargeK == 0) ? newTCharge
										: limitRateOfChange(
												  newTCharge,
												  sd.tCharge,
												  engineConfiguration->tChargeAirIncrLimit,
												  engineConfiguration->tChargeAirDecrLimit,
												  secsPassed);
		sd.tChargeK = convertCelsiusToKelvin(sd.tCharge);
		timeSinceLastTChargeK.reset(nowNt);
	}
#endif
}

void EngineState::updateSplitInjection() {
	if (!requestSplitInjection) {
		doSplitInjection = false;
		return;
	}

	// toggle every 2 seconds
	if (splitInjectionTimer.hasElapsedSec(2)) {
		splitInjectionTimer.reset();

		doSplitInjection ^= true;
	}
}

void TriggerConfiguration::update() {
	VerboseTriggerSynchDetails = isVerboseTriggerSynchDetails();
	TriggerType = getType();
}

trigger_config_s PrimaryTriggerConfiguration::getType() const {
	return engineConfiguration->trigger;
}

bool PrimaryTriggerConfiguration::isVerboseTriggerSynchDetails() const {
	return engineConfiguration->verboseTriggerSynchDetails;
}

vvt_mode_e VvtTriggerConfiguration::getVvtMode() const {
	return engineConfiguration->vvtMode[m_index];
}

bool VvtTriggerConfiguration::needsTriggerDecoder() const {
	auto mode = getVvtMode();

	return mode != VVT_INACTIVE && mode != VVT_TOYOTA_3_TOOTH && mode != VVT_HONDA_K_INTAKE && mode != VVT_MAP_V_TWIN &&
		   mode != VVT_SINGLE_TOOTH;
}

// VVT decoding uses "normal" trigger shapes for decoding but is configured separately.
// This maps from vvt_mode_e -> trigger_type_e (for supported shapes)
static trigger_type_e getVvtTriggerType(vvt_mode_e vvtMode) {
	switch (vvtMode) {
		case VVT_INACTIVE:
			return trigger_type_e::TT_ONE;
		case VVT_MIATA_NB:
			return trigger_type_e::TT_VVT_MIATA_NB;
		case VVT_MIATA_NA:
			return trigger_type_e::TT_VVT_MIATA_NA;
		case VVT_BOSCH_QUICK_START:
			return trigger_type_e::TT_VVT_BOSCH_QUICK_START;
		case VVT_HONDA_K_EXHAUST:
			return trigger_type_e::TT_HONDA_K_CAM_4_1;
		case VVT_HONDA_K24Z_EXHAUST:
			return trigger_type_e::TT_HONDA_K24Z_CAM_3;
		case VVT_FORD_ST170:
			return trigger_type_e::TT_FORD_ST170;
		case VVT_BARRA_3_PLUS_1:
			return trigger_type_e::TT_VVT_BARRA_3_PLUS_1;
		case VVT_MAZDA_SKYACTIV:
			return trigger_type_e::TT_VVT_MAZDA_SKYACTIV;
		case VVT_MAZDA_L:
			return trigger_type_e::TT_VVT_MAZDA_L;
		case VVT_NISSAN_VQ:
			return trigger_type_e::TT_VVT_NISSAN_VQ35;
		case VVT_TOYOTA_4_1:
			return trigger_type_e::TT_VVT_TOYOTA_4_1;
		case VVT_MITSUBISHI_3A92:
			return trigger_type_e::TT_VVT_MITSUBISHI_3A92;
		case VVT_MITSUBISHI_6G75:
		case VVT_NISSAN_MR:
			return trigger_type_e::TT_NISSAN_MR18_CAM_VVT;
		case VVT_MITSUBISHI_4G9x:
			return trigger_type_e::TT_MITSU_4G9x_CAM;
		case VVT_MITSUBISHI_4G63:
			return trigger_type_e::TT_MITSU_4G63_CAM;
		case VVT_HONDA_J_6_2:
			return trigger_type_e::TT_HONDA_J_CAM_6_2;
		default:
			firmwareError("getVvtTriggerType for %s", getVvt_mode_e(vvtMode));
			return trigger_type_e::TT_ONE; // we have to return something for the sake of -Werror=return-type
	}
}

trigger_config_s VvtTriggerConfiguration::getType() const {
	if (!needsTriggerDecoder()) {
		return {trigger_type_e::TT_UNUSED, 0, 0};
	}

	// Convert from VVT type to trigger_config_s
	return {getVvtTriggerType(getVvtMode()), 0, 0};
}

bool VvtTriggerConfiguration::isVerboseTriggerSynchDetails() const {
	return engineConfiguration->verboseVVTDecoding;
}
