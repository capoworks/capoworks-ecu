#include "pch.h"

#include "maf_airmass.h"
#include "maf.h"
#include "speed_density_base.h"

expected<float> MafAirmass::getMaf(const AirmassInputs& inputs) const {
	auto maf = inputs.Maf;
	if (maf && (!std::isfinite(maf.Value) || maf.Value < 0)) {
		maf = unexpected;
	}

	if (inputs.HasSecondaryMaf) {
		auto maf2 = inputs.Maf2;
		if (maf2 && (!std::isfinite(maf2.Value) || maf2.Value < 0)) {
			maf2 = unexpected;
		}

		if (maf && maf2) {
			// Both MAFs work, return the sum
			return maf.Value + maf2.Value;
		} else if (maf) {
			// MAF 1 works, but not MAF 2, so double the value from #1
			return 2 * maf.Value;
		} else if (maf2) {
			// MAF 2 works, but not MAF 1, so double the value from #2
			return 2 * maf2.Value;
		} else {
			// Both MAFs are broken, give up.
			return unexpected;
		}
	} else {
		return maf;
	}
}

expected<AirmassResult> MafAirmass::getAirmass(float rpm, bool postState) {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	const auto maf = getMaf(inputs);
	if (!maf) {
		return unexpected;
	}
	const auto evaluation = evaluateAirmassImpl(maf.Value, inputs, postState, nullptr);
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

AirmassEvaluation MafAirmass::getAirmassForFuel(float rpm) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false, true);
	const auto maf = getMaf(inputs);
	if (!maf) {
		return {};
	}
	return evaluateAirmassImpl(maf.Value, inputs, true, nullptr);
}

AirmassEvaluation MafAirmass::evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	return evaluateAirmass(inputs, diagnostics);
}

AirmassEvaluation MafAirmass::evaluateAirmass(const AirmassInputs& inputs, AirmassDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}
	const auto maf = getMaf(inputs);
	if (!maf) {
		return {};
	}
	return evaluateAirmassImpl(maf.Value, inputs, false, diagnostics);
}

/**
 * Convert measured mass airflow into a standardised cylinder-airmass load,
 * then apply the upstream MAF trim calibration.
 */
expected<AirmassResult> MafAirmass::getAirmassImpl(float massAirFlow, float rpm) const {
	AirmassInputs inputs;
	captureAirmassInputs(rpm, inputs, nullptr, false);
	const auto evaluation = evaluateAirmassImpl(massAirFlow, inputs, false, nullptr);
	return evaluation.Valid ? expected<AirmassResult>(evaluation.Result) : expected<AirmassResult>(unexpected);
}

AirmassEvaluation
MafAirmass::evaluateAirmassImpl(float massAirFlow, const AirmassInputs& inputs, AirmassDiagnostics* diagnostics) const {
	return evaluateAirmassImpl(massAirFlow, inputs, false, diagnostics);
}

AirmassEvaluation MafAirmass::evaluateAirmassImpl(
		float massAirFlow, const AirmassInputs& inputs, bool /*postState*/, AirmassDiagnostics* diagnostics) const {
	if (diagnostics) {
		*diagnostics = {};
	}
	AirmassEvaluation evaluation;
	if (!std::isfinite(inputs.Rpm) || inputs.Rpm <= 0 || !std::isfinite(massAirFlow) || massAirFlow < 0 ||
		!std::isfinite(inputs.Displacement) || inputs.Displacement <= 0 || !std::isfinite(inputs.CylinderCount) ||
		inputs.CylinderCount <= 0 ||
		!(inputs.LiveCalibration ? isCapturedAirmassModelValid(inputs, LM_REAL_MAF)
								 : isAirmassModelConfigurationValid(LM_REAL_MAF))) {
		return evaluation;
	}

	const float standardCharge = idealGasLaw(inputs.Displacement / inputs.CylinderCount, 101.325f, 273.15f + 20.0f);
	if (!std::isfinite(standardCharge) || standardCharge <= 0) {
		return evaluation;
	}

	// kg/hr -> g/s
	float gramPerSecond = massAirFlow / 3.6f;

	// 1/min -> 1/s
	float revsPerSecond = inputs.Rpm / 60.0f;
	mass_t airPerRevolution = gramPerSecond / revsPerSecond;

	// Now we have to divide among cylinders - on a 4 stroke, half of the cylinders happen every revolution
	// This math is floating point to work properly on engines with odd cylinder count
	float halfCylCount = inputs.CylinderCount / 2.0f;

	mass_t cylinderAirmass = airPerRevolution / halfCylCount;

	// Create % load for fuel table using relative naturally aspirated cylinder filling
	float airChargeLoad = 100 * cylinderAirmass / standardCharge;
	if (!std::isfinite(cylinderAirmass) || cylinderAirmass < 0 || !std::isfinite(airChargeLoad) || airChargeLoad < 0) {
		return evaluation;
	}

	// Correct air mass by MAF trim table
	const float trim = interpolate3d(
			config->mafTrimTable, config->mafTrimLoadBins, airChargeLoad, config->mafTrimRpmBins, inputs.Rpm);
	if (!std::isfinite(trim) || trim < 0 || trim > 200) {
		return evaluation;
	}
	mass_t correctedAirmass = cylinderAirmass * trim * PERCENT_DIV;

	evaluation.Result = {
			correctedAirmass,
			airChargeLoad, // AFR/VE/ignition table Y axis
	};
	const bool primaryUsable = inputs.Maf.Valid && std::isfinite(inputs.Maf.Value) && inputs.Maf.Value >= 0;
	const bool secondaryUsable = !inputs.HasSecondaryMaf ||
								 (inputs.Maf2.Valid && std::isfinite(inputs.Maf2.Value) && inputs.Maf2.Value >= 0);
	evaluation.Degraded = !primaryUsable || !secondaryUsable;
	evaluation.Valid = std::isfinite(correctedAirmass) && correctedAirmass >= 0;
	if (!evaluation.Valid) {
		evaluation.Result.CylinderAirmass = 0;
	}
	return evaluation;
}
