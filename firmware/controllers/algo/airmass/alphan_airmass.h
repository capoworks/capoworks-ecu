#pragma once

#include "speed_density_base.h"

enum class AlphaNPressurePolicy {
	PureReference,
	EffectiveMap,
};

class AlphaNAirmass : public SpeedDensityBase {
public:
	explicit AlphaNAirmass(const ValueProvider3D* veTable = nullptr)
		: SpeedDensityBase(veTable, LM_ALPHA_N) {}

	expected<AirmassResult> getAirmass(float rpm, bool postState) override;
	AirmassEvaluation getAirmassForFuel(float rpm) const;
	AirmassEvaluation evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation
	evaluateAirmass(float rpm, AlphaNPressurePolicy policy, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation evaluateAirmass(
			const AirmassInputs& inputs, AlphaNPressurePolicy policy, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation evaluateRawAirmass(
			const AirmassInputs& inputs,
			RawAirmassDiagnostics* diagnostics = nullptr,
			AlphaNPressurePolicy pressurePolicy = AlphaNPressurePolicy::PureReference) const;
	float getVeImpl(float rpm, percent_t load) const override;

private:
	void
	captureInputs(float rpm, AlphaNPressurePolicy policy, AirmassInputs& inputs, bool liveCalibration = false) const;
	AirmassEvaluation evaluateAirmass(
			const AirmassInputs& inputs,
			AlphaNPressurePolicy policy,
			bool postState,
			AirmassDiagnostics* diagnostics) const;
};
