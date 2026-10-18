#pragma once

#include "airmass.h"

class MafAirmass final : public AirmassVeModelBase {
public:
	explicit MafAirmass(const ValueProvider3D* veTable = nullptr)
		: AirmassVeModelBase(veTable, LM_REAL_MAF) {}

	expected<AirmassResult> getAirmass(float rpm, bool postState) override;
	AirmassEvaluation getAirmassForFuel(float rpm) const;
	AirmassEvaluation evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation evaluateAirmass(const AirmassInputs& inputs, AirmassDiagnostics* diagnostics = nullptr) const;

	// Compute airmass based on caller-owned flow and captured engine speed/geometry.
	expected<AirmassResult> getAirmassImpl(float massAirFlow, float rpm) const;
	AirmassEvaluation evaluateAirmassImpl(
			float massAirFlow, const AirmassInputs& inputs, AirmassDiagnostics* diagnostics = nullptr) const;

private:
	AirmassEvaluation evaluateAirmassImpl(
			float massAirFlow, const AirmassInputs& inputs, bool postState, AirmassDiagnostics* diagnostics) const;
	expected<float> getMaf(const AirmassInputs& inputs) const;
};
