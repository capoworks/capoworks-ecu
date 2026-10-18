#pragma once

#include "speed_density_base.h"

class SpeedDensityAirmass : public SpeedDensityBase {
public:
	explicit SpeedDensityAirmass(const ValueProvider3D* veTable, const ValueProvider3D& mapEstimationTable)
		: SpeedDensityBase(veTable)
		, m_mapEstimationTable(&mapEstimationTable) {}

	expected<AirmassResult> getAirmass(float rpm, bool postState) override;
	expected<AirmassResult> getAirmass(float rpm, float map, bool postState);
	AirmassEvaluation getAirmassForFuel(float rpm) const;
	AirmassEvaluation evaluateAirmass(float rpm, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation evaluateAirmass(float rpm, float map, AirmassDiagnostics* diagnostics = nullptr) const;
	AirmassEvaluation evaluateAirmass(const AirmassInputs& inputs, AirmassDiagnostics* diagnostics = nullptr) const;
	float getAirflow(float rpm, float map, bool postState);
	float getVeImpl(float rpm, percent_t load) const override;

	float getMap(float rpm, bool postState) const;
	MapEvaluation evaluateMap(float rpm) const;
	void captureInputs(float rpm, AirmassInputs& inputs, bool liveCalibration = false) const;
	AirmassEvaluation
	evaluateRawAirmass(const AirmassInputs& inputs, RawAirmassDiagnostics* diagnostics = nullptr) const;

private:
	AirmassEvaluation
	evaluateAirmass(const AirmassInputs& inputs, bool postState, AirmassDiagnostics* diagnostics) const;
	const ValueProvider3D* const m_mapEstimationTable;
};
