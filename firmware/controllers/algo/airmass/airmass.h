#pragma once

#include "rusefi_types.h"
#include "rusefi_enums.h"
#include "airmass_calibration.h"

class ValueProvider3D;

struct AirmassResult {
	mass_t CylinderAirmass = 0;
	percent_t EngineLoadPercent = 100;
};

struct AirmassEvaluation {
	AirmassResult Result;
	bool Valid = false;
	bool Degraded = false;
};

struct MapEvaluation {
	float Map = 0;
	float FallbackMap = 0;
	bool HasValue = false;
	bool Valid = false;
	bool UsesEstimate = false;
	bool Fallback = false;
};

// Captured once per physical-model evaluation. Calibration tables remain in
// configuration storage; this snapshot contains only the inputs and policies
// needed by the core model math.
struct AirmassInputs {
	uint32_t PublicationEpoch = 0;
	int ConfigurationVersion = 0;
	engine_load_mode_e ActiveStrategy = LM_SPEED_DENSITY;
	uint32_t CalibrationGeneration = 0;
	bool HasPublicationContext = false;
	bool LiveCalibration = false;

	float Rpm = 0;
	expected<float> MeasuredMap = unexpected;
	MapEvaluation NativeMap;
	expected<float> Tps = unexpected;
	expected<float> Iat = unexpected;
	expected<float> BarometricPressure = unexpected;
	expected<float> DriverThrottleIntent = unexpected;
	expected<float> Pedal = unexpected;
	expected<float> Maf = unexpected;
	expected<float> Maf2 = unexpected;
	expected<float> FuelingLoad = unexpected;
	expected<float> IgnitionLoad = unexpected;
	bool HasSecondaryMaf = false;
	float TemperatureK = 0;
	bool TemperatureValid = false;
	bool TemperatureFallback = false;
	AirmassTemperatureSource TemperatureSource = AirmassTemperatureSource::Tcharge;

	bool UseMapEstimateTable = false;
	bool MapEstimateTransient = false;
	bool AlphaNMultiplyMap = false;
	bool AlphaNBaroCompensation = false;
	float AlphaNBaroReferencePressure = 101.325f;
	IdleVeLoadSource IdleLoadSource = IdleVeLoadSource::ModelDefault;
	bool UseSeparateVeForIdle = false;
	float IdlePidDeactivationTpsThreshold = 0;
	ve_override_e VeOverrideMode = VE_None;
	bool IdleActive = false;
	bool TwoStroke = false;
	float TpsToleranceMin = -10;
	float TpsToleranceMax = 110;

	float Displacement = 0;
	float CylinderCount = 0;
};

struct VeDiagnostics {
	percent_t Ve = 0;
	float Load = 0;
	float IdleLoad = 0;
	bool HasValue = false;
	bool Valid = false;
	bool IdleFallback = false;
	bool CorrectionFallback = false;
};

struct AirmassDiagnostics {
	float TemperatureK = 0;
	bool TemperatureValid = false;
	bool TemperatureFallback = false;
	AirmassTemperatureSource TemperatureSource = AirmassTemperatureSource::Tcharge;

	float BaroCoefficient = 1;
	bool BaroFallback = false;
	MapEvaluation Map;
	VeDiagnostics Ve;
};

bool isAirmassModelConfigurationValid(engine_load_mode_e model);
bool isMapEstimateConfigurationValid();
bool isMapEstimateAxesValid();
bool isAirmassConfigurationValid();
bool validateAirmassConfiguration();

void captureAirmassInputs(
		float rpm,
		AirmassInputs& inputs,
		const ValueProvider3D* estimateTable = nullptr,
		bool resolveMap = true,
		bool liveCalibration = false);
void resolveCapturedMap(AirmassInputs& inputs, const ValueProvider3D* estimateTable = nullptr);
void evaluateAirmassTemperature(AirmassDiagnostics& diagnostics);
void evaluateAirmassTemperature(const AirmassInputs& inputs, AirmassDiagnostics& diagnostics);
MapEvaluation evaluateAirmassMap(float rpm, const ValueProvider3D* estimateTable = nullptr);
void publishAirmassDiagnostics(const AirmassDiagnostics& diagnostics);
expected<float> normalizeAirmassPercent(const AirmassInputs& inputs, expected<float> sensor);

struct AirmassModelBase {
	virtual expected<AirmassResult> getAirmass(float rpm, bool postState) = 0;
};

class AirmassVeModelBase : public AirmassModelBase {
public:
	explicit AirmassVeModelBase(const ValueProvider3D* veTable, engine_load_mode_e model = LM_SPEED_DENSITY);

	// Retrieve the user-calibrated volumetric efficiency from the table
	float getVe(float rpm, percent_t load, bool postState, VeTableType consumer) const;
	VeDiagnostics evaluateVe(
			const AirmassInputs& inputs,
			float load,
			bool postState,
			VeTableType consumer,
			VeDiagnostics* diagnostics = nullptr) const;
	VeDiagnostics
	evaluateVe(float rpm, percent_t load, bool postState, VeTableType consumer, VeDiagnostics* diagnostics = nullptr)
			const;

	virtual float getVeImpl(float /*rpm*/, percent_t /*load*/) const;

protected:
	VeDiagnostics evaluateRawVe(const AirmassInputs& inputs, float load, VeDiagnostics* diagnostics = nullptr) const;

private:
	const ValueProvider3D* const m_veTable;
	const engine_load_mode_e m_model;
};
