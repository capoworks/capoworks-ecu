/**
 * @file blended_airmass.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "airmass.h"
#include "airmass_injection_state.h"

class SpeedDensityAirmass;
class AlphaNAirmass;

enum BlendedAirmassFlags : uint16_t {
	BlendedSdEvaluated = 1 << 0,
	BlendedAlphaNEvaluated = 1 << 1,
	BlendedSdValid = 1 << 2,
	BlendedAlphaNValid = 1 << 3,
	BlendedMapEstimateUsed = 1 << 4,
	BlendedCalculationValid = 1 << 5,
	BlendedEstimateEvaluated = 1 << 6,
	BlendedTemperatureFallback = 1 << 7,
	BlendedBranchFallback = 1 << 8,
	BlendedAuthorityUnavailable = 1 << 9,
	BlendedBaroFallback = 1 << 10,
	BlendedIdleFallback = 1 << 11,
	BlendedCorrectionFallback = 1 << 12,
	BlendedMapFallback = 1 << 13,
	BlendedAuthorityLoadValid = 1 << 14,
};

struct BlendedAirmassEvaluation {
	AirmassEvaluation Airmass;
	// Strict native coordinate: effective MAP regardless of blending weight.
	AirmassLoad NativeLoad;
	// Strict selector provenance remains separate from the downstream numeric
	// substitution exposed by LambdaLoad and IgnitionLoad.
	AirmassLoad LambdaLoadStrict;
	AirmassLoad IgnitionLoadStrict;
	AirmassLoad LambdaLoad;
	AirmassLoad IgnitionLoad;
	float NormalizedFilling = 0;
	AirmassInjectionFault Fault = AirmassInjectionFault::None;
};

// Caller-owned, full-precision model evidence. No field is an output-channel
// packed value, and evaluation never publishes or mutates live state.
struct BlendedAirmassDiagnostics {
	AirmassInputs Inputs;
	float TemperatureK = 0;
	bool TemperatureValid = false;
	bool TemperatureFallback = false;
	AirmassTemperatureSource TemperatureSource = AirmassTemperatureSource::Tcharge;
	float BaroCoefficient = 1;
	AirmassBlendLoadSource AuthorityLoadSource = AirmassBlendLoadSource::Tps;
	float AuthorityLoad = 0;
	bool AuthorityLoadValid = false;
	float SdMass = 0;
	float AlphaNMass = 0;
	float SdWeight = 0;
	float AlphaNWeight = 0;
	float RawMass = 0;
	float CorrectionMultiplier = 1;
	float FinalMass = 0;
	RawAirmassDiagnostics Sd;
	RawAirmassDiagnostics AlphaN;
	MapEvaluation Map;
	VeCorrectionDiagnostics Corrections;
	float RequestedAuthority = 0;
	float EffectiveAuthority = 0;
	AirmassInjectionFault Fault = AirmassInjectionFault::None;
	uint16_t Flags = 0;
};

bool isBlendedAirmassConfigurationValid();

class BlendedAirmass final : public AirmassModelBase {
public:
	BlendedAirmass(const SpeedDensityAirmass& sd, const AlphaNAirmass& alphaN)
		: m_sd(sd)
		, m_alphaN(alphaN) {}

	// Compatibility wrappers capture one caller snapshot and delegate to the
	// pure snapshot evaluator. They intentionally never publish live state.
	expected<AirmassResult> getAirmass(float rpm, bool postState) override;
	BlendedAirmassEvaluation evaluateAirmass(float rpm, BlendedAirmassDiagnostics* diagnostics = nullptr) const;
	BlendedAirmassEvaluation
	evaluateAirmass(const AirmassInputs& inputs, BlendedAirmassDiagnostics* diagnostics = nullptr) const;
	void captureInputs(float rpm, AirmassInputs& inputs, bool liveCalibration = false) const;

private:
	const SpeedDensityAirmass& m_sd;
	const AlphaNAirmass& m_alphaN;
};
