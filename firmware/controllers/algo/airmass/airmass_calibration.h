/**
 * @file airmass_calibration.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "rusefi_enums.h"
#include "rusefi_types.h"

#include <cstdint>

struct AirmassInputs;

// Static calibration validity only. Sensor readings, policies and geometry are
// captured per evaluation and are never reused from this cache.
enum class AirmassCalibration : uint8_t {
	SpeedDensity,
	AlphaN,
	MafTrim,
	MapEstimate,
	Authority,
	Idle,
	Correction0,
};

// Runtime-only qualified endpoint for composite VE analysis. Zero means no
// usable endpoint; one and two select Speed Density and Alpha-N respectively.
enum class AirmassVeAnalyzeEndpoint : uint8_t {
	None,
	SpeedDensity,
	AlphaN,
};

// VE Analyze rejection evidence. Values are bits so the current rejection
// mask can preserve independent causes; the first reason stores one bit only.
enum class AirmassVeAnalyzeRejectReason : uint16_t {
	None = 0,
	NonUniformAuthority = 1 << 0,
	Configuration = 1 << 1,
	Sensor = 1 << 2,
	Fallback = 1 << 3,
	IdleVe = 1 << 4,
	CalculationResult = 1 << 5,
	StaleCalculation = 1 << 6,
	CalibrationWrite = 1 << 7,
	StrategyChange = 1 << 8,
	SessionAlreadyInvalid = 1 << 9,
	// Reserved for a future stabilization policy. Nothing sets this yet.
	Stabilization = 1 << 10,
};

constexpr uint16_t airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason reason) {
	return static_cast<uint16_t>(reason);
}

struct AirmassCalibrationCache {
	uint32_t Generation = 1;
	uint32_t Known = 0;
	uint32_t Valid = 0;
	AirmassVeAnalyzeEndpoint AuthorityEndpoint = AirmassVeAnalyzeEndpoint::None;
	bool AuthorityEndpointKnown = false;
	int ConfigurationVersion = -1;
#if EFI_UNIT_TEST
	uint32_t Scans = 0;
#endif
};

// Call before mutating any cached calibration axis. This does not change the
// global configuration version or perform runtime write/reset work.
void invalidateAirmassCalibration();

bool isCapturedAirmassCalibrationValid(const AirmassInputs& inputs, AirmassCalibration calibration);
bool isCapturedAirmassModelValid(const AirmassInputs& inputs, engine_load_mode_e model);
AirmassVeAnalyzeEndpoint getCapturedAirmassAuthorityEndpoint(const AirmassInputs& inputs);
