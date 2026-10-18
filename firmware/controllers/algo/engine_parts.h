/*
 * @file engine_parts.h
 *
 * @date May 27, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "static_vector.h"
#include "efi_timer.h"

class TransmissionState {
public:
	gear_e gearSelectorPosition;
};

struct warning_t {
	Timer LastTriggered;
	ObdCode Code = ObdCode::None;

	warning_t() {}

	explicit warning_t(ObdCode code)
		: Code(code) {}

	// Equality just checks the code, timer doesn't matter
	bool operator==(const warning_t& other) const {
		return other.Code == Code;
	}

	// Compare against a plain OBD code
	bool operator==(const ObdCode other) const {
		return other == Code;
	}
};

typedef static_vector<warning_t, 8> warningBuffer_t;

class WarningCodeState {
public:
	WarningCodeState();
	void addWarningCode(ObdCode code);
	bool isWarningNow() const;
	bool isWarningNow(ObdCode code) const;
	void clear();
	int warningCounter;
	ObdCode lastErrorCode = ObdCode::None;

	Timer timeSinceLastWarning;

	// todo: we need a way to post multiple recent warnings into TS
	warningBuffer_t recentWarnings;
};

struct multispark_state {
	efidur_t delay;
	efidur_t dwell;
	uint8_t count = 0;
};

// Dual-family per-cylinder model fuel trim publication. Owned by Engine so
// fixtures and address reuse cannot observe another engine's factors.
struct CylinderModelFuelTrimState {
	float Factor[MAX_CYLINDER_COUNT];
	uint8_t Branches;
	bool PublicationValid;

	CylinderModelFuelTrimState() {
		reset();
	}

	void reset() {
		for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
			Factor[i] = 1;
		}
		Branches = 0;
		PublicationValid = true;
	}
};

// Scan cache for the two model-trim families. Validity is tied to the captured
// airmass calibration generation, configuration version and cylinder count.
struct CylinderTrimCalibrationCache {
	uint32_t Generation;
	int ConfigurationVersion;
	size_t CylinderCount;
	uint16_t Known;
	uint16_t Valid;
#if EFI_UNIT_TEST
	uint32_t Scans;
#endif

	CylinderTrimCalibrationCache() {
		reset();
	}

	void reset() {
		Generation = 0xFFFFFFFFu;
		ConfigurationVersion = -1;
		CylinderCount = 0;
		Known = 0;
		Valid = 0;
#if EFI_UNIT_TEST
		Scans = 0;
#endif
	}
};
