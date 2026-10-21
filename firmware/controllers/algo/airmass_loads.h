/**
 * @file airmass_loads.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "airmass.h"
#include "airmass_injection_state.h"
#include "rusefi_types.h"

#include <rusefi/expected.h>

struct output_channels_s;

enum class AirmassConsumer : uint8_t {
	InjectionPhase,
	IgnitionTrim,
	Stft,
	Staging,
	LambdaDeviation,
	LambdaMonitor,
	TrailingSpark,
	IgnitionIat,
	KnockRetard,
	KnockGain,
	HpfpTarget,
#ifdef MODULE_CYLINDER_FUEL_TRIM
	CylinderFuelTrim,
#endif // MODULE_CYLINDER_FUEL_TRIM
	Count
};

using AirmassConsumerMask = uint16_t;

#ifdef MODULE_CYLINDER_FUEL_TRIM
// Cylinder fuel trim coordinates are published as dedicated telemetry rather
// than packed consumer cursors. Keeping them out of the display array avoids
// growing AirmassConsumerLoadCandidate beyond its existing static bound.
constexpr size_t AIRMASS_PACKED_CONSUMER_COUNT = static_cast<size_t>(AirmassConsumer::CylinderFuelTrim);
#else
constexpr size_t AIRMASS_PACKED_CONSUMER_COUNT = static_cast<size_t>(AirmassConsumer::Count);
#endif // MODULE_CYLINDER_FUEL_TRIM

// Full precision control values. Packed output channels are display only and
// must never be read back as control inputs.
struct AirmassLoadSnapshot {
	float Native = 0;
	float Map = 0;
	float EffectiveMap = 0;
	float Tps = 0;
	float Pedal = 0;
	float Filling = 0;
	AirmassConsumerMask EnabledConsumers = 0;
	uint8_t ValidSources = 0;
	bool Valid = false;
	bool UsesEstimate = false;
	bool RequiredFallbackUsed = false;
	bool LambdaTargetRequired = true;

	AirmassInjectionState::CalculationToken PublicationEpoch = 0;
	int ConfigurationVersion = -1;
	uint32_t CalibrationGeneration = 0;
	engine_load_mode_e Strategy = LM_SPEED_DENSITY;
	uint8_t CylinderCount = 0;
	float Displacement = 0;
};

struct AirmassConsumerLoadIdentity {
	AirmassInjectionState::CalculationToken PublicationEpoch = 0;
	int ConfigurationVersion = -1;
	uint32_t CalibrationGeneration = 0;
	engine_load_mode_e Strategy = LM_SPEED_DENSITY;
	uint8_t CylinderCount = 0;
	float Displacement = 0;
	uint32_t StateInvalidationEpoch = 0;
	uint32_t PreviousPublicationSerial = 0;
};

// Prepared outside the publication lock from one immutable captured input set.
// DisplayCursors stores packed 0.1-scaled values so the final commit is a copy.
struct AirmassConsumerLoadCandidate {
	AirmassLoadSnapshot Snapshot;
	AirmassConsumerLoadIdentity Identity;
	load_override_e Sources[static_cast<size_t>(AirmassConsumer::Count)][MAX_CYLINDER_COUNT] = {};
	int16_t DisplayCursors[AIRMASS_PACKED_CONSUMER_COUNT][MAX_CYLINDER_COUNT] = {};
	bool Prepared = false;
};

static_assert(sizeof(AirmassConsumerLoadCandidate) <= 512);

// Immutable per-calculation view for cylinder loops. It captures selectors and
// coordinates once. Existing getters keep returning the captured values after a
// configuration change; callers that need publication safety use isCurrent().
class AirmassConsumerLoadContext {
public:
	AirmassConsumerLoadContext(AirmassConsumer first, AirmassConsumer second = AirmassConsumer::Count);
	// Legacy context with the consumer's historical AFR_None default. All other
	// sources use the live sensor fallback captured by this context.
	AirmassConsumerLoadContext(AirmassConsumer consumer, float legacyDefault);
	// Candidate-backed context for the calculation currently being evaluated.
	// The candidate must remain alive for the lifetime of this context. It keeps
	// the candidate's full-precision coordinates and captured selectors even if
	// configuration changes or a newer candidate replaces it before publication.
	explicit AirmassConsumerLoadContext(const AirmassConsumerLoadCandidate& candidate);

	float get(AirmassConsumer consumer, size_t index = 0) const;
	// Resolve the captured legacy AFR/ignition override selector using the same
	// immutable candidate snapshot.
	float getSelectedLoad(load_override_e source) const;
	bool isCurrent() const;

private:
	AirmassConsumerLoadContext(
			AirmassConsumer first, AirmassConsumer second, float firstLegacyDefault, float secondLegacyDefault);

	float m_loads[AFR_EffectiveMAP + 1] = {};
	float m_hpfpMap = NAN;
	load_override_e m_sources[2][MAX_CYLINDER_COUNT] = {};
	AirmassConsumerLoadIdentity m_identity;
	AirmassConsumer m_first;
	AirmassConsumer m_second;
	uint8_t m_validSources = 0;
	bool m_hasSnapshot = false;
	bool m_legacy = false;
	const AirmassConsumerLoadCandidate* m_candidate = nullptr;
};

static_assert(sizeof(AirmassConsumerLoadContext) <= 160);

AirmassConsumerLoadCandidate prepareAirmassConsumerLoads(const AirmassInputs& inputs, mass_t finalMass);
bool isAirmassConsumerLoadCandidateCurrent(
		const AirmassConsumerLoadCandidate& candidate, AirmassInjectionState::LockedAccess& admission);
bool publishAirmassConsumerLoads(
		const AirmassConsumerLoadCandidate& candidate, AirmassInjectionState::LockedAccess& admission);
void invalidateAirmassConsumerLoads();
void invalidateAirmassConsumerLoadsLocked();
void clearAirmassConsumerCursorFields(output_channels_s& outputs);

float getAirmassConsumerLoad(AirmassConsumer consumer, size_t index = 0);
float getAirmassSelectedLoad(load_override_e source, float legacyDefault);
expected<float> getEffectiveAirmassMap();
bool isAirmassLambdaTargetRequired();
bool hasAirmassLoadFallback();
