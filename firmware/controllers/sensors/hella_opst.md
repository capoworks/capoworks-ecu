# Hella OPS+T oil pressure and temperature sensor

## Supported sensor

This driver supports the Hella OPS+T combined digital oil pressure and
temperature sensor, part number `6PR 010 378-207`.

The sensor transmits a repeating three-symbol pulse-width frame on one digital
line:

1. diagnosis and frame synchronization;
2. oil temperature;
3. absolute oil pressure.

The input must be assigned to a pin that supports EXTI edge capture. Follow the
Hella datasheet for the electrical connection, supply voltage, connector pinout,
and any required input protection.

## Configuration

In TunerStudio, open the oil-pressure sensor settings and assign **Hella OPS+T
input** to the connected digital input pin. Leaving the pin unassigned disables
the driver.

The driver registers the standard `OilPressure` and `OilTemperature` sensor
channels only when those channels do not already have providers. This permits
mixed configurations and prevents OPS+T from replacing an existing analog or
digital sensor. If both channels already have providers, the driver does not
claim the EXTI input.

## Decoding

The synchronization symbol has a nominal period of 1,024 microseconds. The two
data symbols have a nominal period of 4,096 microseconds. A 10 percent period
tolerance is accepted. Pulse width is normalized against the measured period to
compensate for sensor oscillator tolerance.

For a normalized data pulse width `pulse` in microseconds, the decoded values
are:

```text
temperature_degC = ((pulse - 128) / 19.2) - 40
absolute_pressure_kPa = (((pulse - 128) / 384) + 0.5) * 100
```

FOME exposes oil pressure as gauge pressure. The driver therefore subtracts the
current barometric-pressure sensor value and clamps the result to zero. If BARO
is unavailable, it uses 101.325 kPa.

Data pulses below 128 microseconds or above 3,968 microseconds invalidate the
corresponding channel. The diagnosis symbol can independently report pressure,
temperature, or sensor-hardware faults. A valid frame must start with a valid
synchronization symbol, and invalid timing resets frame synchronization.

Both output channels use a 500 ms timeout. Sensor-checker range/performance DTCs
are `P0521` for inconsistent oil pressure and `P0196` for inconsistent oil
temperature.

## Implementation layout

- `hella_opst.cpp` and `hella_opst.h` contain the protocol decoder and have no
  direct dependency on engine configuration.
- `init/sensor/init_hella_opst.cpp` owns the global instance and binds the
  configured pin during sensor initialization.
- EXTI acquisition reports success or failure so partial initialization can be
  rolled back without unregistering providers owned by another driver.

## Validation status

The decoder, diagnostics, timeout, resynchronization, BARO conversion, provider
priority, and reinitialization behavior are covered by unit tests. The Core8 ARM
firmware and bootloader also compile with the driver enabled.

Validation on the physical sensor and a running engine is still required before
the implementation should be considered production-tested.

## Reference

- [Hella oil pressure and temperature sensor product information](https://www.hella.com/forvia-us/assets/documents/BI_Oil_Pressure_and_Temperature_Sensor_2026.pdf)
