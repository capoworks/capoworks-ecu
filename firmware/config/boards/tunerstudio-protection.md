# TunerStudio protection for board configuration

`TunerStudio::handleWriteChunkCommand()` calls `setBoardConfigOverrides()` after
each write. Editing a setting replaced by an override therefore changes the bytes
back before TunerStudio checks their CRC. Keep each firmware override and the
board's `prepend.txt` in sync so the UI does not offer an impossible change.

Declare fixed settings using their generated TunerStudio names:

```
#define ts_readonly_idle_stepperDirectionPin 1
#define ts_readonly_vbattAdcChannel 1
```

The generator emits `readOnly` in `[ConstantsExtensions]` and disables every
`field` referencing that setting, including pinout and feature dialogs.
TunerStudio uses the ECU value when loading a calibration. Unknown or stale
constant names fail generation. These declarations do not change configuration
offsets or the wire protocol.

An override that applies only to certain hardware revisions uses a quoted
TunerStudio expression that is true when the setting is fixed:

```
! AlphaX 2chan rev.D (107) does not reserve SPI1 for SD.
#define ts_readonly_spi1mosiPin "hellenBoardId != 107"
```

`readOnly` cannot vary at runtime. For conditional settings the generator emits
`controllerPriority` and combines the condition with each field's existing enable
expression. It preserves connected ECU values on tune import on all hardware
revisions, including revisions where the field remains editable. On a revision
where a pin is free, configure it via its dialog after import. Offline controls
cannot identify the board revision, so connect to the ECU before configuring
these settings.

## Battery-divider calibration

Boards with a fixed battery-voltage divider expose an opt-in
`enableBatteryDividerOverride` checkbox. With it off, the board applies its fixed
divider. Enable it only for the rare case that requires manual calibration while
connected; clearing it restores the board's nominal coefficient. The coefficient
uses `controllerPriority`, so tune import keeps the ECU value even when the
checkbox is enabled. Edit the coefficient after connecting to change it.

See the [EFI Analytics definition-file specification, section 6](https://www.efianalytics.com/TunerStudio/docs/EFI%20Analytics%20ECU%20Definition%20files.pdf).
