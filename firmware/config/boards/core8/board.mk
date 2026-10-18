# List of all the board related files.
BOARDCPPSRC =  $(BOARD_DIR)/board_configuration.cpp

# Override DEFAULT_ENGINE_TYPE
SHORT_BOARD_NAME = core8
DDEFS += -DFIRMWARE_ID=\"core8\"
DDEFS += -DDEFAULT_ENGINE_TYPE=MINIMAL_PINS
# Core8 v2.3/v2.4 schematics: PG11 is LED3; PD14 drives Low Side 9.
DDEFS += -DLED_CRITICAL_ERROR_BRAIN_PIN=Gpio::G11

# The F407 has only 64k of CCM, and it must hold the calibration, the engine
# state and the 5 KiB main-loop stack. Keep CAN, USB console and USB mass-storage
# background objects in ordinary SRAM (see CCM_OPTIONAL_BACKGROUND).
DDEFS += -DEFI_RESERVE_CCM_FOR_CALIBRATION=TRUE

# Offset the expanded cylinder-model calibration by reducing Core8's Lua heap
# from the 30,000-byte F407 default while keeping full SD and USB MSD support.
DDEFS += -DLUA_USER_HEAP=26000
