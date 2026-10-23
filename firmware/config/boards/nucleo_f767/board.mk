# List of all the board related files.

# F429 and F767 Nucleo are indeed the same board with a different chip fitted - so recycle the F429 config
BOARDCPPSRC = $(BOARDS_DIR)/nucleo_f429/board_configuration.cpp

# reducing flash consumption for EFI_ETHERNET to fit
DDEFS += -DEFI_ALTERNATOR_CONTROL=FALSE
override FOME_MODULES_BOARD_UNSUPPORTED += sd_card

DDEFS += -DLED_CRITICAL_ERROR_BRAIN_PIN=Gpio::B14

# Enable ethernet
LWIP = yes
include $(PROJECT_DIR)/controllers/modules/ethernet_console/ethernet_console.mk

DDEFS += -DFIRMWARE_ID=\"nucleo_f767\"
DDEFS += -DDEFAULT_ENGINE_TYPE=MINIMAL_PINS
