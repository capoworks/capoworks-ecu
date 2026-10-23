# @file sd_card.mk
# @author Giacomo di Ciocco, (C) 2026
#
# SD/FatFs sources are shared with existing hardware and mass-storage code and
# are gated by EFI_FILE_LOGGING. module_profile.mk maps this selection to that
# global flag and USE_FATFS; this recipe only exposes the conventional module
# marker.
DDEFS += -DMODULE_SD_CARD=TRUE
