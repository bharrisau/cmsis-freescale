_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

IDIR += $(realpath $(_DIR)/Include) $(realpath $(_DIR)/../peripheral)
SRC  += $(_DIR)Source/system_MKL25Z.c $(_DIR)Source/GCC/startup_MKL25Z.S
LSCRIPT += $(_DIR)Source/GCC/$(DEVICE).ld