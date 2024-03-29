###################################################################
# About the application name and path
###################################################################

# Application name, can be suffixed by the SDK
APP_NAME ?= dfuflash
APP_LDSCRIPT = $(patsubst -T%.ld,%.ld,$(filter -T%.ld, $(EXTRA_LDFLAGS)))
# application build directory name
DIR_NAME = dfuflash

# project root directory, relative to app dir
PROJ_FILES = ../../

# binary, hex and elf file names
BIN_NAME = $(APP_NAME).bin
HEX_NAME = $(APP_NAME).hex
ELF_NAME = $(APP_NAME).elf

# SDK helper Makefiles inclusion
-include $(PROJ_FILES)/m_config.mk
-include $(PROJ_FILES)/m_generic.mk

# application build directory, relative to the SDK BUILD_DIR environment
# variable.
APP_BUILD_DIR = $(BUILD_DIR)/apps/$(DIR_NAME)

###################################################################
# About the compilation flags
###################################################################

# SDK Cflags
CFLAGS := $(APPS_CFLAGS)
# Application CFLAGS...
CFLAGS += -Isrc -MMD -MP
# dfuflash needs private key access (for overencryption key access).
# This access should be declared voluntary in makefiles
CFLAGS += -I$(PRIVATE_DIR)/DFU/


###################################################################
# About the link step
###################################################################
# linker options to add the layout file
LDFLAGS += $(EXTRA_LDFLAGS) -L$(APP_BUILD_DIR)

# project's library you whish to use...
LD_LIBS += -laes -lcryp -lstd -lfirmware -lflash

ifeq (y,$(CONFIG_STD_DRBG))
LD_LIBS += -lhmac -lsign
endif

LD_LIBS += -Wl,--no-whole-archive

###################################################################
# okay let's list our source files and generated files now
###################################################################

CSRC_DIR = src
SRC = $(wildcard $(CSRC_DIR)/*.c)
OBJ = $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(SRC))
DEP = $(OBJ:.o=.d)

# the output directories, that will be deleted by the distclean target
OUT_DIRS = $(dir $(OBJ))

# the ldcript file generated by the SDK
LDSCRIPT_NAME = $(APP_BUILD_DIR)/$(APP_NAME).ld

# file to (dist)clean

# first, objects and compilation related
TODEL_CLEAN += $(OBJ) $(LDSCRIPT_NAME)

# the overall target content
TODEL_DISTCLEAN += $(APP_BUILD_DIR)

.PHONY: app

############################################################
# explicit dependency on the application libs and drivers
# compiling the application requires the compilation of its
# dependencies
###########################################################

## library dependencies
LIBDEP := $(BUILD_DIR)/libs/libstd/libstd.a \
          $(BUILD_DIR)/libs/libfirmware/libfirmware.a \
          $(BUILD_DIR)/libs/libaes/libaes.a

libdep: $(LIBDEP)

$(LIBDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)libs/$(patsubst lib%.a,%,$(notdir $@))


# drivers dependencies
SOCDRVDEP := $(BUILD_DIR)/drivers/libflash/libflash.a \
             $(BUILD_DIR)/drivers/libcryp/libcryp.a

socdrvdep: $(SOCDRVDEP)

$(SOCDRVDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)drivers/socs/$(SOC)/$(patsubst lib%.a,%,$(notdir $@))

# board drivers dependencies
BRDDRVDEP    :=

brddrvdep: $(BRDDRVDEP)

$(BRDDRVDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)drivers/boards/$(BOARD)/$(patsubst lib%.a,%,$(notdir $@))

# external dependencies
EXTDEP    :=

extdep: $(EXTDEP)

$(EXTDEP):
	$(Q)$(MAKE) -C $(PROJ_FILES)externals


alldeps: libdep socdrvdep brddrvdep extdep

##########################################################
# generic targets of all applications makefiles
##########################################################

show:
	@echo
	@echo "\t\tAPP_BUILD_DIR\t=> " $(APP_BUILD_DIR)
	@echo
	@echo "C sources files:"
	@echo "\t\tSRC\t=> " $(SRC)
	@echo "\t\tOBJ\t=> " $(OBJ)
	@echo "\t\tDEP\t=> " $(DEP)
	@echo
	@echo "\t\tCFG\t=> " $(CFLAGS)


# all (default) build the app
all: $(APP_BUILD_DIR) alldeps app

# Flash is an app using a dedicated section, named
# 'NOUPDATE'. This section hold the flash over encryption key.
# Although, this section is not mapped by the task itself, but by the
# bootloader, which is responsible for copying the encrypted keybag
# from the NOUPDATE section to the SecureRAM.
# Smart access the keybag in the secureRAM directly.
# The goal, here, is to allow firmware upgrade without requiring the
# private AUTH key knowledge. Only the SIG key is required to build
# a fully functional firmware. To do this, the generated firmware image
# must be truncated of the NOUPGRADE secion, which should never be updated
#
# Here, we add NOUPGRADE memory layout and .noupgrade section to the
# generic app ldscripts before compiling and linking
#
update_ld:
	sed '/^$$/d' -i $(APP_BUILD_DIR)/$(APP_LDSCRIPT)
	sed -f update_ld.sed -i $(APP_BUILD_DIR)/$(APP_LDSCRIPT)

# app build the hex and elf binaries
app: update_ld $(APP_BUILD_DIR)/$(ELF_NAME) $(APP_BUILD_DIR)/$(HEX_NAME)

# objet files and dependencies
$(APP_BUILD_DIR)/%.o: %.c
	$(call if_changed,cc_o_c)

# ELF file dependencies. libs are build separately and before.
# Be sure to add the libs to your config file!
$(APP_BUILD_DIR)/$(ELF_NAME): $(OBJ)
	$(call if_changed,link_o_target)

CROSS_OBJCOPY_ARGS="--keep-section=.noupgrade.dfu_flash_key_iv"
# same for hex
$(APP_BUILD_DIR)/$(HEX_NAME): $(APP_BUILD_DIR)/$(ELF_NAME)
	$(call if_changed,objcopy_ihex)

# same for bin. bin is not build but you can add it if you whish
$(APP_BUILD_DIR)/$(BIN_NAME): $(APP_BUILD_DIR)/$(ELF_NAME)
	$(call if_changed,objcopy_bin)

# special target to create the application build directory
$(APP_BUILD_DIR):
	$(call cmd,mkdir)


-include $(DEP)
