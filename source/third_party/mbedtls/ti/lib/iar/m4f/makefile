# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
AR = "$(IAR_ARMCOMPILER)/bin/iarchive"

IAR_M4F_CFLAGS = "-I$(IAR_ARMCOMPILER)/inc" --silent --aeabi --cpu=Cortex-M4F \
        --diag_suppress=Pa050,Go005 --endian=little -e --fpu=VFPv4_sp --thumb

CFLAGS_DEFAULT = $(INCS) $(IAR_M4F_CFLAGS)
CFLAGS_MSP432E4 = $(INCS_MSP432E4) $(IAR_M4F_CFLAGS)

DEFAULT_OBJ_DIR = default
OBJS_DEFAULTDIR = $(addprefix $(DEFAULT_OBJ_DIR)/,$(OBJS))

MSP432E4_OBJ_DIR = msp432e4
OBJS_MSP432E4DIR = $(addprefix $(MSP432E4_OBJ_DIR)/,$(OBJS_MSP432E4))

vpath %.c ../../../../library ../../../port

$(DEFAULT_OBJ_DIR)/%.obj: %.c | $(DEFAULT_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_IAR) $(CFLAGS_DEFAULT) $< -o $@

$(MSP432E4_OBJ_DIR)/%.obj: %.c | $(MSP432E4_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_IAR) $(CFLAGS_MSP432E4) $< -o $@

# To enable a wide set of users, only the hardware-independent library
# is built from the default "all" build goal.  Users can build
# hardware-specific libraries by explicitly naming them (e.g. "make
# mbedtls_msp432e4.a").
#
# Note that in most cases, building hardware-specific libraries
# requires dependent library support (e.g. the MSP432E4 library
# requires crypto drivers provided by the MSP432E4 SimpleLink SDK).
# See documentation in ../../../defs.mak for additional variables that
# must be set for each supported hardware-specific library.
all: mbedtls.a

$(DEFAULT_OBJ_DIR) $(MSP432E4_OBJ_DIR):
	mkdir $@

clean:
	@ $(RMDIR) $(DEFAULT_OBJ_DIR) $(MSP432E4_OBJ_DIR)
	@ $(RM) *.a

mbedtls.a: $(OBJS_DEFAULTDIR)
	@ echo "#"
	@ echo "# Building $@"
	@ $(RM) $@
	$(AR) $@ $^

mbedtls_msp432e4.a: $(OBJS_MSP432E4DIR)
	@ echo "#"
	@ echo "# Building $@"
	@ $(RM) $@
	$(AR) $@ $^
