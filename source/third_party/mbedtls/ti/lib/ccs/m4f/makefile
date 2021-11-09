# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(CCS_ARMCOMPILER)/bin/armcl"
AR = "$(CCS_ARMCOMPILER)/bin/armar"

CCS_M4F_CFLAGS = "-I$(CCS_ARMCOMPILER)/include" --silicon_version=7M4 \
        --code_state=16 --little_endian --display_error_number \
        --diag_warning=255 --diag_wrap=off --gen_func_subsections=on \
        --float_support=FPv4SPD16 -O2

CFLAGS_DEFAULT = $(INCS) $(CCS_M4F_CFLAGS)
CFLAGS_MSP432E4 = $(INCS_MSP432E4) $(CCS_M4F_CFLAGS)

ARFLAGS = -r

DEFAULT_OBJ_DIR = default
OBJS_DEFAULTDIR = $(addprefix $(DEFAULT_OBJ_DIR)/,$(OBJS))

MSP432E4_OBJ_DIR = msp432e4
OBJS_MSP432E4DIR = $(addprefix $(MSP432E4_OBJ_DIR)/,$(OBJS_MSP432E4))

vpath %.c ../../../../library ../../../port

$(DEFAULT_OBJ_DIR)/%.obj: %.c | $(DEFAULT_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_CCS) $(CFLAGS_DEFAULT) $< -fe $@

$(MSP432E4_OBJ_DIR)/%.obj: %.c | $(MSP432E4_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_CCS) $(CFLAGS_MSP432E4) $< -fe $@

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
	$(AR) $(ARFLAGS) $@ $^

mbedtls_msp432e4.a: $(OBJS_MSP432E4DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(AR) $(ARFLAGS) $@ $^
