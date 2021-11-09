# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(CCS_ARMCOMPILER)/bin/armcl"
AR = "$(CCS_ARMCOMPILER)/bin/armar"

CFLAGS = $(INCS) "-I$(CCS_ARMCOMPILER)/include" --silicon_version=7R5 \
        --little_endian --abi=eabi --display_error_number \
        --diag_wrap=off --gen_func_subsections=on \
        --float_support=vfpv3d16 -O2

ARFLAGS = -r

DEFAULT_OBJ_DIR = default
OBJS_DEFAULT = $(addprefix $(DEFAULT_OBJ_DIR)/,$(OBJS))

vpath %.c ../../../../library ../../../port

$(DEFAULT_OBJ_DIR)/%.obj: %.c | $(DEFAULT_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_CCS) $(CFLAGS) $< -fe $@

all: mbedtls.a

$(DEFAULT_OBJ_DIR):
	mkdir $@

clean:
	@ $(RMDIR) $(DEFAULT_OBJ_DIR)
	@ $(RM) *.a

mbedtls.a: $(OBJS_DEFAULT)
	@ echo "#"
	@ echo "# Building $@"
	$(AR) $(ARFLAGS) $@ $^
