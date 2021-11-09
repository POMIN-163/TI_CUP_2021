# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(GCC_ARMCOMPILER)/bin/aarch64-none-elf-gcc"
AR = "$(GCC_ARMCOMPILER)/bin/aarch64-none-elf-gcc-ar"

CFLAGS= $(INCS) -O2 -mcpu=cortex-a53+fp+simd -mabi=lp64 \
        -mcmodel=large -mstrict-align -mfix-cortex-a53-835769 \
        -mfix-cortex-a53-843419 \
        -ffunction-sections \
        -fdata-sections -g -gstrict-dwarf -Wall -std=c99 -c

ARFLAGS = -r

DEFAULT_OBJ_DIR = default
OBJS_DEFAULT = $(addprefix $(DEFAULT_OBJ_DIR)/,$(OBJS))

vpath %.c ../../../../library ../../../port

$(DEFAULT_OBJ_DIR)/%.obj: %.c | $(DEFAULT_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_GCC) $(CFLAGS) $< -o $@

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
