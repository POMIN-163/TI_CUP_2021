#
#  ======== makefile ========
#

CORESDK_INSTALL_DIR ?= $(abspath ../../..)
include $(CORESDK_INSTALL_DIR)/imports.mak

# Set BUILDMINSTACK to 1 to only build base stack libraries (stk.a* and stk6.a*)
BUILDMINSTACK ?= 1

# Specify additional compiler options (can be set via cmd-line)
# ENABLE_JUMBO : If enabled, packet sizes larger than 1500 bytes are supported
# ENABLE_STRONG_CHECKING : If enabled, error checking is performed on all handles
ENABLE_JUMBO ?= 0
ENABLE_STRONG_CHECKING ?= 0

#
# Set XDCARGS to some of the variables above.  XDCARGS are passed
# to the XDC build engine... which will load ndk.bld... which will
# extract these variables and use them to determine what to build and which
# toolchains to use.
#
# Note that not all of these variables need to be set to something valid.
# If you leave one or more of them unassigned, the libraries for that target
# will not be built.
#
# Some background is here:
#     http://rtsc.eclipse.org/docs-tip/Command_-_xdc#Environment_Variables
#
XDCARGS= \
    ti.targets.arm.elf.M4F=\"$(CCS_ARMCOMPILER)\" \
    gnu.targets.arm.M4F=\"$(GCC_ARMCOMPILER)\" \
    iar.targets.arm.M4F=\"$(IAR_ARMCOMPILER)\" \
    CORESDK_INSTALL_DIR=\"$(CORESDK_INSTALL_DIR)\" \
    MSP432E4DRIVERLIB=$(CORESDK_INSTALL_DIR) \
    BUILDMINSTACK=\"$(BUILDMINSTACK)\" \
    TIPOSIX_REPO=\"$(CORESDK_INSTALL_DIR)/source\" \
    TIDRIVERS_INSTALL_DIR=\"$(CORESDK_INSTALL_DIR)/source\" \
    NS_INSTALL_DIR=\"$(CORESDK_INSTALL_DIR)\" \
    MBEDTLS_INSTALL_DIR=\"$(CORESDK_INSTALL_DIR)/source/third_party/mbedtls\" \
    ENABLE_JUMBO=\"$(ENABLE_JUMBO)\" \
    ENABLE_STRONG_CHECKING=\"$(ENABLE_STRONG_CHECKING)\"

#
# Set XDCPATH to contain necessary repositories.
#
XDCPATH=$(CORESDK_INSTALL_DIR)/source;$(CORESDK_INSTALL_DIR)/kernel/tirtos/packages;$(CORESDK_INSTALL_DIR)/source/third_party/CMSIS/Include

#
# Set XDCOPTIONS.  Use -v for a verbose build.
#
# XDCOPTIONS=v
XDCOPTIONS=
export XDCOPTIONS

j = 1
#
# Set XDC executable command
# Note that XDCBUILDCFG points to the ndk.bld file which uses
# the arguments specified by XDCARGS
#
XDC = $(XDC_INSTALL_DIR)/xdc -j $(j) XDCBUILDCFG=$(CORESDK_INSTALL_DIR)/source/ti/ndk/ndk.bld

all:
	@ echo building packages ...
	@ $(XDC) XDCARGS="$(XDCARGS)" XDCPATH="$(XDCPATH)" all -PR .

debug:
	@ echo building debug packages ...
	@ $(XDC) XDCARGS="profile=debug $(XDCARGS)" XDCPATH="$(XDCPATH)" all -PR .

clean:
	@ echo cleaning packages ...
	@ $(XDC) XDCARGS="$(XDCARGS)" XDCPATH="$(XDCPATH)" clean -PR .

help:
	@ echo "Make Goals"
	@ echo "    all     # build all libraries in release profile"
	@ echo ""
	@ echo "    debug   # build all libraries in debug profile"
	@ echo ""
	@ echo "    clean   # remove all generated content"
	@
