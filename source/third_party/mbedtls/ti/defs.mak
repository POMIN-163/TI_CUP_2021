#
# By default, look for an SDK-provided imports.mak file to provide
# dependency locations (e.g. toolchains).
#
# Note, this SDK's imports.mak inclusion is optional (the leading '-'
# when including).  If exists, we'll use variables in it, else the
# user will have to set these variables themselves.  The variables
# this build system may use, depending on what you're trying to build, include:
#    CCS_ARMCOMPILER     - CCS ARM toolchain
#    IAR_ARMCOMPILER     - IAR ARM toolchain
#    GCC_ARMCOMPILER     - GCC ARM toolchain
#    GCC_ARM64COMPILER   - GCC 64-bit ARM toolchain
#    CCS_C6XCOMPILER     - CCS C6x toolchain
#    TICLANG_ARMCOMPILER - TI ARM CLANG toolchain
#    RM                  - delete a file in an OS-independent way
#    RMDIR               - delete a directory in an OS-independent way
#
# Note that this SDK_INSTALL_DIR path is relative to the
# makefiles including this defs.mak, which are 3 directories deeper
# (lib/<toolchain>/<isa>) than this file.
SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

-include $(SDK_INSTALL_DIR)/imports.mak

# Default POSIX is in the SDK's source/ directory, but users can
# override this
POSIX_ROOT = $(SDK_INSTALL_DIR)/source

# To build MSP432E4-specific hardware-accelerated libraries, uncomment
# this variable assignment and ensure it is set to your MSP432E4
# SimpleLink SDK
#MSP432E4_SDK_INSTALL_DIR = $(SDK_INSTALL_DIR)

OBJS_CRYPTO=    aes.obj       aesni.obj     arc4.obj      \
        asn1parse.obj asn1write.obj base64.obj    \
        bignum.obj    blowfish.obj  camellia.obj  \
        ccm.obj       chacha20.obj  chachapoly.obj \
        cipher.obj    cipher_wrap.obj   \
        cmac.obj      ctr_drbg.obj  des.obj       \
        dhm.obj       ecdh.obj      ecdsa.obj     \
        ecjpake.obj   ecp.obj               \
        ecp_curves.obj    entropy.obj   entropy_poll.obj  \
        error.obj     gcm.obj       havege.obj    \
        hmac_drbg.obj md.obj        md2.obj       \
        md4.obj       md5.obj       md_wrap.obj   \
        memory_buffer_alloc.obj       oid.obj       \
        padlock.obj   pem.obj       poly1305.obj     pk.obj        \
        pk_wrap.obj   pkcs12.obj    pkcs5.obj     \
        pkparse.obj   pkwrite.obj   platform.obj     platform_util.obj \
        ripemd160.obj rsa.obj       rsa_internal.obj sha1.obj \
        sha256.obj    sha512.obj    threading.obj \
        timing.obj    version.obj           \
        version_features.obj      xtea.obj

OBJS_X509=  certs.obj     pkcs11.obj    x509.obj      \
        x509_create.obj   x509_crl.obj  x509_crt.obj  \
        x509_csr.obj  x509write_crt.obj x509write_csr.obj

OBJS_TLS=   debug.obj     net_sockets.obj       \
        ssl_cache.obj ssl_ciphersuites.obj  \
        ssl_cli.obj   ssl_cookie.obj        \
        ssl_srv.obj   ssl_ticket.obj        \
        ssl_tls.obj

OBJS_PORT= entropy_alt.obj threading_alt.obj
OBJS_PORT_MSP432E4 = $(OBJS_PORT) sha256_alt.obj aes_alt.obj

OBJS= $(OBJS_CRYPTO) \
        $(OBJS_X509) \
        $(OBJS_TLS) \
        $(OBJS_PORT)

OBJS_MSP432E4 = $(OBJS_CRYPTO) \
        $(OBJS_X509) \
        $(OBJS_TLS) \
        $(OBJS_PORT_MSP432E4)

INCS = -I../../../../include -I../../../../ti/port \
        -I../../../../ti/configs \
        -DMBEDTLS_CONFIG_FILE='"config-ti.h"'

INCS_MSP432E4 = -I../../../../include -I../../../../ti/port \
        -I$(MSP432E4_SDK_INSTALL_DIR)/source \
        -I../../../../ti/configs \
        -DMBEDTLS_CONFIG_FILE='"config-msp432e4.h"' \
        -DDeviceFamily_MSP432E401Y

POSIX_INCS_CCS = -I$(POSIX_ROOT)/ti/posix/ccs
POSIX_INCS_IAR = -I$(POSIX_ROOT)/ti/posix/iar
POSIX_INCS_GCC = -I$(POSIX_ROOT)/ti/posix/gcc
POSIX_INCS_TICLANG = -I$(POSIX_ROOT)/ti/posix/ticlang
