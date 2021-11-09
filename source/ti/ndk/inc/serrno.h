/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== serrno.h ========
 * NDK error return constants for socket operations.
 */

#ifndef _SERRNO_H_
#define _SERRNO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * These values are straight from UNIX. Those that are commented
 * out are not used in this stack implementation.
 *
 * Regarding BSD Support:
 *
 * If user's app is including compiler's errno.h (i.e. getting error
 * definitions from compiler's errno.h), then use error macros defined there.
 * Otherwise, define them here and use fdError to get error codes.
 */

/*#define NDK_EPERM           1       // Operation not permitted */
/*#define NDK_ENOENT          2       // No such file or directory */
/*#define NDK_ESRCH           3       // No such process */

#ifndef NDK_EINTR
#define NDK_EINTR           4       // Interrupted system call
#endif

/*#define NDK_EIO             5       // Input/output error */

#ifndef NDK_ENXIO
#define NDK_ENXIO           6       /* Device not configured */
#endif

/*#define NDK_E2BIG           7       // Argument list too long */
/*#define NDK_ENOEXEC         8       // Exec format error */

#ifndef NDK_EBADF
#define NDK_EBADF           9       /* Bad file descriptor */
#endif

/*#define NDK_ECHILD          10      // No child processes */
/*#define NDK_EDEADLK         11      // Resource deadlock avoided */

#ifndef NDK_ENOMEM
#define NDK_ENOMEM          12      /* Cannot allocate memory */
#endif

#ifndef NDK_EACCES
#define NDK_EACCES          13      /* Permission denied */
#endif

/*#define NDK_EFAULT          14      // Bad address */
/*#define NDK_ENOTBLK         15      // Block device required */
/*#define NDK_EBUSY           16      // Device busy */
/*#define NDK_EEXIST          17      // File exists */
/*#define NDK_EXDEV           18      // Cross-device link */
/*#define NDK_ENODEV          19      // Operation not supported by device */
/*#define NDK_ENOTDIR         20      // Not a directory */
/*#define NDK_EISDIR          21      // Is a directory */

#ifndef NDK_EINVAL
#define NDK_EINVAL          22      /* Invalid argument */
#endif

#ifndef NDK_ENFILE
#define NDK_ENFILE          23      /* Too many open files in system */
#endif

#ifndef NDK_EMFILE
#define NDK_EMFILE          24      /* Too many open files */
#endif

/*#define NDK_ENOTTY          25      // Inappropriate ioctl for device */
/*#define NDK_ETXTBSY         26      // Text file busy */
/*#define NDK_EFBIG           27      // File too large */
/*#define NDK_ENOSPC          28      // No space left on device */
/*#define NDK_ESPIPE          29      // Illegal seek */
/*#define NDK_EROFS           30      // Read-only file system */
/*#define NDK_EMLINK          31      // Too many links */

#ifndef NDK_EPIPE
#define NDK_EPIPE           32      // Broken pipe
#endif

#ifndef NDK_EDOM
#define NDK_EDOM            33      /* Mathematics argument out of domain */
#endif

/* non-blocking and interrupt i/o */

#ifndef NDK_EWOULDBLOCK
#define NDK_EWOULDBLOCK     35      /* Operation would block */
#endif

#ifndef NDK_EAGAIN
#define NDK_EAGAIN          NDK_EWOULDBLOCK      /* Try again */
#endif

#ifndef NDK_EINPROGRESS
#define NDK_EINPROGRESS     36      /* Operation now in progress */
#endif

#ifndef NDK_EALREADY
#define NDK_EALREADY        37      /* Operation already in progress */
#endif

/* ipc/network software -- argument errors */
#ifndef NDK_ENOTSOCK
#define NDK_ENOTSOCK        38      /* Socket operation on non-socket */
#endif

/*#define NDK_EDESTADDRREQ    39      // Destination address required */

#ifndef NDK_EMSGSIZE
#define NDK_EMSGSIZE        40      /* Message too long */
#endif

#ifndef NDK_EPROTOTYPE
#define NDK_EPROTOTYPE      41      /* Protocol wrong type for socket */
#endif

#ifndef NDK_ENOPROTOOPT
#define NDK_ENOPROTOOPT     42      /* Protocol not available */
#endif

/*#define NDK_EPROTONOSUPPORT 43      // Protocol not supported */

#ifndef NDK_ESOCKTNOSUPPORT
#define NDK_ESOCKTNOSUPPORT 44      /* Socket type not supported */
#endif

#ifndef NDK_EOPNOTSUPP
#define NDK_EOPNOTSUPP      45      /* Operation not supported */
#endif

#ifndef NDK_EPFNOSUPPORT
#define NDK_EPFNOSUPPORT    46      /* Protocol family not supported */
#endif

/*#define NDK_EAFNOSUPPORT    47      // Address family not supported by protocol family */

#ifndef NDK_EADDRINUSE
#define NDK_EADDRINUSE      48      /* Address already in use */
#endif

#ifndef NDK_EADDRNOTAVAIL
#define NDK_EADDRNOTAVAIL   49      /* Can't assign requested address */
#endif

/* ipc/network software -- operational errors */
#ifndef NDK_ENETDOWN
#define NDK_ENETDOWN        50     /* Socket operation failed because the */
#endif                         /* network was down. */

/*#define NDK_ENETUNREACH     51      // Network is unreachable */
/*#define NDK_ENETRESET       52      // Network dropped connection on reset */

#ifndef NDK_ECONNABORTED
#define NDK_ECONNABORTED    53      /* Software caused connection abort */
#endif

#ifndef NDK_ECONNRESET
#define NDK_ECONNRESET      54      /* Connection reset by peer */
#endif

#ifndef NDK_ENOBUFS
#define NDK_ENOBUFS         55      /* No buffer space available */
#endif

#ifndef NDK_EISCONN
#define NDK_EISCONN         56      /* Socket is already connected */
#endif

#ifndef NDK_ENOTCONN
#define NDK_ENOTCONN        57      /* Socket is not connected */
#endif

#ifndef NDK_ESHUTDOWN
#define NDK_ESHUTDOWN       58      /* Can't send after socket shutdown */
#endif

/*#define NDK_ETOOMANYREFS    59      // Too many references: can't splice */

#ifndef NDK_ETIMEDOUT
#define NDK_ETIMEDOUT       60      /* Operation timed out */
#endif

#ifndef NDK_ECONNREFUSED
#define NDK_ECONNREFUSED    61      /* Connection refused */
#endif

/*#define NDK_ELOOP           62      // Too many levels of symbolic links */
/*#define NDK_ENAMETOOLONG    63      // File name too long */

#ifndef NDK_EHOSTDOWN
#define NDK_EHOSTDOWN       64      /* Host is down */
#endif

#ifndef NDK_EHOSTUNREACH
#define NDK_EHOSTUNREACH    65      /* No route to host */
#endif

/* Error values for `getaddrinfo' function. */
#ifndef NDK_EAI_BADFLAGS
#define NDK_EAI_BADFLAGS    -1    /* Invalid value for `ai_flags' field. */
#endif

#ifndef NDK_EAI_NONAME
#define NDK_EAI_NONAME      -2    /* NAME or SERVICE is unknown. */
#endif

#ifndef NDK_EAI_AGAIN
#define NDK_EAI_AGAIN       -3    /* Temporary failure in name resolution. */
#endif

#ifndef NDK_EAI_FAIL
#define NDK_EAI_FAIL        -4    /* Non-recoverable failure in name res. */
#endif

#ifndef NDK_EAI_NODATA
#define NDK_EAI_NODATA      -5    /* No address associated with NAME. */
#endif

#ifndef NDK_EAI_FAMILY
#define NDK_EAI_FAMILY      -6    /* `ai_family' not supported. */
#endif

#ifndef NDK_EAI_SOCKTYPE
#define NDK_EAI_SOCKTYPE    -7    /* `ai_socktype' not supported. */
#endif

#ifndef NDK_EAI_SERVICE
#define NDK_EAI_SERVICE     -8    /* SERVICE not supported for `ai_socktype'. */
#endif

#ifndef NDK_EAI_ADDRFAMILY
#define NDK_EAI_ADDRFAMILY  -9    /* Address family for NAME not supported. */
#endif

#ifndef NDK_EAI_MEMORY
#define NDK_EAI_MEMORY      -10   /* Memory allocation failure. */
#endif

#ifndef NDK_EAI_SYSTEM
#define NDK_EAI_SYSTEM      -11   /* System error returned in `errno'. */
#endif

#ifndef NDK_EAI_OVERFLOW
#define NDK_EAI_OVERFLOW    -12   /* Argument buffer overflow. */
#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

