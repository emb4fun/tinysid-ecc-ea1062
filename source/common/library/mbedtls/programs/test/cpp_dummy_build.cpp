/*
 *  This program is a dummy C++ program to ensure Mbed TLS library header files
 *  can be included and built with a C++ compiler.
 *
 *  Copyright (C) 2018, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 *
 *  This file is provided under the Apache License 2.0, or the
 *  GNU General Public License v2.0 or later.
 *
 *  **********
 *  Apache License 2.0:
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  **********
 *
 *  **********
 *  GNU General Public License v2.0 or later:
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  **********
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "mbedtls/aes.h"
#include "mbedtls/aesni.h"
#include "mbedtls/arc4.h"
#include "mbedtls/aria.h"
#include "mbedtls/asn1.h"
#include "mbedtls/asn1write.h"
#include "mbedtls/base64.h"
#include "mbedtls/bignum.h"
#include "mbedtls/blowfish.h"
#include "mbedtls/bn_mul.h"
#include "mbedtls/camellia.h"
#include "mbedtls/ccm.h"
#include "mbedtls/certs.h"
#include "mbedtls/chacha20.h"
#include "mbedtls/chachapoly.h"
#include "mbedtls/check_config.h"
#include "mbedtls/cipher.h"
#include "mbedtls/cipher_internal.h"
#include "mbedtls/cmac.h"
#include "mbedtls/compat-1.3.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#include "mbedtls/des.h"
#include "mbedtls/dhm.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecjpake.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecp_internal.h"
#include "mbedtls/entropy.h"
#include "mbedtls/entropy_poll.h"
#include "mbedtls/error.h"
#include "mbedtls/gcm.h"
#include "mbedtls/havege.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/hmac_drbg.h"
#include "mbedtls/md.h"
#include "mbedtls/md2.h"
#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include "mbedtls/md_internal.h"
#include "mbedtls/net.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/nist_kw.h"
#include "mbedtls/oid.h"
#include "mbedtls/padlock.h"
#include "mbedtls/pem.h"
#include "mbedtls/pk.h"
#include "mbedtls/pk_internal.h"
#include "mbedtls/pkcs11.h"
#include "mbedtls/pkcs12.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/platform_time.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/poly1305.h"
#include "mbedtls/ripemd160.h"
#include "mbedtls/rsa.h"
#include "mbedtls/rsa_internal.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_cache.h"
#include "mbedtls/ssl_ciphersuites.h"
#include "mbedtls/ssl_cookie.h"
#include "mbedtls/ssl_internal.h"
#include "mbedtls/ssl_ticket.h"
#include "mbedtls/threading.h"
#include "mbedtls/timing.h"
#include "mbedtls/version.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/x509_csr.h"
#include "mbedtls/xtea.h"

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

int main()
{
    mbedtls_platform_context *ctx = NULL;
    mbedtls_platform_setup(ctx);
    mbedtls_printf("CPP Build test\n");
    mbedtls_platform_teardown(ctx);
}
