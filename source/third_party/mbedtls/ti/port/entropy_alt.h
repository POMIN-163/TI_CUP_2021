/*
 *  Copyright (C) 2017, Texas Instruments Incorporated, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
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
 */
#ifndef ENTROPY_ALT_H
#define ENTROPY_ALT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  @brief Add a source to gather entropy
 *
 *  Use this function to add an entropy source to mbedtls using
 *  'mbedtls_entropy_add_source()' in the application. Note, this is a sample
 *  definition of an entropy source which is not strong enough. It is
 *  recommended that the application provide a strong implementation of this
 *  API.
 */
int entropy_source(void *data, unsigned char *output, size_t len, size_t *olen);

#ifdef __cplusplus
}
#endif
#endif
