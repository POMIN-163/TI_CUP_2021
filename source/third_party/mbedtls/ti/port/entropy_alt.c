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
#include <stdlib.h>
#include <time.h>
#include <mbedtls/entropy.h>
#include <entropy_alt.h>

int entropy_source(void *data, unsigned char *output, size_t len, size_t *olen)
{
    int i;
    unsigned int seed;

    if (!output || !len || !olen) {
        return (MBEDTLS_ERR_ENTROPY_SOURCE_FAILED);
    }

    if (data != NULL) {
        seed = *(int *)data ^ time(NULL);
    }
    else {
        seed = time(NULL);
    }

    srand(seed);
    for (i = 0; i < len; i++) {
         output[i] = rand() % 256;
    }

    *olen = len;

    return (0);
}
