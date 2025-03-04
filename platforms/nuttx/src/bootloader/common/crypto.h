/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file crypto.h
 *
 * Wrapper for the crypto stuff
 *
 */

#pragma once

/* Using security always needs TOC (but TOC could be used without security) */
#if defined(BOOTLOADER_USE_SECURITY)
# define BOOTLOADER_USE_TOC

#include <stdlib.h>

#include "hw_config.h"
#include "image_toc.h"

bool verify_toc(const void *toc_start);

bool verify_app(uint16_t idx, const image_toc_entry_t *toc_entries);

bool decrypt_app(uint16_t idx, const image_toc_entry_t *toc_entries);

#else

# if defined(BOOTLOADER_USE_TOC)

/* No security, toc verification passes always */

static inline bool verify_toc(const void *toc_start) {return true;}

/* No security, application verification passes always */

static inline bool verify_app(uint16_t idx, const image_toc_entry_t *toc_entries) {return true;}

/* No security, decrypting is not possible */

static inline bool decrypt_app(uint16_t idx, const image_toc_entry_t *toc_entries) {return false;}

# endif

#endif // BOOTLOADER_USE_SECURITY
