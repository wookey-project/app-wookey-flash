/*
 *
 * Copyright 2019 The wookey project team <wookey@ssi.gouv.fr>
 *   - Ryad     Benadjila
 *   - Arnauld  Michelizza
 *   - Mathieu  Renard
 *   - Philippe Thierry
 *   - Philippe Trebuchet
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * the Free Software Foundation; either version 3 of the License, or (at
 * ur option) any later version.
 *
 * This package is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this package; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#ifndef AUTOMATON_H_
#define AUTOMATON_H_

#include "libc/types.h"
#include "wookey_ipc.h"
#include "main.h"

typedef enum {
    DFUFLASH_STATE_INIT = 0,
    DFUFLASH_STATE_IDLE,
    DFUFLASH_STATE_DWNLOAD,
    DFUFLASH_STATE_UPLOAD,
    DFUFLASH_STATE_EOF,
    DFUFLASH_STATE_ERROR
} t_dfuflash_state;

t_dfuflash_state get_task_state(void);

t_dfuflash_state get_next_state(t_dfuflash_state state, uint8_t magic);

void set_task_state(t_dfuflash_state state);

secbool is_valid_transition(t_dfuflash_state state, uint8_t magic);

#if FLASH_DEBUG
const char *get_state_name(t_dfuflash_state state);
#endif

#endif/*!AUTOMATON_H_*/
