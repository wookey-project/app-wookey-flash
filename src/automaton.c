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

#include "automaton.h"
#include "main.h"
#include "wookey_ipc.h"
#include "libc/stdio.h"
#include "libc/nostd.h"

static volatile t_dfuflash_state task_state = DFUFLASH_STATE_INIT;

#if FLASH_DEBUG
static const char *state_tab[] = {
    "DFUFLASH_STATE_INIT",
    "DFUFLASH_STATE_IDLE",
    "DFUFLASH_STATE_DWNLOAD",
    "DFUFLASH_STATE_UPLOAD",
    "DFUFLASH_STATE_EOF",
    "DFUFLASH_STATE_ERROR",
};

const char *get_state_name(t_dfuflash_state state)
{
    return state_tab[state];
}
#endif

typedef struct dfuflash_request_transition {
    uint8_t    request;
    uint8_t    target_state;
} dfuflash_request_transition_t;


static const struct {
    t_dfuflash_state               state;
    dfuflash_request_transition_t  req_trans[5];
} flash_automaton[] = {
    /* initialization phase. init specific IPC should be added here... no filter by now. */
    { DFUFLASH_STATE_INIT,  {
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },
    /* Init phase is now finished. Waiting for first request. */
    { DFUFLASH_STATE_IDLE,  {
                                 {MAGIC_DATA_WR_DMA_REQ,DFUFLASH_STATE_DWNLOAD},
                                 {MAGIC_DATA_RD_DMA_REQ,DFUFLASH_STATE_UPLOAD},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },
    /* Download phase, A download request has been received */
    { DFUFLASH_STATE_DWNLOAD, {
                                 {MAGIC_DATA_WR_DMA_REQ,DFUFLASH_STATE_DWNLOAD},
                                 {MAGIC_DFU_DWNLOAD_FINISHED,DFUFLASH_STATE_EOF},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },
    /* Upload phase, An upload request has been received */
    { DFUFLASH_STATE_UPLOAD,   {
                                 {MAGIC_DATA_RD_DMA_REQ,DFUFLASH_STATE_UPLOAD},
                                 {MAGIC_DFU_DWNLOAD_FINISHED,DFUFLASH_STATE_EOF},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },
    /* EOF phase, the firmware is downloaded, flash unmap. Waiting for reboot */
    { DFUFLASH_STATE_EOF,    {
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },
    /* Any error make smart going to this state. In production mode, this means rebooting the device */
    { DFUFLASH_STATE_ERROR, {
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff},
                                 {0xff,0xff}
                             }
    },

};


t_dfuflash_state get_task_state(void)
{
    return task_state;
}

t_dfuflash_state get_next_state(t_dfuflash_state state, uint8_t magic)
{
    for (uint8_t i = 0; i < 5; ++i) {
        if (flash_automaton[state].req_trans[i].request == magic) {
            return (flash_automaton[state].req_trans[i].target_state);
        }
    }
    /* fallback, no corresponding request found for  this state */
    return 0xff;
}


void set_task_state(t_dfuflash_state state)
{
    /* not moving... */
    if (state == task_state) {
        return;
    }
#if FLASH_DEBUG
    printf("state: %s => %s\n", state_tab[task_state], state_tab[state]);
#endif
    task_state = state;
}

secbool is_valid_transition(t_dfuflash_state state, uint8_t magic)
{
    /* Try to make the automaton transition a bit more robust
     * against fault attacks.
     */
    for (uint8_t i = 0; i < 5; ++i) {
        if (flash_automaton[state].req_trans[i].request == magic) {
            return sectrue;
        }
    }
    return secfalse;
}
