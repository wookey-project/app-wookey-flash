#ifndef AUTOMATON_H_
#define AUTOMATON_H_

#include "api/types.h"
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
