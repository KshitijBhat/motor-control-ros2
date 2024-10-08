//  SPDX-FileCopyrightText: 2022 Jamon Terrell <github@jamonterrell.com>
//  SPDX-License-Identifier: MIT
// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ---------- //
// quadrature //
// ---------- //

#define quadrature_wrap_target 0
#define quadrature_wrap 12

static const uint16_t quadrature_program_instructions[] = {
            //     .wrap_target
    0x2020, //  0: wait   0 pin, 0                   
    0x00c5, //  1: jmp    pin, 5                     
    0xa029, //  2: mov    x, !x                      
    0x0044, //  3: jmp    x--, 4                     
    0xa029, //  4: mov    x, !x                      
    0x0046, //  5: jmp    x--, 6                     
    0x20a0, //  6: wait   1 pin, 0                   
    0x00c9, //  7: jmp    pin, 9                     
    0x0049, //  8: jmp    x--, 9                     
    0xa029, //  9: mov    x, !x                      
    0x004b, // 10: jmp    x--, 11                    
    0xa029, // 11: mov    x, !x                      
    0x0000, // 12: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program quadrature_program = {
    .instructions = quadrature_program_instructions,
    .length = 13,
    .origin = -1,
};

static inline pio_sm_config quadrature_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + quadrature_wrap_target, offset + quadrature_wrap);
    return c;
}

static inline void quadrature_program_init(PIO pio, uint sm, uint offset, uint a_pin, uint b_pin) {
    pio_sm_config c = quadrature_program_get_default_config(offset);
    sm_config_set_in_pins(&c, b_pin);
    sm_config_set_jmp_pin(&c, a_pin);
    sm_config_set_in_shift(&c, false, true, 32);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif