#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "mk20dx128.h"
#include "util.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "sdio.h"
#include "fusion.h"
#include "log.h"

typedef enum {
    NONE = 0,
    IDENTIFIER,
    INTEGER,
    END_OF_STATEMENT,
} TokenType;

typedef enum {
    PRIMING = 0,
    PRIMED,
    FIRED,
    ERROR,
} ConsoleState;

typedef enum {
    BREAK,
    RESET,
    INITIALIZE,
    BLOCKS,
    TOGGLE,
    LIST,
    LOG,
    REPLAY,

    IDENTIFIERS_N,
} Identifier;

static char *identifiers[IDENTIFIERS_N] = {
    [BREAK] = "break",
    [RESET] = "reset",
    [INITIALIZE] = "initialize",
    [BLOCKS] = "blocks",
    [TOGGLE] = "toggle",
    [LIST] = "list",
    [LOG] = "log",
    [REPLAY] = "replay",
};

static volatile uint32_t tokens[4];
static volatile uint8_t tokens_n;
static volatile ConsoleState state = PRIMING;

static void dump_blocks(int from, int to)
{
    int i, j, k;

    for (k = from ; k <= to ; k += 1) {
        uint8_t buffer[512];

        sdio_read_single_block(k, buffer);
        sleep_while(sdio_is_busy());

        for (i = 0 ; i < 16 ; i += 1) {
            for (j = 0 ; j < 32 ; j += 1) {
                usbserial_printf("%2x ", buffer[i * 32 + j]);
            }

            usbserial_printf("\n");
        }

        usbserial_printf("\n");
    }
}

static void usb_data_in (uint8_t *data, int length)
{
    static TokenType type;
    static uint32_t token;
    int i;

    /* Disregard any further input once a command has been read and
     * until it is executed. */

    if (state == PRIMED) {
        return;
    } else if (state == FIRED) {
        memset((void *)tokens, 0, sizeof(tokens));
        tokens_n = 0; state = PRIMING;
    }

    /* printf (">> type = %d, j = %d\n", type, j); */

    for(i = 0 ; i < length ; i += 1) {
        /* Parse the token. */

        if (type == NONE) {
            /* Determine the token type. */

            if (isblank((int)data[i])) {
                continue;
            }

            if (isalpha((int)data[i]) || data[i] == '_') {
                type = IDENTIFIER;
            } else if (isdigit((int)data[i])) {
                type = INTEGER;
            } else if (data[i] == '\r' || data[i] == '\n' || data[i] == ',') {
                type = END_OF_STATEMENT;
            } else {
                usbserial_printf("Can't tokenize '%c'.\n", data[i]);
                break;
            }
        }

        switch (type) {
        case END_OF_STATEMENT:
            type = NONE; state = (state == ERROR) ? FIRED : PRIMED;

            break;

        case IDENTIFIER: {
            static uint8_t j;

            if (!isalnum((int)data[i]) && data[i] != '_') {
                /* Match bits are stored inverted (set bit signifies
                 * mismatch (for a reason). */

                token = ~token & ((1 << IDENTIFIERS_N) - 1);

                /* Consume the token. */

                if (token == 0) {
                    usbserial_printf("Unknown identifier.\n");
                    tokens[tokens_n] = 0; state = ERROR;
                } else if ((token & (token - 1)) != 0) {
                    int k;

                    /* More than one bit set.  Command is
                     * ambiguous. */

                    usbserial_printf("Command ambiguous.  "
                                     "Possible candidates:\n");

                    for (k = 0 ; k < IDENTIFIERS_N ; k += 1) {
                        if (token & (1 << k)) {
                            usbserial_printf("%s ", identifiers[k]);
                        }
                    }

                    usbserial_printf("\n");

                    tokens[tokens_n] = 0; state = ERROR;
                } else {
                    tokens[tokens_n] = ffs(token) - 1;
                }

                /* Reset. */

                tokens_n += 1; type = NONE; token = j = 0; i -= 1;
            } else {
                int k;

                for (k = 0 ; k < IDENTIFIERS_N ; k += 1) {
                    if (token & (1 << k)) {
                        continue;
                    }

                    if (j > strlen(identifiers[k]) ||
                        data[i] != identifiers[k][j]) {
                        token |= (1 << k);
                    }
                }

                j += 1;
            }

            break;
        }
        case INTEGER:
            if (!isdigit((int)data[i])) {
                /* Consume the token and reset. */

                tokens[tokens_n] = token;
                tokens_n += 1; type = NONE; token = 0; i -= 1;
            } else {
                token = 10 * token + (data[i] - '0');
            }

            break;

        default:
            assert(0);
        }
    }
}

void console_initialize()
{
    usb_set_data_in_callback(usb_data_in);
    usbserial_set_state(SERIAL_STATE_DSR);
}

void console_enter()
{
    while (1) {
        sleep_while(state != PRIMED || tokens_n == 0);

        switch(tokens[0]) {
        case BREAK: request_reboot(); break;
        case RESET: request_reset(); break;
        case INITIALIZE: log_initialize(); break;

        case BLOCKS:
            switch(tokens_n) {
            case 1: break;
            case 2: dump_blocks(tokens[1], tokens[1]); break;
            case 3: dump_blocks(tokens[1], tokens[2]); break;
            }

            break;

        case TOGGLE: log_toggle(); break;
        case LIST: log_list(); break;
        case LOG: /* usbserial_printf("log %d\n", tokens[1]); */ break;

        case REPLAY:
            log_replay(tokens[1], tokens[2]);
            break;

        default: assert(0);
        }

        state = FIRED;
    }
}
