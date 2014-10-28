#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "mk20dx128.h"
#include "util.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "log.h"

typedef enum {
    NONE = 0,
    IDENTIFIER,
    INTEGER,
} TokenType;

typedef enum {
    /* Commands with no argument. */

    INITIALIZE,
    TOGGLE,
    LIST,

    /* Commands with one argument. */

    LOG,
    REPLAY,

    IDENTIFIERS_N,
} Identifier;

static char *identifiers[IDENTIFIERS_N] = {
    [INITIALIZE] = "initialize",
    [TOGGLE] = "toggle",
    [LIST] = "list",

    [LOG] = "log",
    [REPLAY] = "replay",
};

static void execute(uint8_t command, uint32_t argument)
{
    turn_on_led();
    switch(command) {
    case INITIALIZE: log_initialize(); break;
    case TOGGLE: log_toggle(); break;
    case LIST: log_list(); break;
    case LOG: usbserial_printf("log %d\n", argument); break;
    default: assert(0);
    }
    turn_off_led();
}

static void consume(TokenType type, uint32_t token)
{
    static uint8_t command = IDENTIFIERS_N;

    if (command == IDENTIFIERS_N) {
        if(type == IDENTIFIER && token >= INITIALIZE && token <= REPLAY) {
            if (token < LOG) {
                execute(token, 0);
            } else {
                command = token;
            }
        } else {
            usbserial_printf("Not a command.\n");
        }
    } else {
        execute(command, token);
        command = IDENTIFIERS_N;
    }
}

static void parse_input (uint8_t *data, int length)
{
    static TokenType type;
    static uint32_t token;
    int i;

    /* printf (">> type = %d, j = %d\n", type, j); */

    for(i = 0 ; i < length ; i += 1) {
        /* Parse the token. */

        if (type == NONE) {
            /* Determine the token type. */

            if (isspace((int)data[i])) {
                continue;
            }

            if (isalpha((int)data[i]) || data[i] == '_') {
                type = IDENTIFIER;
            } else if (isdigit((int)data[i])) {
                type = INTEGER;
            } else {
                usbserial_printf("Can't tokenize '%c'.\n", data[i]);
                break;
            }
        }

        if (type == IDENTIFIER) {
            static uint8_t j;

            if (!isalnum((int)data[i]) && data[i] != '_') {
                /* Consume the token. */

                token = ffs(~token) - 1;

                if (token >= IDENTIFIERS_N) {
                    usbserial_printf("Unknown identifier.\n");
                } else {
                    consume(IDENTIFIER, token);
                }

                /* Reset. */

                type = NONE; token = j = 0; i -= 1;
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
        } else if (type == INTEGER) {
            if (!isdigit((int)data[i])) {
                /* Consume the token and reset. */

                consume(INTEGER, token);
                type = NONE; token = 0; i -= 1;
            } else {
                token = 10 * token + (data[i] - '0');
            }
        } else {
            assert(0);
        }
    }
}

void console_cycle()
{
    char *buffer;
    int length;

    if (usb_read (&buffer, &length)) {
        parse_input ((uint8_t *)buffer, length);
    }
}
