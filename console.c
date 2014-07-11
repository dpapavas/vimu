#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "mk20dx128.h"
#include "util.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "log.h"

#define TOKEN_BUFFER_SIZE 16

typedef enum {
    IDENTIFIER,
    INTEGER,
    END_OF_STATEMENT,
} TokenType;

typedef enum {
    INITIALIZE,
    LOG,
    END,
    LIST,
    SET,
    GET,
    EXIT,

    DATA,

    IDENTIFIERS_N
} Identifier;

static char *identifiers[IDENTIFIERS_N] = {
    [INITIALIZE] = "initialize",
    [LOG] = "log",
    [END] = "end",
    [LIST] = "list",
    [SET] = "set",
    [GET] = "get",

    [DATA] = "data",

    [EXIT] = "exit",
};

static int parse_token(TokenType type)
{
    static char buffer[TOKEN_BUFFER_SIZE];
    static int i, n;
    int j, skip;

    assert (i > 0 || n == 0);

    if (i > 0 && i < n) {
        memmove(buffer, buffer + i, (n -= i));
        /* usbserial_trace("* %s\n", buffer); */
    } else {
        n = 0;
    }
    /* usbserial_trace("** n=%d\n", n); */
    /* usbserial_trace("%d\n", buffer[0]); */

    i = 0, j = -1, skip = 0;

    while (1) {
        if (i == n) {
            /* Fetch more data from the USB interface.  If we've used
             * up the whole buffer while parsing the token bail
             * out.  */

            if (n == TOKEN_BUFFER_SIZE) {
                if(!skip) {
                    usbserial_printf("Parse error, token too long.\n");
                    skip = 1;
                }

                i = n = 0;
            }

            n += usb_read(buffer + n, TOKEN_BUFFER_SIZE - n);
            assert (n > 0);
        }

        /* If there was a parse error, skip to the end of the statement. */

        if (skip) {
            if (buffer[i] == '\n' || buffer[i] == ';') {
                i += 1;

                return -1;
            } else {
                i += 1;

                continue;
            }
        }

        /* Consume leading white-space. */

        if (j < 0) {
            if (!isblank((int)buffer[i])) {
                j = i;
            } else {
                i += 1;

                continue;
            }
        }

        /* Parse the token. */

        switch (type) {
        case IDENTIFIER:
            if (!isalnum((int)buffer[i]) && buffer[i] != '_') {
                if(i == j) {
                    usbserial_printf("Parse error, identifier expected.\n");
                    skip = 1;
                } else {
                    int k;

                    for (k = 0 ; k < IDENTIFIERS_N ; k += 1) {
                        if (!strncmp(identifiers[k], buffer + j, i - j)) {
                            break;
                        }
                    }

                    if (k == IDENTIFIERS_N) {
                        usbserial_printf("Parse error, unknown identifier.\n");
                        skip = 1;
                    } else {
                        return k;
                    }
                }
            } else {
                i += 1;
            }
            break;

        case INTEGER:
            if (!isdigit((int)buffer[i])) {
                if(i == j) {
                    usbserial_printf("Parse error, integer expected.\n");
                    skip = 1;
                } else {
                    return atoi(buffer + j);
                }
            } else {
                if (!isdigit((int)buffer[i])) {
                }

                i += 1;
            }
            break;

        case END_OF_STATEMENT:
            if (buffer[i] != '\n' && buffer[i] != ';') {
                usbserial_printf("Parse error, end of statement expected.\n");
                skip = 1;
            } else {
                i += 1;

                return 1;
            }
            break;
        }
    }

    return -1;
}

void console_enter()
{
    int i, j, done;

    for(done = 0 ; !done ; ) {
        if ((i = parse_token(IDENTIFIER)) >= 0) {

            turn_on_led();

            switch(i) {
            case INITIALIZE:
                if (parse_token(END_OF_STATEMENT) >= 0) {
                    log_initialize();
                }

                break;

            case LOG:
                if (parse_token(END_OF_STATEMENT) >= 0) {
                    log_begin();
                }

                break;

            case END:
                if (parse_token(END_OF_STATEMENT) >= 0) {
                    log_end();
                }

                break;

            case LIST:
                if (parse_token(END_OF_STATEMENT) >= 0) {
                    log_list();
                }

                break;

            case SET:
                if ((j = parse_token(IDENTIFIER)) >= 0) {
                    int n;

                    switch(j) {
                    case DATA:
                        if ((n = parse_token(INTEGER)) >= 0 &&
                            parse_token(END_OF_STATEMENT) >= 0) {
                            log_set_data(n);
                        }

                        break;
                    default:
                        usbserial_printf("I can't set that.\n");
                    }
                }

                break;

            case GET:
                if ((j = parse_token(IDENTIFIER)) >= 0) {
                    switch(j) {
                    case DATA:
                        if (parse_token(END_OF_STATEMENT) >= 0) {
                            usbserial_printf("%d\n", log_get_data());
                        }

                        break;
                    default:
                        usbserial_printf("I can't get that.\n");
                    }
                }

                break;

            case EXIT:
                if (parse_token(END_OF_STATEMENT) >= 0) {
                    done = 1;
                }

                break;

            default:
                if (parse_token(END_OF_STATEMENT) < 0);
                usbserial_printf("Huh?\n");

                break;
            }

            turn_off_led();
        }
    }
}
