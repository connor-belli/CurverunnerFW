#include "serialcomm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <ctype.h>
#include <stdbool.h>

#define SERIALCOMM_COMMAND_QUEUE_SIZE 5



void serialcomm_init(struct SerialCommProcessor *ctx)
{
    serialcomm_reset_state(ctx);
    ctx->command_queue = xQueueCreate(SERIALCOMM_COMMAND_QUEUE_SIZE, sizeof(struct SerialCommand));
}

void serialcomm_reset_state(struct SerialCommProcessor *ctx)
{
    struct SerialCommParseState *s = &ctx->state;
    s->state_code = SERIALCOMM_STATE_PARSE_COMMAND;
    s->command = 0;
    s->param1 = 0;
    s->param2 = 0;
}

BaseType_t serialcomm_add_char(struct SerialCommProcessor *ctx, char c)
{
    struct SerialCommParseState *s = &ctx->state;
    struct SerialCommand command;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (s->state_code == SERIALCOMM_STATE_PARSE_COMMAND)
    {
        if (c != '\r' && c != '\n' && s->command == 0)
        {
            // Read command character if command has not already been read
            s->command = c;
        }
        else if (c == ',')
        {
            // After command has been read, we are expecting a comma to start parsing param1
            s->state_code = SERIALCOMM_STATE_PARSE_PARAM1;
        }
        else
        {
            // Invalid character found, state will become error
            s->state_code = SERIALCOMM_STATE_ERROR;
        }
    }
    else if (s->state_code == SERIALCOMM_STATE_PARSE_PARAM1)
    {
        if (isdigit(c))
        {
            // Read digits into param1
            s->param1 = s->param1 * 10 + (c - '0');
        }
        else if (c == ',')
        {
            // End of param1
            s->state_code = SERIALCOMM_STATE_PARSE_PARAM2;
        }
        else
        {
            // Invalid character in param
            s->state_code = SERIALCOMM_STATE_ERROR;
        }
    }
    else if (s->state_code == SERIALCOMM_STATE_PARSE_PARAM2)
    {
        if (isdigit(c))
        {
            // Read digits into param1
            s->param2 = s->param2 * 10 + (c - '0');
        }
        else if (c == '\n' || c == '\r')
        {
            // End of param2 and command, reset state
            command.status = true;
            command.command = s->command,
            command.param1 = s->param1,
            command.param2 = s->param2,

            xQueueSendToBackFromISR(ctx->command_queue, &command, &higherPriorityTaskWoken);
            serialcomm_reset_state(ctx);
        }
        else
        {
            // Invalid character in param
            s->state_code = SERIALCOMM_STATE_ERROR;
        }
    }

    if (s->state_code == SERIALCOMM_STATE_ERROR)
    {
        // Currently in error code, wait for newline to reset
        if (c == '\n' || c == '\r')
        {
            command.status = false;
            command.command = s->command,
            command.param1 = s->param1,
            command.param2 = s->param2,
            serialcomm_reset_state(ctx);

            xQueueSendToBackFromISR(ctx->command_queue, &command, &higherPriorityTaskWoken);
        }
    }

    return higherPriorityTaskWoken;
}

BaseType_t serialcomm_add_chars(struct SerialCommProcessor *ctx, uint8_t *buf, uint32_t len)
{
    int i = 0;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    for (i = 0; i < len; i++) {
        higherPriorityTaskWoken |= serialcomm_add_char(ctx, buf[i]);
    }

    return higherPriorityTaskWoken;
}