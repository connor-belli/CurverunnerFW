#ifndef __SERIALCOMM_H
#define __SERIALCOMM_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

struct SerialCommand {
    bool status;
    char command[3];
    int param1;
    int param2;
};

enum SerialCommStateCode {
    SERIALCOMM_STATE_PARSE_COMMAND,
    SERIALCOMM_STATE_PARSE_PARAM1,
    SERIALCOMM_STATE_PARSE_PARAM2,
    SERIALCOMM_STATE_ERROR,
};

struct SerialCommParseState {
    enum SerialCommStateCode state_code;
    char command[3];
    int char_count;
    int param1;
    bool is_negative1;
    int param2;
    bool is_negative2;
};

struct SerialCommProcessor {
    QueueHandle_t command_queue;
    struct SerialCommParseState state;
};

void serialcomm_init(struct SerialCommProcessor *ctx);
void serialcomm_reset_state(struct SerialCommProcessor *ctx);
BaseType_t serialcomm_add_char(struct SerialCommProcessor *ctx, char c);
BaseType_t serialcomm_add_chars(struct SerialCommProcessor *ctx, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SERIALCOMM_H */
