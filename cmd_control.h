#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "drivers/com.h"

/* CONSTANTS ----------------------------------------------------------------*/
/**
 * The robot's ID.
 *
 * NOTE: The message is in hexadecimal, so it is easier to store the id also in
 * hexadecimal.
 */
#define ROBOT_ID 0x45

/* The preamble for every message/command */
#define CMDC_PREAMBLE "0000"

/**
 * The minimum length of the radio buffer string (minimal length of one
 * command message)
 */
#define CMDC_MIN_BUF_LEN 13

/* The maximum length of the whole radio buffer/channel string */
#define CMDC_MAX_BUF_LEN 1024

/* The maximum argument count for cmd_t.data array */
#define CMDC_MAX_DATA_ARG_LEN 256

/**
 * The last command type - if the command type is bigger in the message than
 * the value defined here, then the message will be rejectd
 */
#define CMDC_LAST_CMD_TYPE 3

/* Delimeter for separating command's data, which has multiple arguments */
#define ARG_DELIM ','

/* STURCTS ------------------------------------------------------------------*/
/**
 * The command data type.
 *
 * NOTE: the data_len field should not be confused with data length in the
 * message (data length byte). Here the data_len refers to the data array
 * length, not to the length of data substring.
 */
typedef struct cmd_struct{
    uint8_t type;
    int16_t data[CMDC_MAX_DATA_ARG_LEN];
    uint8_t done;
    uint8_t data_len;
} cmd_t;

/* ENUMS --------------------------------------------------------------------*/
/* Command types enum */
enum cmdc_cmd_enum{
    CMD_END = 0,
    CMD_DRIVE = 1,
    CMD_TURN = 2,
    CMD_MOTORS = 3
};

/**
 * Commands offset enum. It shows where different parts of the message begin
 * (e.g. when the OFFSET_ID is 4, then it shows that ID part of the message
 * starts at index 4)
 */
enum cmdc_offset_enum{
    OFFSET_ID = 4,
    OFFSET_TYPE = 6,
    OFFSET_LEN = 8,
    OFFSET_DATA = 10
};

/* PUBLIC PROTOTYPES --------------------------------------------------------*/
void init_cmd_control();
cmd_t *get_cmd();

#endif

