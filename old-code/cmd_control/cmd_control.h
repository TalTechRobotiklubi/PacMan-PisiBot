/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "drivers/com.h"

/* CONSTANTS ----------------------------------------------------------------*/
/* The maximum length for the radio buffer where all the commands are read */
#define MAX_CMD_BUF_LEN 512
/* The minimum length of the radio buffer. If the message is shorter than that
 * then it will be ignored.*/
#define MIN_CMD_BUF_LEN 11
/* The minimum amount of commands that is going to be allocated from get go */
#define MIN_CMDS 4
/* Delimeter for separating command's data, which has multiple arguments */
#define ARG_DELIM ','

/* ENUMS --------------------------------------------------------------------*/
enum command_enum{
    CMD_END = 0,
    CMD_DRIVE = 1,
    CMD_TURN = 2,
    CMD_ADV = 3
};

/* STURCTS ------------------------------------------------------------------*/
typedef struct cmd{
    uint8_t type;
    int16_t *data;
    uint8_t data_len;
    uint8_t done;
} cmd_t;

/* PROTOTYPES ---------------------------------------------------------------*/
void free_cmds(cmd_t *cmds);
uint8_t preamble_check(char *radio_buf);
uint8_t checksum_check(char *radio_buf, uint16_t radio_buf_len);
cmd_t *get_cmds();
