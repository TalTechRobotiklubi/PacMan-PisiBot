/* HEADER -------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define CMDC_MAX_BUF_LEN 512
#define CMDC_MIN_BUF_LEN 13
#define ARG_DELIM ','

#define MAX_DATA_ARG_LEN 256

#define ROBOT_ID 0x69
#define CMDC_PREAMBLE "0000"

enum cmdc_cmd_enum{
    CMD_END = 0,
    CMD_DRIVE = 1,
    CMD_TURN = 2,
    CMD_MOTORS = 3
};

enum cmdc_offset_enum{
    OFFSET_ID = 4,
    OFFSET_TYPE = 6,
    OFFSET_LEN = 8,
    OFFSET_DATA = 10
};

typedef struct cmd_struct{
    uint8_t type;
    int16_t data[MAX_DATA_ARG_LEN];
    uint8_t data_len;
} cmd_t;

void init_cmd_control();
cmd_t *get_cmd();
uint8_t get_byte(char *radio_buf, uint8_t offset);
uint8_t check_checksum(char *radio_buf, uint8_t checksum_offset);
uint8_t jump_to_preamble(char **radio_buf);
uint8_t get_data(char *radio_buf, uint8_t data_len);

/* PROGRAM ------------------------------------------------------------------*/
/* 
 * A (preamble) - 1 bait (2 symbolit)
 * I (ID) - 1 bait (2 symbolit)
 * T (tyyp) - 1 bait (2 symbolit)
 * L (pikkus) - 1 bait (2 symbolit)
 * d (andmed/args) - vähemalt 1 symbol
 * C (checksum) - 1 bait (2 symbolit)
 *
 * A  A   I   T  L  d   C
 * 00 00; 69; 02 02 5A; AA
 */
char radio_buffer[CMDC_MAX_BUF_LEN] = "0000690306-C8,C889";
char *radio_buf_ptr = radio_buffer;

cmd_t cmd;

int main(void)
{
    init_cmd_control();
    
    uint16_t i = 0;
    for(; i < 1024 /* just replaces while(1) */; i++){
        get_cmd();
    }

    return 0;
}

void init_cmd_control()
{
    cmd.type = CMD_END;
    cmd.data[0] = 0;
    cmd.data_len = 0;
}

cmd_t *get_cmd()
{
    if(/*!radio_gets(radio_buf_ptr) &&*/ *radio_buf_ptr == 0) return NULL;
    /*printf("Buffer before jump to nearest preamble: %s\n", radio_buf_ptr);*/
    
    /* Check message length */
    if(strnlen(radio_buf_ptr, 512) < CMDC_MIN_BUF_LEN){
        printf("Message too short! Aborting...\n\n");
        radio_buffer[0] = 0;
        radio_buf_ptr = radio_buffer; 
        return NULL;
    } 
    
    /* Find the next message */
    if(!jump_to_preamble(&radio_buf_ptr)){
        printf("Preamble was not found! Aborting...\n\n");
        radio_buffer[0] = 0;
        radio_buf_ptr = radio_buffer; 
        return NULL;
    }
    printf("Buffer after jump to nearest preamble: %s\n", radio_buf_ptr);
    
    /* Check ID */ 
    if(get_byte(radio_buf_ptr, OFFSET_ID) != ROBOT_ID){
        printf("Not correct ID!\n\n");
        /* Indicator for moving to the next message */
        radio_buf_ptr += 4;
        return NULL;
    }
    
    /* Get data length */ 
    uint8_t data_len = get_byte(radio_buf_ptr, OFFSET_LEN);
    if(!data_len){
        /* Send NACK? */
        printf("Data length cannot be 0!\n\n");
        radio_buf_ptr += 4;
        return NULL;
    }
    
    /* Check checksum */ 
    if(!check_checksum(radio_buf_ptr, OFFSET_DATA+data_len)){
        /* Send NACK? */ 
        printf("Checksum failed!\n\n");
        radio_buf_ptr += 4;
        return NULL;
    }
    
    /* Get command type */
    uint8_t cmd_type = get_byte(radio_buf_ptr, OFFSET_TYPE);
    if(!cmd_type || cmd_type > 3){
        /* Send NACK? */
        printf("Wrong cmd type!\n\n");
        radio_buf_ptr += 4;
        return NULL;
    }
    
    /* Get command data and data argument count (cmd.data_len) */
    if(!get_data(radio_buf_ptr, data_len)){
        /* Send NACK? */
        printf("Could not extract data!\n\n");
        radio_buf_ptr += 4;
        return NULL;
    }

    cmd.type = cmd_type;
    
    printf("Successfully converted a msg!\nTranslated msg:\n\tType: %d\n\tData len: %d\n\tData: ",
            cmd.type, cmd.data_len);
    
    uint8_t i = 0;
    for(; i < cmd.data_len; i++){
        if(i+1 == cmd.data_len){
            printf("%d\n\n", cmd.data[i]);
        }else{
            printf("%d, ", cmd.data[i]);
        }
    }
    
    
    radio_buf_ptr += 4;

    return &cmd;
}

uint8_t jump_to_preamble(char **radio_buf)
{
    if((*radio_buf = strstr(*radio_buf, CMDC_PREAMBLE)) != NULL){
        return 1;
    } 
    
    return 0;
}

uint8_t get_byte(char *radio_buf, uint8_t offset)
{
    char byte[3];
    strncpy(byte, radio_buf+offset, 2);
    byte[2] = 0;

    char *err_ptr;
    uint8_t byte_val = strtol(byte, &err_ptr, 16);

    if(err_ptr[0] != 0){
        return 0;
    }

    return byte_val;
}

uint8_t check_checksum(char *radio_buf, uint8_t checksum_offset)
{
    uint16_t checksum = (uint16_t) get_byte(radio_buf, checksum_offset);
    uint16_t calced_sum = 0;
    
    uint16_t i = OFFSET_ID;
    for(; i < checksum_offset; i++){
        calced_sum += radio_buf[i];
    }
    calced_sum %= 255;


    if(calced_sum == checksum){
        return 1;
    }
    
    printf("Checksum in msg: 0x%X\nCalced checksum: 0x%X\n",
            checksum, calced_sum);
    return 0;
}

uint8_t get_data(char *radio_buf, uint8_t data_len)
{
    char *data_str = radio_buf+OFFSET_DATA;
    /* Kasuta siin strnlen */
    if(strnlen(data_str, 512) < data_len){
        return 0;
    }
    data_str[data_len] = 0;

    if(strchr(data_str, ARG_DELIM) != NULL){
        uint8_t i = 0;
        uint8_t arg_count = 1;

        for(; data_str[i] != 0; i++){
            if(data_str[i] == ARG_DELIM){
                arg_count++;
            }
        }
        
        for(i = arg_count; i > 0; i--){
            char *last_delim_ptr = strrchr(data_str, ARG_DELIM);

            uint8_t delim_i = 0;
            if(last_delim_ptr != NULL){
                delim_i = (uint8_t) (last_delim_ptr - data_str)+1;
            }
            
            char *err_ptr;
            int16_t data_val = strtol(data_str+delim_i, &err_ptr, 16);

            if(err_ptr[0] != 0){
                return 0;
            }
            
            cmd.data[i-1] = data_val;
            data_str[delim_i-1] = 0;
        }

        cmd.data_len = arg_count;
        return 1;
    }else{
        char *err_ptr;
        int16_t data_val = strtol(data_str, &err_ptr, 16);

        if(err_ptr[0] != 0){
            return 0;
        }

        cmd.data[0] = data_val;
        cmd.data_len = 1;

        return 1;
    }

    return 0;
}
