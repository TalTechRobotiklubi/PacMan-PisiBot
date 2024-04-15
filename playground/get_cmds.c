#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MIN_CMDS 4
#define ARG_DELIM ','

enum commands{
    END_CMD = 0,
    DRIVE_CMD = 1,
    TURN_CMD = 2,
    ADV_CMD = 3
};

typedef struct cmd{
    int type;
    int *data;
    int data_len;
    int done;
} cmd_t;

cmd_t *get_cmds();
void free_cmds();

int main(void)
{
    cmd_t *cmds = get_cmds();

    if(cmds != NULL){
        for(int i = 0; i < 5; i++){
            printf("%d: ", cmds[i].type);
            for(int j = 0; j < cmds[i].data_len; j++){
                printf("%d ", cmds[i].data[j]);
            }

            if(cmds[i].data == NULL){
                printf("END");
            }
            printf("\n");
        }
        
    }

    free_cmds(cmds);
    
    return 0;
}

void free_cmds(cmd_t *cmds)
{
    if(cmds != NULL){
        for(int i = 0; cmds[i].type != END_CMD; i++){
            if(cmds[i].data != NULL){
                free(cmds[i].data);
            }
        }
        
        free(cmds);
    }
}

int control_checksum(char *radio_buf)
{
    int radio_buf_len = strlen(radio_buf);

    char checksum_byte[3];
    strncpy(checksum_byte, radio_buf+radio_buf_len-2, 2);
    checksum_byte[2] = 0;

    int checksum = strtol(checksum_byte, NULL, 16);
    int i = 4, sum = 0;
    while(i < radio_buf_len-2){
        sum += radio_buf[i];
        i++;
    }

    printf("%d ?= %d\n", (sum % 255), checksum);

    if((sum %= 255) != checksum) return 0;

    return 1;
}

cmd_t *get_cmds()
{
    /* preambles;drive:200;turn:90;drive:15;advanced:8,10,12;checksum */
    /* Checksum = data_chars_sum % 255 */
    /* 45 45; 01 02 C8 ; 01 02 5A ; 00 01 F; 03 05 8,A,C; 61 */
    /*char radio_buf[255] = "45450102C802025A0101F03058,A,C61"; */
    /* char radio_buf[255] = "45450102C802025A0101F83"; */
    char radio_buf[255] = "Tere";
    
    /* Control the checksum */
    if(!control_checksum(radio_buf)){
        printf("Checksums do not match:\n");
        /* printf("\tCounted: 0x%x\n\tIn msg: 0x%x\n", sum, checksum); */
        /* Send error? */
        return NULL;
    }

    /* Getting the commands */
    int radio_buf_len = strlen(radio_buf);
    radio_buf[radio_buf_len-2] = 0;
    char *radio_buf_ptr = radio_buf+4;
    radio_buf_len = strlen(radio_buf_ptr);
    
    int cmd_i = 0, pos = 0, cmds_malloced = MIN_CMDS;
    cmd_t *cmds = (cmd_t *) malloc(sizeof(cmd_t) * cmds_malloced);
   
    while(radio_buf_ptr[0] != 0){
        /* Get type byte and data length byte (on different iterations) */
        int temp_value = 0;
        char temp_byte[3];
        
        strncpy(temp_byte, radio_buf_ptr, 2);
        temp_byte[2] = 0;
        temp_value = strtol(temp_byte, NULL, 16);
        
        radio_buf_ptr += 2;
        
        /* Add more memory if we exceed the initial amount of commands */
        if(cmd_i+1 >= cmds_malloced){
            cmds_malloced += MIN_CMDS;
            cmds = (cmd_t *) realloc(cmds, sizeof(cmd_t) * cmds_malloced);
        }

        if(pos == 0){
            /* Get command type. Here temp_value represents command type */
            cmds[cmd_i].type = temp_value;
            cmds[cmd_i].done = 0;
            pos++;
        }else if(pos == 1){
            /* Get command data. Here temp_value represents command's data
             * length in a string (NOTE: temp_value != the data_len variable 
             * inside cmd_t. The data_len inside the cmd_t must equal to 
             * argument count */
            char data_str[temp_value+1];
            strncpy(data_str, radio_buf_ptr, temp_value);
            data_str[temp_value] = 0;

            if(strchr(data_str, ARG_DELIM) != NULL){
                int arg_count = 1, j = 0;
                char arg_delim = ARG_DELIM;
                char *arg;

                for(;data_str[j] != 0; j++){
                    if(data_str[j] == arg_delim){
                        arg_count++;
                    }
                }
                
                cmds[cmd_i].data_len = arg_count;
                cmds[cmd_i].data = (int *) malloc(sizeof(int) * arg_count);
                
                arg = strtok(data_str, &arg_delim);
                for(j = 0; arg != NULL; j++){
                    cmds[cmd_i].data[j] = strtol(arg, NULL, 16);
                    arg = strtok(NULL, &arg_delim);
                }
            }else{
                cmds[cmd_i].data_len = 1;
                cmds[cmd_i].data = (int *) malloc(sizeof(int));
                *(cmds[cmd_i].data) = strtol(data_str, NULL, 16);
            }
            
            pos = 0;
            radio_buf_ptr += temp_value;
            cmd_i++;
        }
        
    }
    
    /* Adding end command */
    cmds[cmd_i].type = END_CMD;
    cmds[cmd_i].data = NULL;
    cmds[cmd_i].data_len = 0;
    cmds[cmd_i].done = 0;

    printf("malloc: %d\n", cmds_malloced);    
    return cmds;
}
