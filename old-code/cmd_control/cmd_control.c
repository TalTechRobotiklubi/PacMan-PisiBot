/**
 * File for getting commands.
 */

#include "cmd_control.h"


/* FUNCTIONS ----------------------------------------------------------------*/

/**
 * Get/parse the commands that were recieved through radio communcation.
 * 
 * Basic format:
 *      AATLdataTLdataTLdataC
 *      A - preamble
 *      T - command type
 *      L - command data length
 *      data - command data/value
 *      C - checksum
 * Message is in hexadecimal for decreasing the amount of symbols needed to
 * transfer the message. The capital letters (A, T, L) represent 1 byte (must
 * be 2 symbols). Data must be atleast 1 symbol and one preamble byte must
 * represent values from 1-255 (e.g. cannot be 00). Multiple argument command
 * data must be separated with ARG_DELIM (see cmd_control.h). Checksum is
 * calculated by adding all symbols' ASCII values after preambles and doing a
 * remainder division on that sum by 255 (or simply put: sum % 255). The
 * minimum length of the message is 11 symbols. For more details how to
 * calculate the checksum and on preamble requirements see checksum_check
 * function (in this file) and preamble_check function (in this file).
 *
 *
 * 
 * Example message (d - data):
 *      45450102C802025A0101F03058,A,C61
 *                |
 *                V
 *      A  A   T  L  d   T  L  d   T  L  d  T  L  d      C
 *      45 45; 01 02 C8; 02 02 5A; 01 01 F; 03 05 8,A,C; 61
 *                |
 *                V
 *      preambles; drive:200; turn:90; drive:15; adv:8,10,12; checksum
 * To see which command number responds to command type, see the command enum
 * in the cmd_control.h file.
 *
 * Returns: pointer to malloced array of cmd_t
 */
cmd_t *get_cmds()
{
    char radio_buf[MAX_CMD_BUF_LEN];

    if(!radio_gets(radio_buf)) return NULL;
    
    uint16_t radio_buf_len = strlen(radio_buf);
    
    if(radio_buf_len < MIN_CMD_BUF_LEN){
        /* Sending message for debugging */
        char error_buf[64];
        sprintf(error_buf,
                "ERROR: Message too short!\n\r");
        radio_puts(error_buf);
        /* Send error? NACK maybe? */
        return NULL;
    }
    
    /* Controlling the preambles */
    if(!preamble_check(radio_buf)){
        /* Sending message for debugging */
        char error_buf[64];
        sprintf(error_buf,
                "ERROR: Preamble check failed!\n\r");
        radio_puts(error_buf);
        /* Send error? NACK maybe? */ 
        return NULL;
    }
    
    /* Controlling the checksum */ 
    if(!checksum_check(radio_buf, radio_buf_len)){
        /* Send error? NACK maybe? */
        return NULL;
    }

    /* Getting the commands */
    radio_buf[radio_buf_len-2] = 0;
    char *radio_buf_ptr = radio_buf+4;
    radio_buf_len = strlen(radio_buf_ptr);
    
    uint8_t cmd_i = 0, pos = 0, cmds_malloced = MIN_CMDS;
    cmd_t *cmds = (cmd_t *) malloc(sizeof(cmd_t) * cmds_malloced);

    if(cmds == NULL){
        /* Send error? */
        return NULL;
    }

    while(radio_buf_ptr[0] != 0){
        /* Get type byte and data length byte (on different iterations) */
        uint8_t temp_value = 0;
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
             * argument count) */
            char data_str[temp_value+1];
            strncpy(data_str, radio_buf_ptr, temp_value);
            data_str[temp_value] = 0;

            if(strchr(data_str, ARG_DELIM) != NULL){
                uint8_t arg_count = 1, j = 0;
                char arg_delim = ARG_DELIM;
                char *arg;

                for(;data_str[j] != 0; j++){
                    if(data_str[j] == arg_delim){
                        arg_count++;
                    }
                }
                
                cmds[cmd_i].data_len = arg_count;
                cmds[cmd_i].data = (int16_t *) malloc(
                                                sizeof(int16_t) * arg_count);
               
                if(cmds[cmd_i].data == NULL){
                    /* Send error? */
                    return NULL;
                }

                arg = strtok(data_str, &arg_delim);
                for(j = 0; arg != NULL; j++){
                    cmds[cmd_i].data[j] = strtol(arg, NULL, 16);
                    arg = strtok(NULL, &arg_delim);
                }
            }else{
                cmds[cmd_i].data_len = 1;
                cmds[cmd_i].data = (int16_t *) malloc(sizeof(int16_t));
                
                if(cmds[cmd_i].data == NULL){
                    /* Send error? */
                    return NULL;
                }
                
                *cmds[cmd_i].data = strtol(data_str, NULL, 16);
            }
            
            pos = 0;
            radio_buf_ptr += temp_value;
            cmd_i++;
        }
    }

    
    /* Adding end command */
    cmds[cmd_i].type = CMD_END;
    cmds[cmd_i].data = NULL;
    cmds[cmd_i].data_len = 0;
    cmds[cmd_i].done = 0;

    return cmds;
}

/**
 * Verify the recieved radio message preambles. One preamble must be a value in
 * range of 1-255 or in other words - preamble's string must represent one byte
 * (except 00) in hexadecimal.
 *
 * Parameters: 
 *      radio_buf - char pointer, recieved radio message string
 *
 * Returns: uint8_t
 *      0 when atleast one of the preambles does not meet the specified
 *        requirements mentioned above
 *      1 when both of the preambles meet the specified requirements
 */
uint8_t preamble_check(char *radio_buf)
{
    for(uint8_t i = 0; i < 2; i++){
        char preamble_byte[3];
        strncpy(preamble_byte, radio_buf+i*2, 2);
        preamble_byte[2] = 0;

        char *err_ptr;
        int16_t preamble = 0;
        preamble = strtol(preamble_byte, &err_ptr, 16);

        if(err_ptr[0] != 0 || preamble < 1){
            return 0;
        }
    }

    return 1;
}

/**
 * Verify the recieved radio message checksum. Checksum is calculated by
 * adding all symbols ASCII values after preambles and doing a remainder
 * division on that sum by 255 (or simply put: sum % 255)
 *
 * ASCII table (left is in hexadecimal; right in decimal):
 *
 *        2 3 4 5 6 7       30 40 50 60 70 80 90 100 110 120
 *      -------------      ---------------------------------
 *     0:   0 @ P ` p     0:    (  2  <  F  P  Z  d   n   x
 *     1: ! 1 A Q a q     1:    )  3  =  G  Q  [  e   o   y
 *     2: " 2 B R b r     2:    *  4  >  H  R  \  f   p   z
 *     3: # 3 C S c s     3: !  +  5  ?  I  S  ]  g   q   {
 *     4: $ 4 D T d t     4: "  ,  6  @  J  T  ^  h   r   |
 *     5: % 5 E U e u     5: #  -  7  A  K  U  _  i   s   }
 *     6: & 6 F V f v     6: $  .  8  B  L  V  `  j   t   ~
 *     7: ' 7 G W g w     7: %  /  9  C  M  W  a  k   u  DEL
 *     8: ( 8 H X h x     8: &  0  :  D  N  X  b  l   v
 *     9: ) 9 I Y i y     9: '  1  ;  E  O  Y  c  m   w
 *     A: * : J Z j z
 *     B: + ; K [ k {
 *     C: , < L \ l |
 *     D: - = M ] m }
 *     E: . > N ^ n ~
 *     F: / ? O _ o DEL
 *
 * For reference: A is 65 and 0x41; 0 (zero) is 48 and 0x30.
 *
 * Checksum calculation example:
 *      45450102C802025A0101F03058,A,C
 * First four symbols (representing 2 bytes) are preamble - they will be
 * skipped:
 *      0102C802025A0101F03058,A,C
 *                 |
 *                 V
 *0  1  0  2  C  8  0  2  0  2  5  A  0  1  0  1  F  0  3  0  5  8  ,  A  ,  C 
 *48+49+48+50+67+56+48+50+48+50+53+65+48+49+48+49+70+48+51+48+53+56+44+65+44+67
 *                 |
 *                 V
 *             sum = 1372
 *                 |
 *                 V
 *      Checksum = sum % 255 = 97
 *                 |
 *                 V
 * Message itself is in hexadecimal, so the checksum must also be converted to 
 * hexadecimal (97 = 0x61). So the complete message is this:
 *      45450102C802025A0101F03058,A,C61
 *
 * For other details on the radio message (parsing) see get_cmds function (in
 * this file).
 *
 * Parameters: 
 *      radio_buf - char pointer, recieved radio message string
 *      radio_buf_len - uin16_t, recieved radio message length
 *
 * Returns: uint8_t
 *      0 when the checksums do not match
 *      1 when the chekcsum match
 */
uint8_t checksum_check(char *radio_buf, uint16_t radio_buf_len)
{
    char checksum_byte[3];
    strncpy(checksum_byte, radio_buf+radio_buf_len-2, 2);
    checksum_byte[2] = 0;
    uint8_t checksum = strtol(checksum_byte, NULL, 16);
    
    uint16_t i = 4, sum = 0;
    while(i < radio_buf_len-2){
        sum += radio_buf[i];
        i++;
    }

    if((sum %= 255) != checksum){
        /* Sending message for debugging */
        char error_buf[64];
        sprintf(error_buf,
                "ERROR: Checksums failed!\n\rCalculated: %d\n\rIn msg: %d\n\r",
                sum, checksum);
        radio_puts(error_buf);
        return 0;
    }

    return 1;
}

/**
 * Free malloced cmds
 *
 * Parameters: cmds - cmd_t pointer, pointer to the malloced array
 */
void free_cmds(cmd_t *cmds)
{
    if(cmds != NULL){
        for(uint16_t i = 0; cmds[i].type != CMD_END; i++){
            if(cmds[i].data != NULL){
                free(cmds[i].data);
            }
        }
        
        free(cmds);
    }
}

/**
 * Get commands from radio communication
 *
 * Returns: cmd_t pointer/array, array of structs that contain the new 
 *          commands. See also cmd_control.h for the struct. Returns NULL
 *          pointer if no command was read from the radio communication.
 */
/*cmd_t *get_cmds()
{
    * See constants from cmd_control.h */
    /*char radio_buffer[MAX_CMD_RADIO_BUFFER];

    if(radio_gets(radio_buffer)){ 
        uint8_t i = 0, j = 0, cmd_part = 0;
        char cmd_strings[MAX_CMDS][MAX_CMD_LEN];
       
        * Separate single command strings from the whole radio buffer */
        /*char *buffer_ptr = strtok(radio_buffer, ";");
        while(buffer_ptr != NULL && i < MAX_CMDS-1){
            strcpy(cmd_strings[i], buffer_ptr);
            buffer_ptr = strtok(NULL, ";");
            i++;
        }
        * Last command will always be the "end" command */
        /*strcpy(cmd_strings[i], "end:0\0");

        * Separate command type and value from the single command strings */
        /*for(j = 0; j < i+1; j++){
            cmd_part = 0;
            char *cmd_ptr = strtok(cmd_strings[j], ":");
            
            while(cmd_ptr != NULL){
                if(cmd_part == 0){
                    strcpy(cmds[j].type, cmd_ptr);
                }else{
                    cmds[j].value = strtol(cmd_ptr, NULL, 10);
                }
                
                cmds[j].done = 0;
                cmd_ptr = strtok(NULL, ":");
                cmd_part++;
            }
        }
        * For the last (end) command */
        /*cmds[j].done = 1;
        
        return cmds;
    }

    return NULL; 
}*/
