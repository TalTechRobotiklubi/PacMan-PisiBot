/**
 * File for getting commands. Part of the drone/bot swarm project.
 *
 * NOTE: If you see terms "radio buffer" or "radio channel", then it is usually
 *       meant to indicate the whole string that was recieved through the UART.
 *       If you see terms "radio message" or "message", then it usually
 *       indicates one command part of the recieved radio buffer.
 *
 * NOTE: Do not confuse the length byte in the message and cmd_t.data_len -
 *       they are different things. The cmd_t.data_len represents the
 *       cmd_t.data array length/command argument count, while the length byte
 *       in the message represents the data substring length.
 */

#include "cmd_control.h"

/* PRIVATE PROTOTYPES -------------------------------------------------------*/
uint8_t jump_to_preamble(char **radio_buf);
uint8_t get_byte(char *radio_buf, uint16_t offset);
uint8_t check_checksum(char *radio_buf, uint8_t data_len);
uint8_t get_data(char *radio_buf, uint8_t data_len);

/* PRIVATE GLOBALS ----------------------------------------------------------*/
/* Current command */
cmd_t cmd;

/* Radio buffer (memory for the radio_buf_ptr) */
char radio_buffer[CMDC_MAX_BUF_LEN];

/* Radio buffer pointer - used to parse the buffer */
char *radio_buf_ptr = radio_buffer;

/* FUNCTIONS ----------------------------------------------------------------*/
/**
 * Initialize command control.
 */
void init_cmd_control()
{
    cmd.type = CMD_END;
    cmd.data_len = 0;
    cmd.done = 1;

    uint16_t i = 0;
    /* Setting the data array to zero just in case */
    for(; i < CMDC_MAX_DATA_ARG_LEN; i++){
        cmd.data[i] = 0;
    }
}

/**
 * Get/parse the commands that were recieved through radio communcation.
 *
 * Basic format (one message): AAITLdataC
 *      A - preamble
 *      I - ID/aadress
 *      T - command type
 *      L - command data length
 *      data - command data/value
 *      C - checksum
 *
 * Message is in hexadecimal for decreasing the amount of symbols needed to
 * transfer the message. The minimum length of the message is 13 symbols.
 *
 * The capital letters (A, I, T, L, C) represent 1 byte (must be 2 symbols). I,
 * L cannot represent the value 0 (cannot be 00).
 *
 * Data must be atleast 1 symbol. Multiple argument command data must be
 * separated with ARG_DELIM (see cmd_control.h). The argument limit is 256.
 *
 * Checksum is calculated by adding all symbols' ASCII values after preambles
 * and doing a remainder division on that sum by 255 (or simply put:
 * sum % 255). For more details how to calculate the checksum see
 * check_checksum function (in this file).
 *
 * Example message (d - data):
 *       000069030712C,1F4B8
 *                |
 *                V
 *     A  A   I   T  L  d        C
 *     00 00; 69; 03 07 12C,8AC; B8
 *                |
 *                V
 * preambles; robot_id=105; motors_set:300,500; checksum
 *
 * To see which command number responds to command type, see the command enum
 * in the cmd_control.h file.
 *
 * NOTE: It is very likely that when the robot reads from the radio buffer then
 *       the radio buffer has also messages for other robots. So the situation
 *       in the radio buffer is probably something like this:
 *       AAITLdataCAAITLdataCAAITLdataCAAITLdataC
 *       where also there could be corrupted messages. The parser should handle
 *       those situations accordingly.
 *
 * Returns: pointer to cmd_t
 */
cmd_t *get_cmd()
{
    if(!radio_gets(radio_buf_ptr) && *radio_buf_ptr == 0) return NULL;

    /* Check message length */
    uint16_t radio_buf_len = strnlen(radio_buf_ptr, CMDC_MAX_BUF_LEN);
    if(radio_buf_len < CMDC_MIN_BUF_LEN || radio_buf_len == CMDC_MAX_BUF_LEN){
        /* Clearing out the buffer for indicating that parsing is completed */
        radio_buffer[0] = 0;
        radio_buf_ptr = radio_buffer;
        return NULL;
    }

    /* Find the next message */
    if(!jump_to_preamble(&radio_buf_ptr)){
        radio_buffer[0] = 0;
        radio_buf_ptr = radio_buffer;
        return NULL;
    }

    /* Check ID */
    uint8_t id = get_byte(radio_buf_ptr, OFFSET_ID);
    if(id != ROBOT_ID && id != 255){
        /* Indicator for moving to the next message */
        radio_buf_ptr += 4;
        return NULL;
    }

    /* Get length of data substring in the message */
    uint8_t data_len = get_byte(radio_buf_ptr, OFFSET_LEN);
    if(!data_len){
        radio_buf_ptr += 4;
        return NULL;
    }

    /* Check checksum */
    if(!check_checksum(radio_buf_ptr, data_len)){
        radio_buf_ptr += 4;
        return NULL;
    }

    /* Get command type */
    uint8_t cmd_type = get_byte(radio_buf_ptr, OFFSET_TYPE);
    if(cmd_type > CMDC_LAST_CMD_TYPE){
        radio_buf_ptr += 4;
        return NULL;
    }

    /* Get command data and data array length (cmd.data_len) */
    if(!get_data(radio_buf_ptr, data_len)){
        radio_buf_ptr += 4;
        return NULL;
    }

    cmd.type = cmd_type;
    cmd.done = 0;
    radio_buf_ptr += 4;
    return &cmd;
}

/**
 * Jump to nearest preamble. See the CMDC_PREAMBLE constant in cmd_control.h
 *
 * Parameters:
 *      radio_buf - pointer to string, The pointer to the radio buffer pointer.
 *                  NOTE: The radio buffer pointer will be modified (it will be
 *                  moved to the start of the nearest preamble).
 *
 * Returns:
 *      0 if the jump was unsuccessful (preamble was not found in the buffer)
 *      1 if the jump was successful (preamble was found in the radio buffer)
 */
uint8_t jump_to_preamble(char **radio_buf)
{
    if((*radio_buf = strstr(*radio_buf, CMDC_PREAMBLE)) != NULL){
        return 1;
    }

    return 0;
}

/**
 * Get byte by offset.
 *
 * Paramters:
 *      radio_buf - string, Radio buffer (must be jumped to the beginning of
 *                  the desired message, see jump_to_preamble function)
 *      offset - uint16_t, Offset from the start of the message. NOTE: The
 *               offset should point to the beginning of the needed byte.
 *
 * Returns: uint8_t. If the byte was successfully parssed, the return value is
 *          the byte's value. If the byte parsing fails, the return value is
 *          0 - so it would be wise to avoid using 00 as a byte value.
 */
uint8_t get_byte(char *radio_buf, uint16_t offset)
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

/**
 * Get/parse the data of the recieved message.
 *
 * NOTE: Multiple arguments in the message should be separated by ARG_DELIM
 *       defined in cmd_control.h
 * NOTE: This function does not return a data filled cmd_t struct but it
 *       changes the global cmd struct.
 * NOTE: Do not confuse the length byte in the message and cmd_t.data_len -
 *       they are different things. The cmd_t.data_len represents the
 *       cmd_t.data array length/command argument count, while the length byte
 *       in the message represents the data substring length.
 *
 * Parameters:
 *      radio_buf - string, Radio buffer (must be jumped to the beginning of
 *                  the desired message, see jump_to_preamble function)
 *      data_len - uint16_t, length of data substring in the message
 *
 * Returns:
 *      0 if data conversion was unsuccessful
 *      1 if data conversion was successful
 */
uint8_t get_data(char *radio_buf, uint8_t data_len)
{
    char *data_str = radio_buf+OFFSET_DATA;

    uint16_t data_str_len = strnlen(data_str, CMDC_MAX_BUF_LEN);
    if(data_str_len < data_len || data_str_len == CMDC_MAX_BUF_LEN){
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

/**
 * Verify the checksum of the message. Checksum is calculated by adding all
 * symbols ASCII values after preambles and doing a remainder division on that
 * sum by 255 (or simply put: sum % 255)
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
 *          000069030712C,1F4
 * First four symbols (representing 2 bytes) are preamble - they will be
 * skipped:
 *            69030712C,1F4
 *                 |
 *                 V
 *  6  9  0  3  0  7  1  2  C  ,  1  F  4
 *  54+57+48+51+48+55+49+50+67+44+49+70+52
 *                 |
 *                 V
 *             sum = 694
 *                 |
 *                 V
 *       Checksum = sum % 255 = 184
 *                 |
 *                 V
 * Message itself is in hexadecimal, so the checksum must also be converted to
 * hexadecimal (184 = 0xB8). So the complete message is this:
 *          000069030712C,1F4B8
 *
 * For other details on the radio message (parsing) see get_cmd function (in
 * this file).
 *
 * Parameters:
 *      radio_buf - string, Radio buffer (must be jumped to the beginning of
 *                  the desired message, see jump_to_preamble function)
 *      data_len - uint16_t, length of data substring in the message
 *
 * Returns:
 *      0 if the checksum and the calculated checksum do not match (checksum
 *        check fails)
 *      1 if the checksum and the calculated checksum match (checksum check
 *        succeeds)
 */
uint8_t check_checksum(char *radio_buf, uint8_t data_len)
{
    uint16_t checksum_offset = (uint16_t) (OFFSET_DATA + data_len);
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

    return 0;
}

