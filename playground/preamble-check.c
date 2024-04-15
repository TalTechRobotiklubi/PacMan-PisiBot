#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

uint8_t preamble_check(char *radio_buf);

int main(void)
{
    printf("%d\n", preamble_check("GG10"));
    /* preamble_check("45450102C802025A0101F03058,A,C61"); */
}

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
