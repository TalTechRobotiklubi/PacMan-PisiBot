#include <stdio.h>
#include <string.h>

int main(void)
{
    /*char cmd[512] = "45 03 05 64,64";*/
    char cmd[512] = "0A0206-5A,C8";
    int cmd_len = strlen(cmd);

    int sum = 0;

    for(int i = 0; i < cmd_len; i++){
        sum += cmd[i];
    }

    sum %= 255;

    printf("dec: %d, hex: %X\n", sum, sum);

    return 0;
}
