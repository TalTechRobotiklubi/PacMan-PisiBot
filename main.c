/**
 * Drivers for PisiBots (v5) for the Pac-Man project.
 *
 * Original author (base code): Rain (TTÜ Robotiklubi)
 *
 * Project lead: Erki Meinberg (TTÜ Robotiklubi)
 * PisiBot v5 (i.e. current) drivers: Oliver Paljak (TTÜ Robotiklubi)
 * PisiBot v6 electronics and drivers: Märtin Laanemets (TTÜ Robotiklubi)
 *
 * Some information on the robot:
 * https://www.robotiklubi.ee/projektid/arendusprojektid/pisi_xbee5#atmel_studio_template
 * https://www.robotiklubi.ee/projektid/arendusprojektid/pisi_xbee5/teek
 *
 */

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include "drivers/board.h"
#include "drivers/com.h"

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "drive_control.h"
#include "cmd_control.h"

/* CONSTANTS ----------------------------------------------------------------*/
/* Delay in milliseconds for radio communication delay */
#define COM_DELAY 80

/**
 * If robot does not recieve any commands in KILL_SWITCH_TIME (ms), then it
 * stops all acitivity (drops the active command)
 */
#define KILL_SWITCH_TIME 5000

/*
 * TODO:
 *  * Accurate turning on one place - you give degrees and robot turns that
 *    degrees - kinda done?
 *  * When starting on a high speed the robot turns off course at the very
 *    beginning - probably it would be smart idea to have some grace period:
 *    robot accelarates to the desired (high) speed.
 *  * Timming the PID constants for straight line driving
 *  * Fix known bugs
 *
 * DONE:
 *  * Kill switch - robot drops the (motor_set) command if there has been no
 *    communication between the robot and the camera for some time+
 *  * New format for drive_mm and turn_deg command messages (add pwr argument;
 *    removal of the C_POWER constant)+
 *  * Driving forward and backwards (generally)+
 *  * Turning (generally)+
 *  * Getting right data (direction+distance) from drive_control+
 *  * You give a command and robot does it - that also means state handling+
 *  * Radio com message format - shorter and multiple arguments+
 *  * Accurate distance driving - siin peaks arvatavasti tegema
 *    (lineariseeritud) funktsiooni, mis arvutab vea+
 *  * Accurate straight-line driving - probably a matter of timming the
 *    PID control constants (so-so, but can be better)+
 *  * Doing PID as proportional and integral, not proportional and derative+
 *
 * KNOWN BUGS:
 *   * In radio com there has to be an end letter (e.g. "G") to indicate the
 *     end of the whole buffer (see drivers/com.c:radio_gets()). How to fix: ?
 *     (it is not critical)
 *
 * FIXED BUGS:
 *   * Radio com parsing ("the odd bug") - for some reason the multiple
 *     argument data parser returns quite bizarre results. For example, if
 *     data is -500,-500, then the parsed values are 500,-500; if data is
 *     -500,500, then the parsed values are 500,500. It is important to note
 *     that the unparsed data is always correct - problems arise right in the
 *     parsing. My guess is that the robot cannot handle the strtok function as
 *     I have failed to reproduce similar results on my own computer. It seems
 *     that strtok does not like the fact that the first value is negative -
 *     other values (such as 500,500 and 500,-500) are always parsed correctly.
 *     How to fix: rewrite multiple argument parsing without the aid of strtok.
 *     Should be fixed - rewrote the function without the aid of strtok.
 */

/* CODE ---------------------------------------------------------------------*/
int main(void)
{
    /* Radio communications variables (for sending (debug) messages) */
    char radio_buffer[512];
    uint32_t last_radio_com_time = 0;

    /* Kill switch variables */
    uint32_t last_cmd_time = 0;

    /* State handling variables */
    cmd_t *new_cmd = NULL;
    cmd_t *cmd = NULL;

    /* Set the system clock to 32MHz */
    clock_init();
    /* Set up the LED and buttons */
    board_init();
    /* Init drive control */
    drive_control_init();
    /* Init command control */
    init_cmd_control();

    /*
     * More accurate radio set up goes through a program called XCTU.
     * Here we will just set the right baud - right now 57600.
     */
    radio_init(57600);

    rgb_set(BLUE);
    while(!sw1_read());
    rgb_set(GREEN);

    /**
     * Radio communication problem fix - TEMPSOL
     */
    radio_puts("1\n\r");

    _delay_ms(1000);


    while(1){
        /* If there is a new command available, then set it as currently active
         * command (drop/stop the older command) */
        if((new_cmd = get_cmd()) != NULL){
            cmd = new_cmd;
            new_cmd = NULL;
            last_cmd_time = millis();
            drive_control_reset();

            sprintf(radio_buffer, "Got a new cmd\n\r");
        }

        /* State handling */
        if(cmd != NULL){
	    /* For debugging */
            /* sprintf(radio_buffer, "ld: %ld, rd: %ld, cmd %d: %d %d %d, t: %ld\n\r",
                    get_left_distance_mm(), get_right_distance_mm(),
                    cmd->type, *(cmd->data), *(cmd->data+1), cmd->done,
                    millis()); */

            sprintf(radio_buffer,
                    "le: %d, re: %d, err: %d, pwrl: %d, pwrr: %d, t: %ld\n\r",
                    -get_left_enc(), -get_right_enc(), error, debug_pwr_left,
                    debug_pwr_right, millis());

            if(cmd->done || cmd->type == CMD_END){
                cmd = NULL;
                drive_control_reset();
            }else if(cmd->type == CMD_DRIVE){
                if(drive_mm(cmd->data[0], cmd->data[1])) cmd->done = 1;
            }else if(cmd->type == CMD_TURN){
                if(turn_deg(cmd->data[0], cmd->data[1])) cmd->done = 1;
            }else if(cmd->type == CMD_MOTORS){
                drive(cmd->data[0], cmd->data[1]);
            }else{
                cmd = NULL;
                drive_control_reset();
            }
        }else{
            /* sprintf(radio_buffer, "cmd is NULL\n\r"); */
            sprintf(radio_buffer, "\n\r");
        }

        /* Kill switch logic */
        if((millis() - last_cmd_time) >= KILL_SWITCH_TIME){
            cmd->type = CMD_END;
        }

	/* For debugging */
        /*if((millis() - last_radio_com_time) >= COM_DELAY
                || last_radio_com_time == 0){

            last_radio_com_time = millis();*/

            /*sprintf(radio_buffer,
                    "le: %d, re: %d, ld: %ld, rd: %ld, lda: %ld, rda: %ld, e: %d, pwrr: %d, pwrl: %d, t: %ld\n\r",
                    -get_left_enc(), -get_right_enc(),
                    get_left_distance_mm(), get_right_distance_mm(),
                    get_left_abs_distance_mm(), get_right_abs_distance_mm(),
                    error, debug_pwr_left, debug_pwr_right, millis());*/

           /* sprintf(radio_buffer, "le: %d, re: %d, ld: %ld, rd: %ld\n\r",
                    -get_left_enc(), -get_right_enc(),
                    get_left_distance_mm(), get_right_distance_mm());*/

            /*radio_puts(radio_buffer);*/
        /*}*/
    }
}

