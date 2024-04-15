/**
 * Code for drone/bot swarm using PisiBots.
 *
 * Created: 18.12.2014 17:50:18
 * Original author (base code): Rain (TTÜ Robotiklubi)
 * 
 * Drone swarm project lead: Erki Meinberg (TTÜ Robotiklubi)
 * Drone swarm PisiBot code authors: Oliver Paljak (TTÜ Robotiklubi)
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
 * Otse sõitmiseks rakendasin PID kontrolli, kuid see tõstatab
 * küsimusi/probleeme:
 *  * Milline on meie liikumiskiirus rajal? Oluline, kuna PID konstandid
 *    sõltuvad kiirusest. Ise arvan, et võiks teha erinevad raskustasemed, et
 *    kus kergemal raskustasemel on mängija vastased robotid aeglasemad ja
 *    raskemal tasemel on mängija vastased robotid kiiremad - järelikult mitu
 *    raskustaset ja mis kiirused?
 *  * Kas me rakendame ainult otse sõitmist ja kohapeal keeramist? Kui ei, siis
 *    antud lahendus jääb kaarjas sõitmisele ette ja tuleb lisada sõtmine koos
 *    keeramisega.
 *  * PID konstandid sõltuvad ka roboti ehitusest - kui on suuremad rattad,
 *    siis konstandid on erinevad, kuid kas võib olla ka nii, et sama
 *    ehitusega, aga erinevatel robotitel võivad PID konstandid mööda panna.
 *  * Kuigi ma kasutasin PID kontrollist proportsionaalse ja tuletise osa,
 *    oleks vist olnud mõtekam kasutada proportsionaalset osa koos integraaliga
 *
 * Raadio kom:
 *  * Sõnum on kuueteistkümnendsüsteemis (2 sümbolit esindavad ühte baiti ehk
 *    kümnendsüsteemi väärtusi 0-255)
 *  * Raadio formaat on järgmine:
 *      AATLdataTLdata...C
 *      
 *      A - preamble (1 bait)
 *      T - käsu/sõnumi tüüp (1 bait)
 *      L - andmete (data) pikkus (1 bait)
 *      data - andmed ehk käsu parameeter/parameetrid (vähemalt 1 sümbol)
 *      C - checksum
 *     
 *    Näiteks:
 *      45450102C802025A0101F03058,A,C61
 *                |
 *                V
 *      A  A   T  L  d   T  L  d   T  L  d  T  L  d      C
 *      45 45; 01 02 C8; 02 02 5A; 01 01 F; 03 05 8,A,C; 61
 *                |
 *                V
 *      preambles; drive:200; turn:90; drive:15; adv:8,10,12; checksum
 *
 *    Tingimused:
 *      * A, T, L, C peavad olema täpselt kaks sümbolit (ehk esindama ainult
 *        ühe baiti)
 *      * data peab olema vähemalt üks sümbol (ehk võib olla ka lihtsalt 0,
 *        kuid ära jätta ei tohi)
 *      * Kui käsk nõuab mitut argumenti, siis tuleb need eraldada üksteisest
 *        cmd_control.h's defineeritud ARG_DELIM'iga 
 *      * Minimaalne sõnumi pikkus on 11 sümbolit (vt cmd_control.h
 *        MIN_CMD_BUF_LEN)
 *      * Ükski preamble ei tohi olla null (ehk sõnumis 00)
 *      * Näha, millisele tüübile vastab mingi käsk, vt cmd_control.h command
 *        enumit
 *      * Checksum leitakse järgmiselt: kõik sõnumi sümbolite (välja arvatud
 *        mõlemad preamble'i) ASCII väärtused liidetakse kokku ja jagatakse
 *        summa mooduliga 255. Täpsemalt vt cmd_control.c funktsiooni
 *        checksum_check.
 *  * BUGS:
 *      * Millegipärast ei tunne raadiomoodul minicom'ist (failis olevat
 *        sõnumit) saates ära terminating char'i ehk '\0'. Praegu kiireks
 *        lahenduseks muutsin com.c's ära radio_gets funktsiooni nii, et
 *        raadiokanalist lugemine lõpetakse ära siis, kui tuvastakse täht G.
 *        Huvitaval kombel aga peab ikkagi (minicomi enda pärast)
 *        olema sõnumil pärast G'd ka terminating char. Vastasel juhul ei
 *        prindi minicom enam järgmisi sõnumeid (või nii vähemalt mulle
 *        tundus).
 *  * NOTES:
 *      * Antud raadio komi on ainult testitud mock-up funktsioonidega.
 *      * Käskude arv sõltub MAX_CMD_RADIO_BUFFER (failis cmd_control.h) ja ka 
 *        failis cmd_control.c cmd_i ja cmds_malloced tüübist.
 *  * Küsimused:
 *      * Kas robotitel on kaameraga suhtlemisel igal ühel oma enda kanal või 
 *        jagavad mitu robotit ühte kanalit? Kui on mitu, siis kas preamble on
 *        vajalik?
 */

/*
 * TODO:
 *  * Driving forward and backwards+
 *  * Turning (on one place - you give degrees and robot turns that degrees)+
 *  * Getting right data (direction+distance) from drive_control+
 *  * You give a command and robot does it - that also means state handling+
 *    (well atlest initial state handling and radio com is done)
 *  * Better message format - lühemaks ja kui tahame saata keerukamaid (mitme
 *    argumendiga)+
 *  * Accurate turning (on one place - you give degrees and robot turns that
 *    degrees)
 *  * Accurate distance driving - siin peaks arvatavasti tegema
 *    (lineariseeritud) funktsiooni, mis arvutab vea
 *  * Accurate straight-line driving - probably a matter of timming the
 *    PID control constants (enam-vähem, aga võiks olla parem)
 */

/* CODE ---------------------------------------------------------------------*/
int main(void)
{
    /* Radio communications variables */
    char radio_buffer[512];
    uint32_t last_radio_com_time = 0;
    
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

    _delay_ms(1000);

    while(1){
        /* If there is a new command available, then set it as currently active
         * command (drop/stop the older command) */
        if((new_cmd = get_cmd()) != NULL){
            cmd = new_cmd;
            new_cmd = NULL;
            drive_control_reset();
            
            sprintf(radio_buffer, "Got a new cmd\n\r");
        }

        /* (Basic) State handling */
        if(cmd != NULL){
            /* Printing/debugging */
            sprintf(radio_buffer, "ld: %ld, rd: %ld, cmd %d: %d %d, t: %ld\n\r",
                    get_left_distance_mm(), get_right_distance_mm(),
                    cmd->type, *(cmd->data), cmd->done,
                    millis());
            
            if(cmd->done || cmd->type == CMD_END){
                cmd = NULL;
                drive_control_reset();
            }else if(cmd->type == CMD_DRIVE){
                if(drive_mm(*(cmd->data))) cmd->done = 1;
            }else if(cmd->type == CMD_TURN){
                if(turn_deg(*(cmd->data))) cmd->done = 1;
            }else if(cmd->type == CMD_MOTORS){
                motor_set(cmd->data[0], cmd->data[1]);
            }else{
                cmd = NULL;
                drive_control_reset();
            }
        }else{
            sprintf(radio_buffer, "cmd is NULL\n\r");
        }

        if((millis() - last_radio_com_time) >= COM_DELAY
                || last_radio_com_time == 0){

            last_radio_com_time = millis();

            /*sprintf(radio_buffer,
                    "le: %d, re: %d, ld: %ld, rd: %ld, lda: %ld, rda: %ld, e: %d, pwrr: %d, pwrl: %d, t: %ld\n\r",
                    -get_left_enc(), -get_right_enc(),
                    get_left_distance_mm(), get_right_distance_mm(),
                    get_left_abs_distance_mm(), get_right_abs_distance_mm(),
                    error, debug_pwr_left, debug_pwr_right, millis());*/

           /* sprintf(radio_buffer, "le: %d, re: %d, ld: %ld, rd: %ld\n\r",
                    -get_left_enc(), -get_right_enc(),
                    get_left_distance_mm(), get_right_distance_mm());*/

            radio_puts(radio_buffer);
        }
    }
}
