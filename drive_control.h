#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <math.h>
#include <stdlib.h>
#include "drivers/motor.h"

/* CONSTANTS ----------------------------------------------------------------*/
/**
 * The maximum absolute power that goes to our motors. Must be positive and
 * should not be bigger than 1000.
 */
#define DRIVEC_MAX_PWR 800

/**
 * NOTE: Only one (either PI or PD control) can be used. Comment/uncomment the
 * necessary parts in pid_control function (in drive_control.c)
 */
/* The proportional constant for PID control */
#define DRIVEC_P_CONST 0.05f
/* The derivative constant for PD control */
#define DRIVEC_D_CONST 0.1f
/* The integral constant for PI control */
#define DRIVEC_I_CONST 0.0006f
/* The maximum for the integral in PI control (percentage of the power) */
#define DRIVEC_I_MAX 0.2f

/**
 * The constant for converting clicks to mm.
 *
 * Basic logic:
 * 512 clicks and 50:1 transmission
 * 50*512 = one revolution
 * 360/512 = 0.703125 degrees (on the wheel) is one click
 *
 * 512/(2*pi*wheel_diameter_mm/2) =
 *      = 512/(pi*wheel_diameter_mm) = click per mm
 *
 * For wheels with larger diameter:
 * 512/(2*pi*34.5/2) = 4.723 click per mm
 * clicks/4.723 = distance in mm or (clicks*1000)/4723 ~= distance in mm
 * (floating point math should be avoided - see also CLICK_MULTIPLIER)
 *
 * For wheels with smaller diameter:
 * 512/(2*pi*25/2) = 6.519 click per mm
 * clicks/6.519 ~= distance in mm
 *
 * NOTE: For some reason the calulcation does not produce the correct constant.
 *       After some measurments and excel calculations the right CLICK_CONST is
 *       7744.
 */
#define DRIVEC_CLICK_CONST 7744

/* The multiplier needed to avoid floating point math */
#define DRIVEC_CLICK_MULTIPLIER 1000

/* PUBLIC PROTOTYPES --------------------------------------------------------*/
void drive_control_init();
void drive_control_reset();

uint32_t get_left_abs_enc();
uint32_t get_right_abs_enc();

uint32_t get_left_abs_distance_mm();
uint32_t get_right_abs_distance_mm();

int32_t get_left_distance_mm();
int32_t get_right_distance_mm();

void drive(int16_t pwr_left, int16_t pwr_right);
uint8_t drive_mm(int16_t distance_mm, int16_t pwr);
uint8_t turn_deg(int32_t deg, int16_t pwr);

/* For debug */
extern int16_t error;
extern int16_t debug_pwr_right, debug_pwr_left;

#endif

