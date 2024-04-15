/**
 * Driving control for drone/bot swarm.
 */

#include "drive_control.h"

/* PRIVATE PROTOTYPES -------------------------------------------------------*/
void pwr_limit(int16_t *pwr);
void fpwr_limit(float *pwr);

/* PRIVATE GLOBALS ----------------------------------------------------------*/
/* PID control variables */
int16_t last_error;
int32_t error_integral;

/* FUNCTIONS ----------------------------------------------------------------*/
/**
 * Get absolute value of left encoder. Using uint32_t as it ensures that we 
 * will never run out of calculation space. 
 */
uint32_t get_left_abs_enc()
{
    return (uint32_t) (abs(get_left_enc()));
}

/**
 * Get absolute value of right encoder. Using uint32_t as it ensures that we 
 * will never run out of calculation space.
 */
uint32_t get_right_abs_enc()
{
    return (uint32_t) (abs(get_right_enc()));
}

/**
 * Get left wheel absolute driven distance in mm. To see the basic logic behind
 * conversion to mm, see get_right_abs_distance_mm()
 *
 * Returns: uint32_t, Absolute distance in mm
 */
uint32_t get_left_abs_distance_mm()
{
    uint32_t left_enc = get_left_abs_enc()*DRIVEC_CLICK_MULTIPLIER;
    return (uint32_t) (roundf(left_enc / DRIVEC_CLICK_CONST));
}


/**
 * Get right wheel absolute driven distance in mm. To see the basic logic
 * behind conversion to mm, see get_right_abs_distance_mm()
 *
 * Returns: uint32_t, Absolute distance in mm
 */
uint32_t get_right_abs_distance_mm()
{
    uint32_t right_enc = get_right_abs_enc()*DRIVEC_CLICK_MULTIPLIER;
    return (uint32_t) (roundf(right_enc / DRIVEC_CLICK_CONST));
}

/**
 * Get right wheel driven distance in mm. To see the basic logic behind
 * conversion to mm, see the DRIVEC_CLICK_CONST in the cmd_control.h file
 *
 * Returns: int32_t, Distance in mm. If the value is negative, then it means
 *          the wheel has been going backwards. If the value is positive, then
 *          the wheel has been going forward.
 */
int32_t get_left_distance_mm()
{
    /*
     * It is necessary to multiply get_right_enc by -1 to get the direction
     * right.
     */
    int32_t right_enc = (int32_t) (-get_left_enc());
    right_enc *= DRIVEC_CLICK_MULTIPLIER;
    return (int32_t) (roundf(right_enc / DRIVEC_CLICK_CONST));
}

/**
 * Get right wheel driven distance in mm. To see the basic logic behind
 * conversion to mm, see the DRIVEC_CLICK_CONST in the cmd_control.h file
 *
 * Returns: int32_t, Distance in mm. If the value is negative, then it means
 *          the wheel has been going backwards. If the value is positive, then
 *          the wheel has been going forward.
 */
int32_t get_right_distance_mm()
{
    /*
     * It is necessary to multiply get_right_enc by -1 to get the direction
     * right.
     */
    int32_t right_enc = (int32_t) (-get_right_enc());
    right_enc *= DRIVEC_CLICK_MULTIPLIER;
    return (int32_t) (roundf(right_enc / DRIVEC_CLICK_CONST));
}

/**
 * Resets drive control variables. Needed when a robot starts to execute a new
 * command.
 */
void drive_control_reset()
{
    motor_set(0, 0);
    last_error = 0.0f;
    error_integral = 0;
    left_enc_reset();
    right_enc_reset();
}

/**
 * Initialize drive control and its needed components.
 */
void drive_control_init()
{
    /* Set up motors */
    motor_init();
    /* Set up encoders - they read how many clicks has the motor done */
    quadrature_init();
    /* Reset all drive control variables */
    drive_control_reset();
}

/**
 * Limit the power to DRIVEC_MAX_PWR (see drive_control.h) if the power exceeds
 * the DRIVEC_MAX_PWR.
 *
 * Parameters: pwr - int16_t*, Pointer to the value that is going to be
 *                   throttled
 */
void pwr_limit(int16_t *pwr)
{
    if(pwr == NULL) return;

    if(*pwr > DRIVEC_MAX_PWR){
        *pwr = DRIVEC_MAX_PWR;
    }else if(*pwr < -DRIVEC_MAX_PWR){
        *pwr = -DRIVEC_MAX_PWR;
    }
}

/**
 * Limit the power to DRIVEC_MAX_PWR (see drive_control.h) if the power exceeds
 * the DRIVEC_MAX_PWR.
 *
 * Parameters: pwr - float*, Pointer to the value that is going to be
 *                   throttled
 */
void fpwr_limit(float *pwr)
{
    if(pwr == NULL) return;
    
    if(*pwr > DRIVEC_MAX_PWR){
        *pwr = DRIVEC_MAX_PWR;
    }else if(*pwr < -DRIVEC_MAX_PWR){
        *pwr = -DRIVEC_MAX_PWR;
    }
}

/**
 * Calculate PID control powers
 *
 * Parameters:
 *      c_pwr - uint16_t, Constant power the PID control power calculation is
 *              based on
 *      pwr_left - float*, Pointer to variable where the PID controlled
 *                 left power is going to be saved
 *      pwr_right - float*, Pointer to variable where the PID controlled
 *                  right power is going to be saved
 *
 * Returns: float, the PID controlled lead power (u)
 */
/* For debug */
int16_t error;
int16_t debug_pwr_right, debug_pwr_left;
float pid_control(uint16_t c_pwr, float *fpwr_left, float *fpwr_right)
{
    error = (int16_t) (get_left_abs_enc() - get_right_abs_enc());
    
    /* Choose either PD or PI control */ 
    /**
     * PD (Proportional Derivative) control
     */
    float u = DRIVEC_P_CONST*c_pwr*error +
              DRIVEC_D_CONST*c_pwr*(last_error - error);
    
    /**
     * PI (Proportinal Integral) control
     */
    /*
    if((error_integral > 0 && error < 0) || (error_integral < 0 && error > 0)){
        error_integral = 0;
    }else if(error_integral < (int32_t) roundf(DRIVEC_I_MAX*c_pwr)){
        error_integral += error;
    }
    float u = DRIVEC_P_CONST*c_pwr*error + DRIVEC_I_CONST*c_pwr*error_integral;
    */
    
    /* Make the adjustments */ 
    *fpwr_left = roundf(c_pwr - u);
    *fpwr_right = roundf(c_pwr + u);
    
    /* Limiting the PID powers to DRIVEC_MAX_PWR */
    fpwr_limit(fpwr_left);
    fpwr_limit(fpwr_right);

    debug_pwr_left = (int16_t) *fpwr_left;
    debug_pwr_right = (int16_t) *fpwr_right;
    
    last_error = error;
    
    return u;    
}

/**
 * Drive by setting the motors. This function does not check if any distance
 * has been driven, it just sets the motor powers to pwr_left and pwr_right 
 * with PID control for straight line driving and other helpful features.
 *
 * Parameters:
 *      pwr_left - int16_t, Left motor power
 *      pwr_right - int16_t, Right motor power
 *
 * NOTE: pwr_left and pwr_right are throttled by the constant DRIVEC_MAX_PWR
 *       (see drive_control.h)
 */
void drive(int16_t pwr_left, int16_t pwr_right)
{
    /* Limiting the base powers to DRIVEC_MAX_PWR (see drive_control.h) */
    pwr_limit(&pwr_left);
    pwr_limit(&pwr_right);
    
    if((pwr_left == pwr_right) && pwr_right != 0){
        int16_t pwr = abs(pwr_right);
        int16_t direction = (pwr_right > 0) ? 1 : -1;

        /**
         * PID (Proportional Integral Derivative) control
         */
        float fpwr_left = 0.0f;
        float fpwr_right = 0.0f;
        float u = pid_control(pwr, &fpwr_left, &fpwr_right);
        
        /* Let's drive */
        if(u > 0){
            /* Turn left */
            motor_set((int16_t) (direction * fpwr_left), direction * pwr);
        }else if(u < 0){
            /* Turn right */ 
            motor_set(direction * pwr, (int16_t) (direction * fpwr_right));
        }else{
            /* Drive straight */
            motor_set(direction * pwr, direction * pwr);
        }

    }else{
        motor_set(pwr_left, pwr_right);
    }
}


/**
 * Drive backwards or forward with PID control. PID control is necessary for
 * driving straight (the encoders are not super accurate)
 *
 * Parameters:
 *      distance_mm - int16_t, The distance to be driven. If the
 *                    parameter is positive then drive forward. If
 *                    negative then drive backwards
 *      pwr - int16_t, The power that goes to the motors or simply put: motor
 *            speed. Should be positive. NOTE: The value is throttled by the
 *            constant DRIVEC_MAX_PWR (see drive_control.h)
 * 
 * Returns: 0 or 1 (uint8_t) - 0 indicating that task is not completed (aka the
 *          given distance is not yet driven); 1 indicating that he task is
 *          completed (the given distance has been driven)
 */
uint8_t drive_mm(int16_t distance_mm, int16_t pwr)
{
    if(distance_mm == 0 || pwr == 0){
        motor_set(0, 0);
        return 1; 
    }

    pwr_limit(&pwr);
    pwr = abs(pwr);

    uint16_t abs_distance_mm = abs(distance_mm);
    int16_t direction = (distance_mm > 0) ? 1 : -1;
    
    /**
     * PID (Proportional Integral Derivative) control
     */
    float fpwr_left = 0.0f;
    float fpwr_right = 0.0f;
    float u = pid_control(pwr, &fpwr_left, &fpwr_right);
    
    /* Let's drive */
    if(get_left_abs_distance_mm() >= abs_distance_mm || 
            get_right_abs_distance_mm() >= abs_distance_mm){
        motor_set(0, 0);
        return 1;
    }else{
        if(u > 0){
            /* Turn left */
            motor_set((int16_t) (direction*fpwr_left), direction*pwr);
        }else if(u < 0){
            /* Turn right */ 
            motor_set(direction*pwr, (int16_t) (direction*fpwr_right));
        }else{
            /* Drive straight */
            motor_set(direction*pwr, direction*pwr);
        }
    }

    return 0;
}

/**
 * Turn the robot by deg degrees.
 *
 * Parameters:
 *      deg - int16_t, How much should the robot turn in degrees
 *            (positive values turns robot in clockwise, negative values
 *            in counter clockwise)
 *      pwr - int16_t, The power that goes to the motors or simply put: motor
 *            speed. Should be positive. NOTE: The value is throttled by the
 *            constant DRIVEC_MAX_PWR (see drive_control.h)
 *
 * Basic logic (might be wrong):
 * Robot's radius from robt's center to wheel center is 44.65 mm. Thus rotating
 * on its z axis it would make a circle with a diameter 89.3 mm - the circle
 * perimeter is 89.3*PI mm. So that means that the robot needs to drive
 * 89.3*PI~=280.54 mm to make a full cirle. Thus:
 *   89.65*PI mm -> 360 deg
 *   x           -> 1   deg
 *   x = (89.65*PI) / 360 ~= 0.779 mm
 * So the necessary distance needs that robot needs to drive to turn y degrees
 * would be y*0.779 but to avoid floating point math we can also calculate the
 * necessary distance like this:
 *   (z * 779) / 1000 ~= distance mm
 * 
 * Returns: 0 or 1 (uint8_t) - 0 indicating that task is not completed (aka the
 *                             given distance is not yet driven); 1 indicating
 *                             that the task is completed (the given distance
 *                             has been driven)
 */
uint8_t turn_deg(int32_t deg, int16_t pwr)
{
    if(deg == 0 || pwr == 0){
        motor_set(0, 0);
        return 1;
    }

    pwr_limit(&pwr);
    pwr = abs(pwr);

    /**
     * PID (Proportional Integral Derivative) control specially for turning
     */
    /*error = (int16_t) (get_left_abs_enc() - get_right_abs_enc());
    float u = DRIVEC_P_CONST*pwr*error +
              DRIVEC_D_CONST*pwr*(last_error - error);*/
    
    /* Make the adjustments */ 
    /*float fpwr_left;

    if(deg < 0){
        fpwr_left = roundf(-pwr+u);
    }else{
        fpwr_left = roundf(pwr-u);
    }*/
    
    /* Limiting the PID powers to DRIVEC_MAX_PWR */
    /*fpwr_limit(&fpwr_left);

    debug_pwr_left = (int16_t) fpwr_left;*/

    uint32_t circle_distance_mm = labs(deg);
    circle_distance_mm = (uint32_t) roundf(circle_distance_mm*779/1000);

    if(get_left_abs_distance_mm() >= circle_distance_mm || 
            get_right_abs_distance_mm() >= circle_distance_mm){
        motor_set(0, 0); 
        return 1;
    }else{
        if(deg < 0){
            motor_set(-pwr, pwr);
            /* motor_set((int16_t) fpwr_left, pwr); */
        }else{
            motor_set(pwr, -pwr);
            /* motor_set((int16_t) fpwr_left, -pwr); */
        }
    }

    /* last_error = error; */
    return 0;
}
