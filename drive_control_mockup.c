#include "drivers/motor.h"

uint32_t mock_distance;
uint32_t mock_delay = 20;
uint32_t mock_last_time_delay = 0;

uint32_t mock_get_distance()
{
    return mock_distance;
}

void mock_drive_control_init()
{
    mock_distance = 0;
}

void mock_drive_control_reset()
{
    mock_distance = 0;
}

int8_t mock_drive_mm(uint32_t distance_mm, uint32_t millis)
{
    if(mock_distance == distance_mm){
        return 1; /* Distance has been driven  */
    }
    
    if((millis - mock_last_time_delay) > mock_delay){ 
        mock_last_time_delay = millis; 
        mock_distance++;
    }

    return 0;
}



int8_t mock_turn_deg(uint32_t deg, uint32_t millis)
{
    if(mock_distance == deg){
        return 1; /* Distance has been driven  */
    }
    
    if((millis - mock_last_time_delay) > mock_delay){ 
        mock_last_time_delay = millis; 
        mock_distance++;
    }

    return 0;
}
