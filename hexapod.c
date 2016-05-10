
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "hexapod.h"
#include "pwm_driver.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#define MAX_SPEED 100 //movements per cycle (1000)
#define MIN_SPEED 10
#define MAX_FRAMES 100

#define LEG_DEFAULT_VALUE 1500
#define LEG_HEIGHT_OFFSET  100

#define TOP_LEG_DEFAULT_VALUE 1500
#define TOP_LEG_MAX_VALUE 1700
#define TOP_LEG_MIN_VALUE 1300

#define MID_LEG_DEFAULT_VALUE (1500 + LEG_HEIGHT_OFFSET)
#define MID_LEG_MAX_VALUE (1700 + LEG_HEIGHT_OFFSET)
#define MID_LEG_MIN_VALUE (1500 + LEG_HEIGHT_OFFSET)

#define BOT_LEG_DEFAULT_VALUE (1200 + LEG_HEIGHT_OFFSET)
#define BOT_LEG_MAX_VALUE (1700 + LEG_HEIGHT_OFFSET)
#define BOT_LEG_MIN_VALUE (1200 + LEG_HEIGHT_OFFSET)

#define DEF_POSE_LEFT_LEG (hexapod_leg_t){3000 - TOP_LEG_DEFAULT_VALUE, 3000 - MID_LEG_DEFAULT_VALUE, 3000 - BOT_LEG_DEFAULT_VALUE}
#define DEF_POSE_RIGHT_LEG (hexapod_leg_t){TOP_LEG_DEFAULT_VALUE, MID_LEG_DEFAULT_VALUE, BOT_LEG_DEFAULT_VALUE}

#define NUMBER_OF_LEGS  6

#define LEG_LIFT_HEIGHT -300

//pin definitions
#define FRONT_LEFT_BOT      5
#define FRONT_LEFT_MID      6
#define FRONT_LEFT_TOP      7

#define FRONT_RIGHT_BOT      31
#define FRONT_RIGHT_MID     30
#define FRONT_RIGHT_TOP     29
    
#define MID_LEFT_BOT        8
#define MID_LEFT_MID        12
#define MID_LEFT_TOP        11

#define MID_RIGHT_BOT       4
#define MID_RIGHT_MID       13
#define MID_RIGHT_TOP       28

#define BACK_LEFT_BOT       18
#define BACK_LEFT_MID       16
#define BACK_LEFT_TOP       17

#define BACK_RIGHT_BOT      3
#define BACK_RIGHT_MID      15
#define BACK_RIGHT_TOP      14

/*
For writability legs are numbered 0-5 from front left to bottom right

0 ---|  |--- 1
2 ---|  |--- 3
4 ---|  |--- 5

Group 1 is leg 0, 3 and 4
Group 2 is leg 1, 2 and 5 

*/

////////////////

typedef enum
{
    IDLE,
    TRANSITION,
    NEW_ENDPOINTS,
} hexapod_state_t;

typedef struct
{
    hexapod_leg_t current_point;
    hexapod_leg_t endpoint_out;
    hexapod_leg_t endpoint_in;
    hexapod_leg_t new_endpoint_out;
    hexapod_leg_t new_endpoint_in;
    hexapod_state_t state;
    int32_t count;
    int32_t frames;
    bool direction;
    bool is_moving;
    bool stop;
    bool is_left_leg;
} hexapod_leg_data_t;

hexapod_leg_data_t leg_data[6];

static int32_t new_frames = 0;
static int32_t transition_frames = 0;

/*
static int32_t m_frames = 0;
static int32_t m_new_frames = 0;
static int32_t m_count[6] = {0, 0, 0, 0, 0, 0};
static bool direction[6] = {false, true, true, false, false, true};
static bool new_direction[6];
static hexapod_leg_t m_current_point[6];
static uint32_t m_endpoint_reached = 6;
volatile bool is_moving = false;
volatile bool stop = false;

hexapod_state_t m_state = IDLE;
//static bool new_endpoints = false;
//static bool transition = false;

static hexapod_leg_t m_endpoint_out_left;
static hexapod_leg_t m_endpoint_out_right;
static hexapod_leg_t m_endpoint_in_left;
static hexapod_leg_t m_endpoint_in_right;

static hexapod_leg_t m_new_endpoint_out_left;
static hexapod_leg_t m_new_endpoint_out_right;
static hexapod_leg_t m_new_endpoint_in_left;
static hexapod_leg_t m_new_endpoint_in_right;
*/


void hexapod_init()
{
    hexapod_leg_t leg_pins[6];
    
    leg_pins[0] = (hexapod_leg_t){FRONT_LEFT_TOP, FRONT_LEFT_MID, FRONT_LEFT_BOT};
    leg_pins[1] = (hexapod_leg_t){FRONT_RIGHT_TOP, FRONT_RIGHT_MID, FRONT_RIGHT_BOT};
    leg_pins[2] = (hexapod_leg_t){MID_LEFT_TOP, MID_LEFT_MID, MID_LEFT_BOT};
    leg_pins[3] = (hexapod_leg_t){MID_RIGHT_TOP, MID_RIGHT_MID, MID_RIGHT_BOT};
    leg_pins[4] = (hexapod_leg_t){BACK_LEFT_TOP, BACK_LEFT_MID, BACK_LEFT_BOT};
    leg_pins[5] = (hexapod_leg_t){BACK_RIGHT_TOP, BACK_RIGHT_MID, BACK_RIGHT_BOT};
    
    
    for(int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        if( i % 2 == 0)
        {
            leg_data[i].current_point = DEF_POSE_LEFT_LEG;
            leg_data[i].endpoint_out = DEF_POSE_LEFT_LEG;
            leg_data[i].endpoint_in = DEF_POSE_LEFT_LEG;
            leg_data[i].is_left_leg = true;
        }
        else
        {
            leg_data[i].current_point = DEF_POSE_RIGHT_LEG;
            leg_data[i].endpoint_out = DEF_POSE_RIGHT_LEG;
            leg_data[i].endpoint_in = DEF_POSE_RIGHT_LEG;
            leg_data[i].is_left_leg = false;
        }
        
        //check which group
        if( (i == 0) || (i == 3) || (i == 4) )
        {
            leg_data[i].direction = false;
        }
        else
        {
            leg_data[i].direction = true;
        }
        
        leg_data[i].count = 0;
        leg_data[i].frames = 0;
        leg_data[i].is_moving = false;
        leg_data[i].state = IDLE;
        leg_data[i].stop = false;
    }

    hexapod_servo_pwm_init(leg_pins);
}

void hexapod_shutdown()
{
    //stop the pwm
    hexapod_servo_pwm_stop();
}


void calculate_endpoints(uint32_t dir_x, uint32_t dir_y, uint32_t rot_x)
{
    
}

void calc_next_points_leg(uint32_t leg_nr)
{
    
    int32_t leg_lift_height = 0;
    int32_t point_lift_height = 0;
    hexapod_leg_t endpoint_1;
    hexapod_leg_t endpoint_2;
    hexapod_leg_data_t *p_leg_data;
    
    p_leg_data = &leg_data[leg_nr];
    
    //If frames is equal to 0 it means that we are standing still
    if(p_leg_data->frames == 0)
    {
        return;
    }
    
    if(p_leg_data->direction == true)
    {
        endpoint_1 = p_leg_data->endpoint_in;
        if(p_leg_data->state == NEW_ENDPOINTS)
        {
            endpoint_2 = p_leg_data->new_endpoint_out;
        }
        else
        {
            endpoint_2 = p_leg_data->endpoint_out;
        }
        
        //lift leg if left leg
        if(p_leg_data->is_left_leg)
        {
            leg_lift_height = -LEG_LIFT_HEIGHT;
        }
        else
        {
            leg_lift_height = LEG_LIFT_HEIGHT;
        }
    }
    else
    {
        endpoint_1 = p_leg_data->endpoint_out;
        if(p_leg_data->state == NEW_ENDPOINTS)
        {
            endpoint_2 = p_leg_data->new_endpoint_in;
        }
        else
        {
            endpoint_2 = p_leg_data->endpoint_in;
        }
    }
    
    //going from endpoint_1 to endpoint_2
            
    //top leg
    p_leg_data->current_point.leg_top = endpoint_1.leg_top + (endpoint_2.leg_top - endpoint_1.leg_top) * p_leg_data->count / p_leg_data->frames;
    
    //mid leg
    //calculate lift height
    if(leg_lift_height != 0)
    {
        if(p_leg_data->count < p_leg_data->frames/2)
        {
            point_lift_height = 2 * p_leg_data->count * leg_lift_height / p_leg_data->frames;
        }
        else
        {
            point_lift_height = leg_lift_height - (2 * p_leg_data->count - p_leg_data->frames) * leg_lift_height / p_leg_data->frames;
        }
    }
    
    p_leg_data->current_point.leg_mid = endpoint_1.leg_mid + (endpoint_2.leg_mid - endpoint_1.leg_mid) * p_leg_data->count / p_leg_data->frames + point_lift_height;
    
    //bottom leg
    p_leg_data->current_point.leg_bot = endpoint_1.leg_bot + (endpoint_2.leg_bot - endpoint_1.leg_bot) * p_leg_data->count / p_leg_data->frames;
    
    //increase count value
    p_leg_data->count++;
    
}

int32_t hexapod_get_next_seq_value(uint32_t leg_nr, hexapod_leg_t *leg)
{    

    hexapod_leg_data_t *p_leg_data;
    
    p_leg_data = &leg_data[leg_nr];
    *leg = p_leg_data->current_point;
    
    if(leg_nr == 1)
    {
        //NRF_LOG_PRINTF("%d, %d: %d\t%d\t%d\n", p_leg_data->count, p_leg_data->frames, leg->leg_top, leg->leg_mid, leg->leg_bot);
    }
    
    if(p_leg_data->is_moving == false)
    {
        return 0;
    }
    
    calc_next_points_leg(leg_nr);
    
    if(p_leg_data->count >= p_leg_data->frames)
    {
        //we are at a enpoint
        
        //change direction
        p_leg_data->direction = !p_leg_data->direction;
        
        switch(p_leg_data->state)
        {
            case IDLE:
                break;
            case TRANSITION:
                p_leg_data->state = NEW_ENDPOINTS;
                p_leg_data->frames = transition_frames;
                break;
            case NEW_ENDPOINTS:

                p_leg_data->endpoint_in = p_leg_data->new_endpoint_in;
                p_leg_data->endpoint_out = p_leg_data->new_endpoint_out;
            
                if(p_leg_data->stop == true)
                {
                    p_leg_data->is_moving = false;
                }
                
                p_leg_data->state = IDLE;
                p_leg_data->frames = new_frames;
                break;
            default:
                //ERROR!
                break;
        }

        p_leg_data->count = 0;
    }
    return 0;
}

void print_trajectory(uint32_t leg_nr)
{
    hexapod_leg_t leg;
    //NRF_LOG_PRINTF("endpoint_left_out: %d\t%d\t%d\n", m_count[leg_nr],
      //  leg.leg_top, leg.leg_mid, leg.leg_bot);
    NRF_LOG_PRINTF("m_frames: %d\n", leg_data[leg_nr].frames);
    NRF_LOG_PRINTF("trajectory leg number: %d\nT\t\tM\t\tB\n", leg_nr);
    for(int i = 0; i < leg_data[leg_nr].frames; i++)
    {
        hexapod_get_next_seq_value(leg_nr, &leg);
        NRF_LOG_PRINTF("%d: %d\t%d\t%d\n", leg_data[leg_nr].count,
        leg.leg_top, leg.leg_mid, leg.leg_bot);
        nrf_delay_ms(10);
    }
}

uint32_t max(uint32_t value1, uint32_t value2, uint32_t value3)
{
    uint32_t max_value = value1;
    if(max_value < value2)
    {
        max_value = value2;
    }
    if(max_value < value3)
    {
        max_value = value3;
    }
    return max_value;
}

int32_t abs_int32(int32_t value)
{
    if(value < 0)
    {
        return -value;
    }
    return value;
}

void calculate_frames(hexapod_leg_t new_endpoint_out_right, hexapod_leg_t new_endpoint_in_right, int32_t *p_transition_frames, int32_t *p_new_frames)
{
    

        
}


void hexapod_move_forward(bool direction, uint8_t speed)
{
    
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_1_right = {TOP_LEG_MAX_VALUE, MID_LEG_DEFAULT_VALUE, BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_2_right = {TOP_LEG_MIN_VALUE, MID_LEG_DEFAULT_VALUE, BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_1_left = {3000 - TOP_LEG_MAX_VALUE, 3000 - MID_LEG_DEFAULT_VALUE, 3000 - BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_2_left = {3000 - TOP_LEG_MIN_VALUE, 3000 - MID_LEG_DEFAULT_VALUE, 3000 - BOT_LEG_DEFAULT_VALUE};
    
    //TODO: NEED TO CALCULATE FRAMES
    transition_frames = 400/speed;
    new_frames = 400/speed;
    
    for( int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        p_leg_data = &leg_data[i];
        
        if(p_leg_data->is_left_leg)
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_left;
                p_leg_data->new_endpoint_out = endpoint_1_left;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_left;
                p_leg_data->new_endpoint_out = endpoint_2_left;
            }
        }
        else
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_right;
                p_leg_data->new_endpoint_out = endpoint_1_right;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_right;
                p_leg_data->new_endpoint_out = endpoint_2_right;
            }
        }
        
        p_leg_data->stop = false;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }

    /*
    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
    */
    
}


void hexapod_move_sideways(bool direction, uint8_t speed)
{
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_1_right = {TOP_LEG_DEFAULT_VALUE, MID_LEG_MAX_VALUE, BOT_LEG_MAX_VALUE};
    hexapod_leg_t endpoint_2_right = {TOP_LEG_DEFAULT_VALUE, MID_LEG_MIN_VALUE, BOT_LEG_MIN_VALUE};
    hexapod_leg_t endpoint_1_left = {3000 - TOP_LEG_DEFAULT_VALUE, 3000 - MID_LEG_MIN_VALUE, 3000 - BOT_LEG_MIN_VALUE};
    hexapod_leg_t endpoint_2_left = {3000 - TOP_LEG_DEFAULT_VALUE, 3000 - MID_LEG_MAX_VALUE, 3000 - BOT_LEG_MAX_VALUE};
    
    //TODO: NEED TO CALCULATE FRAMES
    transition_frames = 400/speed;
    new_frames = 500/speed;
    
    for( int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        p_leg_data = &leg_data[i];
        
        if(p_leg_data->is_left_leg)
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_left;
                p_leg_data->new_endpoint_out = endpoint_1_left;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_left;
                p_leg_data->new_endpoint_out = endpoint_2_left;
            }
        }
        else
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_right;
                p_leg_data->new_endpoint_out = endpoint_1_right;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_right;
                p_leg_data->new_endpoint_out = endpoint_2_right;
            }
        }
        
        p_leg_data->stop = false;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }
/*
    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
    */
}

void hexapod_move_diagonal(bool direction, uint8_t speed)
{
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_1_right;
    hexapod_leg_t endpoint_2_right;
    hexapod_leg_t endpoint_1_left;
    hexapod_leg_t endpoint_2_left;
    
    if(direction)
    {
        endpoint_1_right = (hexapod_leg_t){TOP_LEG_MAX_VALUE, MID_LEG_MAX_VALUE, BOT_LEG_MAX_VALUE};
        endpoint_2_right = (hexapod_leg_t){TOP_LEG_MIN_VALUE, MID_LEG_MIN_VALUE, BOT_LEG_MIN_VALUE};
        
        endpoint_1_left = (hexapod_leg_t){3000 - TOP_LEG_MAX_VALUE, 3000 - MID_LEG_MIN_VALUE, 3000 - BOT_LEG_MIN_VALUE};
        endpoint_2_left = (hexapod_leg_t){3000 - TOP_LEG_MIN_VALUE, 3000 - MID_LEG_MAX_VALUE, 3000 - BOT_LEG_MAX_VALUE};
    }
    else
    {
        endpoint_1_right = (hexapod_leg_t){1700, 1500, 1200};
        endpoint_2_right = (hexapod_leg_t){1300, 1700, 1700};
        
        endpoint_1_left = (hexapod_leg_t){1300, 1300, 1300};
        endpoint_2_left = (hexapod_leg_t){1700, 1500, 1800};
    }
    
    //TODO: NEED TO CALCULATE FRAMES
    transition_frames = 400/speed;
    new_frames = 500/speed;
    
    for( int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        p_leg_data = &leg_data[i];
        
        if(p_leg_data->is_left_leg)
        {
            p_leg_data->new_endpoint_in = endpoint_2_left;
            p_leg_data->new_endpoint_out = endpoint_1_left;
        }
        else
        {
            p_leg_data->new_endpoint_in = endpoint_2_right;
            p_leg_data->new_endpoint_out = endpoint_1_right;
        }
        
        p_leg_data->stop = false;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }
}

void hexapod_turn(bool direction, uint8_t speed)
{
    
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_2_right = {TOP_LEG_MAX_VALUE, MID_LEG_DEFAULT_VALUE, BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_1_right = {TOP_LEG_MIN_VALUE, MID_LEG_DEFAULT_VALUE, BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_1_left = {3000 - TOP_LEG_MAX_VALUE, 3000 - MID_LEG_DEFAULT_VALUE, 3000 - BOT_LEG_DEFAULT_VALUE};
    hexapod_leg_t endpoint_2_left = {3000 - TOP_LEG_MIN_VALUE, 3000 - MID_LEG_DEFAULT_VALUE, 3000 - BOT_LEG_DEFAULT_VALUE};
    
    //TODO: NEED TO CALCULATE FRAMES
    transition_frames = 400/speed;
    new_frames = 400/speed;
    
    for( int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        p_leg_data = &leg_data[i];
        
        if(p_leg_data->is_left_leg)
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_left;
                p_leg_data->new_endpoint_out = endpoint_1_left;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_left;
                p_leg_data->new_endpoint_out = endpoint_2_left;
            }
        }
        else
        {
            if(direction)
            {
                p_leg_data->new_endpoint_in = endpoint_2_right;
                p_leg_data->new_endpoint_out = endpoint_1_right;
            }
            else
            {
                p_leg_data->new_endpoint_in = endpoint_1_right;
                p_leg_data->new_endpoint_out = endpoint_2_right;
            }
        }
        
        p_leg_data->stop = false;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }

}

void hexapod_stop(uint8_t speed)
{
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_front_right = DEF_POSE_RIGHT_LEG;
    hexapod_leg_t endpoint_back_right = DEF_POSE_RIGHT_LEG;
    hexapod_leg_t endpoint_front_left = DEF_POSE_LEFT_LEG;
    hexapod_leg_t endpoint_back_left = DEF_POSE_LEFT_LEG;
    
    //TODO: NEED TO CALCULATE FRAMES
    transition_frames = (BOT_LEG_MAX_VALUE - BOT_LEG_DEFAULT_VALUE)/speed;
    new_frames = 0;
    
    for( int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        p_leg_data = &leg_data[i];
        
        if(p_leg_data->is_left_leg)
        {
            p_leg_data->new_endpoint_in = endpoint_back_left;
            p_leg_data->new_endpoint_out = endpoint_front_left;
        }
        else
        {
            p_leg_data->new_endpoint_in = endpoint_back_right;
            p_leg_data->new_endpoint_out = endpoint_front_right;
        }
        
        p_leg_data->stop = true;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }
/*
    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
    */
}

void hexapod_test_sequence(uint8_t speed)
{
    NRF_LOG("\nFORWARD\n\n");
    hexapod_move_forward(true, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nBACKWARD\n\n");
    hexapod_move_forward(false, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nRIGHT\n\n");
    hexapod_move_sideways(true, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nLEFT\n\n");
    hexapod_move_sideways(false, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nDIAGONAL RIGHT FORWARD\n\n");
    hexapod_move_diagonal(true, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nDIAGONAL LEFT FORWARD\n\n");
    hexapod_move_diagonal(false, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nTURN CW\n\n");
    hexapod_turn(true, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nTURN CCW\n\n");
    hexapod_turn(false, speed);
    
    nrf_delay_ms(4000);
    
    NRF_LOG("\nSTOP\n\n");
    hexapod_stop(speed);
    
    nrf_delay_ms(4000);
}
