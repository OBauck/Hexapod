
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

#define DEF_POSE_LEFT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1800}
#define DEF_POSE_RIGHT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1200}

#define DEF_POSE_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1200}

#define NUMBER_OF_LEGS  6

//#define LEG_LIFT_HEIGHT_LEFT 50
//#define LEG_LIFT_HEIGHT_RIGHT -50

#define LEG_LIFT_HEIGHT -200

#define MAX_FORWARD_VALUE    1750
#define MIN_FORWARD_VALUE   1250
#define MAX_SIDEWAYS_VALUE   1750
#define MIN_SIDEWAYS_VALUE   1250

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

typedef enum
{
    FORWARD,
    FORWARD_RIGHT,
    FORWARD_LEFT,
    RIGHT,
    LEFT,
    BACKWARD,
    BACKWARD_RIGHT,
    BACKWARD_LEFT,
    CW_TURN,
    CCW_TURN,
    STOP,
} hexapod_dir_t;

/*
For writability legs are numbered 0-5 from front left to bottom right

0 ---|  |--- 1
2 ---|  |--- 3
4 ---|  |--- 5

Group 1 is leg 0, 3 and 4
Group 2 is leg 1, 2 and 5 

*/
/*
static uint32_t m_speed = MIN_SPEED;

//static hexapod_leg_t m_leg_seq[6][MAX_FRAMES];

static hexapod_leg_t m_leg_seq_right[MAX_FRAMES*2];
static hexapod_leg_t m_leg_seq_left[MAX_FRAMES*2];

static uint32_t m_seq_count[6] = {0, 0, 0, 0, 0, 0};
static bool m_seq_updated[6] = {false, false, false, false, false, false};

static bool m_lift_group1 = false;
static bool m_repeat_seq = true;
static bool m_change_direction = false;
static hexapod_dir_t m_direction = STOP;
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
    
    /*
    m_leg_seq_right[0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq_left[0] = DEF_POSE_LEFT_LEG;

    for(int i = 0; i < 6; i++)
    {
        m_seq_count[i] = 0;
        m_seq_updated[i] = true;
    }
    m_frames = 1;
    */
    
    /*
    m_current_point[0] = DEF_POSE_LEFT_LEG;
    m_current_point[1] = DEF_POSE_RIGHT_LEG;
    m_current_point[2] = DEF_POSE_LEFT_LEG;
    m_current_point[3] = DEF_POSE_RIGHT_LEG;
    m_current_point[4] = DEF_POSE_LEFT_LEG;
    m_current_point[5] = DEF_POSE_RIGHT_LEG;
    */
    
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
    /*
    m_endpoint_out_left = DEF_POSE_LEFT_LEG;
    m_endpoint_out_right = DEF_POSE_RIGHT_LEG;
    m_endpoint_in_left = DEF_POSE_LEFT_LEG;
    m_endpoint_in_right = DEF_POSE_RIGHT_LEG;
    */
    hexapod_servo_pwm_init(leg_pins);
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
    
    //we can't divide by 0
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

void calc_next_points()
{
    //TODO: include flag to block updating PWM if values are not updated
    
    for(int i = 0; i < 6; i++)
    {
        calc_next_points_leg(i);
    }
}

int32_t hexapod_get_next_seq_value(uint32_t leg_nr, hexapod_leg_t *leg)
{    

    hexapod_leg_data_t *p_leg_data;
    
    p_leg_data = &leg_data[leg_nr];
    *leg = p_leg_data->current_point;
    
    if(leg_nr == 1)
    {
        NRF_LOG_PRINTF("%d, %d: %d\t%d\t%d\n", p_leg_data->count, p_leg_data->frames, leg->leg_top, leg->leg_mid, leg->leg_bot);
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

int32_t abs_int32(int32_t value)
{
    if(value < 0)
    {
        return -value;
    }
    return value;
}
/*
void calculate_frames()
{
    uint32_t max;
    
    switch(m_state)
        {
            case IDLE:
                break;
            case TRANSITION:
                max = abs_int32(m_endpoint_in_right.leg_top - m_new_endpoint_out_right.leg_top);
                break;
            case NEW_ENDPOINTS:
                break;
            default:
                //ERROR!
                break;
        }
        
}
*/

void hexapod_move_forward(uint8_t speed)
{
    
    //TODO: NEED TO CHECK IF LEG IS ALREADY IN TRANSITION
    //TODO: NEED TO CALCULATE FRAMES
    
    hexapod_leg_data_t *p_leg_data;
    
    hexapod_leg_t endpoint_front_right = {1700, 1500, 1200};
    hexapod_leg_t endpoint_back_right = {1300, 1500, 1200};
    hexapod_leg_t endpoint_front_left = {1300, 1500, 1800};
    hexapod_leg_t endpoint_back_left = {1700, 1500, 1800};
    
    transition_frames = 200/speed;
    new_frames = 400/speed;
    
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
        
        p_leg_data->stop = false;
        p_leg_data->is_moving = true;
        p_leg_data->state = TRANSITION;
    }
    
    /*
    if(is_moving == false)
    {
        m_new_frames = 200 / speed;
    }
    else
    {
        m_new_frames = 400 / speed;
    }
    
    m_new_endpoint_out_right.leg_top = 1700;
    m_new_endpoint_out_right.leg_mid = 1500;
    m_new_endpoint_out_right.leg_bot = 1200;
    
    m_new_endpoint_in_right.leg_top = 1300;
    m_new_endpoint_in_right.leg_mid = 1500;
    m_new_endpoint_in_right.leg_bot = 1200;
    
    m_new_endpoint_out_left.leg_top = 1300;
    m_new_endpoint_out_left.leg_mid = 1500;
    m_new_endpoint_out_left.leg_bot = 1800;
    
    m_new_endpoint_in_left.leg_top = 1700;
    m_new_endpoint_in_left.leg_mid = 1500;
    m_new_endpoint_in_left.leg_bot = 1800;

    m_state = TRANSITION;
    stop = false;
    is_moving = true;
    */

    /*
    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
    */
    
}
/*

void hexapod_move_right(uint8_t speed)
{
    //change this
    if(is_moving == false)
    {
        m_new_frames = 200 / speed;
    }
    else
    {
        m_new_frames = 500 / speed;
    }
    
    //CHANGE THIS!
    m_frames = 200 / speed;
    
    m_new_endpoint_out_right.leg_top = 1500;
    m_new_endpoint_out_right.leg_mid = 1700;
    m_new_endpoint_out_right.leg_bot = 1700;
    
    m_new_endpoint_in_right.leg_top = 1500;
    m_new_endpoint_in_right.leg_mid = 1500;
    m_new_endpoint_in_right.leg_bot = 1200;
    
    m_new_endpoint_out_left.leg_top = 1500;
    m_new_endpoint_out_left.leg_mid = 1300;
    m_new_endpoint_out_left.leg_bot = 1300;
    
    m_new_endpoint_in_left.leg_top = 1500;
    m_new_endpoint_in_left.leg_mid = 1500;
    m_new_endpoint_in_left.leg_bot = 1800;

    m_state = TRANSITION;
    stop = false;
    is_moving = true;

    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
}

void hexapod_stop(uint8_t speed)
{
    //change this

    m_new_frames = 200 / speed;
    
    m_new_endpoint_out_right = DEF_POSE_RIGHT_LEG;
    m_new_endpoint_in_right = DEF_POSE_RIGHT_LEG;
    
    m_new_endpoint_out_left = DEF_POSE_LEFT_LEG;
    m_new_endpoint_in_left = DEF_POSE_LEFT_LEG;

    stop = true;
    is_moving = true;
    m_state = TRANSITION;

    print_trajectory(1);
    print_trajectory(1);
    print_trajectory(1);
}
*/

///OLD CODE USING SEQUENCE:
/*
void change_direction(hexapod_dir_t direction)
{
    m_direction = direction;
    m_change_direction = true;
}

int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg)
{
    if(m_seq_updated[leg_number])
    {
        if(m_seq_count[leg_number] == m_frames)
        {
            //stop if sequence should not be repeated
            if(m_repeat_seq == false)
            {
                for(int i = 0; i < 6; i++)
                {
                    m_seq_updated[i] = false;
                }
                
                if(m_change_direction == true)
                {
                    
                    
                    m_change_direction = false;
                }
                
                return -1;
            }
            
            //TODO: do calculating of new trajectory in main with app_scheduler
            
            //check if new trajectory should be calculated
            if(m_change_direction == true)
            {
                 
            }
            else
            {
                m_seq_count[leg_number] = 0;
            }
        }
        if((leg_number % 2) == 0)
        {
            //left leg
            *leg = m_leg_seq_left[m_seq_count[leg_number]];
        }
        else
        {
            //right leg
            *leg = m_leg_seq_right[m_seq_count[leg_number]];
        }
        
        m_seq_count[leg_number]++;
        return 0;
    }
    return -1;
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

void calculate_trajectory(hexapod_leg_t endpoint_front, hexapod_leg_t endpoint_back, hexapod_dir_t direction, int32_t frames)
{
    //uses leg 0 (any leg can be used)
    hexapod_leg_t current_point = endpoint_back;
    
    int32_t diff_top = endpoint_front.leg_top - current_point.leg_top;
    int32_t diff_mid = endpoint_front.leg_mid - current_point.leg_mid;
    int32_t diff_bot = endpoint_front.leg_bot - current_point.leg_bot;
    
    int32_t right_way_lift_height = 0;
    int32_t left_way_lift_height = 0;
    
    bool turn = false;
    
    switch(direction)
    {
        case FORWARD:
            break;
        case BACKWARD:
            break;
        case RIGHT:
        case FORWARD_RIGHT:
        case BACKWARD_RIGHT:
            right_way_lift_height = LEG_LIFT_HEIGHT;
            break;
        case LEFT:
        case FORWARD_LEFT:
        case BACKWARD_LEFT:
            left_way_lift_height = LEG_LIFT_HEIGHT;
            break;
        case CW_TURN:
        case CCW_TURN:
            turn = true;
            break;
    }
    
    //TOP SERVO
    for(int i = 0; i < frames; i++)
    {
        m_leg_seq_right[i].leg_top = current_point.leg_top + diff_top*i/frames;
        m_leg_seq_right[frames*2 - i - 1].leg_top = current_point.leg_top + diff_top*(i+1)/frames;  

        if(turn)
        {
            m_leg_seq_left[i].leg_top = 3000 - m_leg_seq_right[i].leg_top;
            m_leg_seq_left[frames*2 - i - 1].leg_top = 3000 - m_leg_seq_right[i].leg_top;
        }
        else
        {
            m_leg_seq_left[i].leg_top = 3000 - (endpoint_front.leg_top - diff_top*i/frames);        
            m_leg_seq_left[frames*2 - i - 1].leg_top = 3000 - (endpoint_front.leg_top - diff_top*(i+1)/frames);
        }
    }
    
    
    //MIDDLE SERVO
    for(int i = 0; i < frames/2; i++)
    {
        //first quarter
        m_leg_seq_right[i].leg_mid = current_point.leg_mid + (diff_mid+2*LEG_LIFT_HEIGHT)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
        m_leg_seq_left[i].leg_mid = 3000 - (current_point.leg_mid + diff_mid*i/frames);
        //forth quarter
        m_leg_seq_right[frames*2 - i - 1].leg_mid = current_point.leg_mid + diff_mid*(i+1)/frames;
        m_leg_seq_left[frames*2 - i - 1].leg_mid = 3000 - (current_point.leg_mid + (diff_mid + 2*LEG_LIFT_HEIGHT)*(i+1)/frames);
    }
    for(int i = 0; i < frames/2; i++)
    {
        //second quarter
        m_leg_seq_right[i + frames/2].leg_mid = current_point.leg_mid + (diff_mid/2 + LEG_LIFT_HEIGHT) + diff_mid*i/frames - 2*LEG_LIFT_HEIGHT*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
        m_leg_seq_left[i + frames/2].leg_mid = 3000 - (current_point.leg_mid + diff_mid/2 + diff_mid*i/frames);
        //third quarter
        m_leg_seq_right[frames * 3 / 2 - i - 1].leg_mid = current_point.leg_mid + diff_mid/2 + diff_mid*(i+1)/frames;
        m_leg_seq_left[frames * 3 / 2 - i - 1].leg_mid = 3000 - (current_point.leg_mid + (diff_mid/2 + LEG_LIFT_HEIGHT) + diff_mid*(i+1)/frames - 2*LEG_LIFT_HEIGHT*(i+1)/frames);
    }
    
    //BOTTOM SERVO
    for(int i = 0; i < frames/2; i++)
    {
        //m_leg_seq[i].leg_bot = current_point.leg_bot + (diff_bot+2*LEG_LIFT_HEIGHT)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
        m_leg_seq_right[i].leg_bot = current_point.leg_bot + (diff_bot)*i/frames;
        m_leg_seq_left[i].leg_bot = 3000 - m_leg_seq_right[i].leg_bot;
        
        m_leg_seq_right[frames*2 - i - 1].leg_bot = current_point.leg_bot + diff_bot*(i+1)/frames;
        m_leg_seq_left[frames*2 - i - 1].leg_bot = 3000 - m_leg_seq_right[frames*2 - i - 1].leg_bot;
    }
    for(int i = 0; i < frames/2; i++)
    {
        //m_leg_seq[i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2 + LEG_LIFT_HEIGHT) + diff_bot*i/frames - 2*LEG_LIFT_HEIGHT*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
        m_leg_seq_right[i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2) + diff_bot*i/frames;
        m_leg_seq_left[i + frames/2].leg_bot = 3000 - m_leg_seq_right[i + frames/2].leg_bot;
        
        m_leg_seq_right[frames * 3 / 2 - i - 1].leg_bot = current_point.leg_bot + diff_bot/2 + diff_bot*(i+1)/frames;
        m_leg_seq_left[frames * 3 / 2 - i - 1].leg_bot = 3000 - m_leg_seq_right[frames * 3 / 2 - i - 1].leg_bot;
    }
    
    m_frames = frames*2;
    
}

void print_directory()
{
    NRF_LOG_PRINTF("directory:\nRight leg\t\t\tLeft leg\nT\tM\tB\t\tT\tM\tB\t\n");
    for(int i = 0; i < m_frames; i++)
    {
        NRF_LOG_PRINTF("%d\t%d\t%d\t\t%d\t%d\t%d\t\n", 
        m_leg_seq_right[i].leg_top, m_leg_seq_right[i].leg_mid, m_leg_seq_right[i].leg_bot,
        m_leg_seq_left[i].leg_top, m_leg_seq_left[i].leg_mid, m_leg_seq_left[i].leg_bot);
        nrf_delay_ms(10);
    }
}

void hexapod_move_forward(uint8_t speed)
{
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = false;
    }
    
    hexapod_leg_t endpoint_front = {1700, 1500, 1200};
    hexapod_leg_t endpoint_back = {1300, 1500, 1200};
    
    uint32_t frames = (1700 - 1300) / speed;
    calculate_trajectory(endpoint_front, endpoint_back, FORWARD, frames);
    
    //if group 1 and right leg -> start at frames
    //if group 2 and left leg ->start at frames
    
    //group 1
    m_seq_count[0] = 0;
    m_seq_count[3] = frames;
    m_seq_count[4] = 0;
    
    //group 2
    m_seq_count[1] = 0;
    m_seq_count[2] = frames;
    m_seq_count[5] = 0;
    
    //print_directory();
    
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = true;
    }
}

void hexapod_move_right(uint8_t speed)
{
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = false;
    }
    
    hexapod_leg_t endpoint_right = {1500, 1700, 1700};
    hexapod_leg_t endpoint_left = {1500, 1500, 1200};
    
    uint32_t frames = (1700 - 1200) / speed;
    calculate_trajectory(endpoint_right, endpoint_left, RIGHT, frames);
    
    //group 1
    m_seq_count[0] = 0;
    m_seq_count[3] = frames;
    m_seq_count[4] = 0;
    
    //group 2
    m_seq_count[1] = 0;
    m_seq_count[2] = frames;
    m_seq_count[5] = 0;
    
    //print_directory();
    
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = true;
    }
}

void hexapod_move_diagonal(uint8_t speed)
{
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = false;
    }
    
    hexapod_leg_t endpoint_front_right = {1700, 1700, 1700};
    hexapod_leg_t endpoint_back_left = {1300, 1500, 1200};
    
    uint32_t frames = (1700 - 1200) / speed;
    calculate_trajectory(endpoint_front_right, endpoint_back_left, RIGHT, frames);
    
    //group 1
    m_seq_count[0] = 0;
    m_seq_count[3] = frames;
    m_seq_count[4] = 0;
    
    //group 2
    m_seq_count[1] = 0;
    m_seq_count[2] = frames;
    m_seq_count[5] = 0;
    
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = true;
    }
}

void hexapod_turn_clockwise(uint8_t speed)
{
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = false;
    }
    
    hexapod_leg_t endpoint_front = {1700, 1500, 1200};
    hexapod_leg_t endpoint_back = {1300, 1500, 1200};
    
    uint32_t frames = (1700 - 1300) / speed;
    calculate_trajectory(endpoint_front, endpoint_back, CW_TURN, frames);
    
    //if group 1 and right leg -> start at frames
    //if group 2 and left leg ->start at frames
    
    //group 1
    m_seq_count[0] = 0;
    m_seq_count[3] = frames;
    m_seq_count[4] = 0;
    
    //group 2
    m_seq_count[1] = 0;
    m_seq_count[2] = frames;
    m_seq_count[5] = 0;
    
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = true;
    }
}

void calculate_change_trajectory(hexapod_dir_t last_dir, hexapod_dir_t new_dir)
{
    hexapod_leg_t endpoint_1;
    hexapod_leg_t endpoint_2;
    
    switch(last_dir)
    {
        case FORWARD:
        case BACKWARD:
            endpoint_1 = (hexapod_leg_t){1700, 1500, 1200};
            break;
        case RIGHT:
        case LEFT:
            endpoint_1 = (hexapod_leg_t){1500, 1700, 1700};
            break;
        case FORWARD_RIGHT:
        case BACKWARD_RIGHT:
            //not implemented yet
            break;
        case FORWARD_LEFT:
        case BACKWARD_LEFT:
            //not implemented yet
            break;
        case CW_TURN:
        case CCW_TURN:
            //not implemented yet
            break;
        case STOP:
            break;
    }
    
    switch(new_dir)
    {
        case FORWARD:
        case BACKWARD:
            endpoint_2 = (hexapod_leg_t){1300, 1500, 1200};
            break;
        case RIGHT:
        case LEFT:
            endpoint_2 = (hexapod_leg_t){1500, 1500, 1200};
            break;
        case FORWARD_RIGHT:
        case BACKWARD_RIGHT:
            //not implemented yet
            break;
        case FORWARD_LEFT:
        case BACKWARD_LEFT:
            //not implemented yet
            break;
        case CW_TURN:
        case CCW_TURN:
            //not implemented yet
            break;
        case STOP:
            break;
    }
}

*/

