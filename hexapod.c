
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "hexapod.h"
#include "pwm_driver.h"
#include "segger_rtt.h"
#include "nrf_delay.h"

#define MAX_SPEED 100 //movements per cycle (1000)
#define MIN_SPEED 10
#define MAX_FRAMES 100

#define LEG_DEFAULT_VALUE 1500

#define DEF_POSE_LEFT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1800}
#define DEF_POSE_RIGHT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1200}

#define DEF_POSE_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, 1200}

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
} hexapod_dir_t;

/*
For writability legs are numbered 0-5 from front left to bottom right

0 ---|  |--- 1
2 ---|  |--- 3
4 ---|  |--- 5

Group 1 is leg 0, 3 and 4
Group 2 is leg 1, 2 and 5 

*/

static uint32_t m_speed = MIN_SPEED;

//static hexapod_leg_t m_leg_seq[6][MAX_FRAMES];

//static hexapod_leg_t m_leg_seq[MAX_FRAMES*2];
static hexapod_leg_t m_leg_seq_right[MAX_FRAMES*2];
static hexapod_leg_t m_leg_seq_left[MAX_FRAMES*2];

static uint32_t m_seq_count[6] = {0, 0, 0, 0, 0, 0};
static uint32_t m_frames = 0;
static bool m_seq_updated[6] = {false, false, false, false, false, false};

static bool m_lift_group1 = false;

void hexapod_init()
{
    hexapod_leg_t leg_pins[6];
    
    leg_pins[0] = (hexapod_leg_t){FRONT_LEFT_TOP, FRONT_LEFT_MID, FRONT_LEFT_BOT};
    leg_pins[1] = (hexapod_leg_t){FRONT_RIGHT_TOP, FRONT_RIGHT_MID, FRONT_RIGHT_BOT};
    leg_pins[2] = (hexapod_leg_t){MID_LEFT_TOP, MID_LEFT_MID, MID_LEFT_BOT};
    leg_pins[3] = (hexapod_leg_t){MID_RIGHT_TOP, MID_RIGHT_MID, MID_RIGHT_BOT};
    leg_pins[4] = (hexapod_leg_t){BACK_LEFT_TOP, BACK_LEFT_MID, BACK_LEFT_BOT};
    leg_pins[5] = (hexapod_leg_t){BACK_RIGHT_TOP, BACK_RIGHT_MID, BACK_RIGHT_BOT};
    
    //m_leg_seq[0] = DEF_POSE_LEG;
    m_leg_seq_right[0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq_left[0] = DEF_POSE_LEFT_LEG;
    
   /* 
    m_leg_seq[0][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[1][0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq[2][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[3][0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq[4][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[5][0] = DEF_POSE_RIGHT_LEG;
    */
    for(int i = 0; i < 6; i++)
    {
        m_seq_count[i] = 0;
        m_seq_updated[i] = true;
    }
    m_frames = 1;
    
    hexapod_servo_pwm_init(leg_pins);
}


hexapod_leg_t hexapod_invert_leg(hexapod_leg_t leg)
{
    hexapod_leg_t inverted_leg;
    
    //leg value = (1500 - leg value) + 1500
    inverted_leg.leg_top = 3000 - leg.leg_top;
    inverted_leg.leg_mid = 3000 - leg.leg_mid;
    inverted_leg.leg_bot = 3000 - leg.leg_bot;
    
    return inverted_leg;
}

int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg)
{
    if(m_seq_updated[leg_number])
    {
        if(m_seq_count[leg_number] == m_frames)
        {
            m_seq_count[leg_number] = 0;
        }
        if((leg_number % 2) == 0)
        {
            //left leg
            //*leg = hexapod_invert_leg(m_leg_seq[m_seq_count[leg_number]]);
            *leg = m_leg_seq_left[m_seq_count[leg_number]];
        }
        else
        {
            //*leg = m_leg_seq[m_seq_count[leg_number]];
            *leg = m_leg_seq_right[m_seq_count[leg_number]];
        }
        
        m_seq_count[leg_number]++;
        return 0;
    }
    return -1;
}

int32_t abs_int32(int32_t value)
{
    if(value < 0)
    {
        return -value;
    }
    return value;
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

void hexapod_stop()
{
    
}

void hexapod_move(uint8_t x, uint8_t y)
{
    uint8_t speed = sqrt(x*x + y*y);
    
    
    
}

