
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

//#define DEF_POSE_LEFT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE}
//#define DEF_POSE_RIGHT_LEG (hexapod_leg_t){LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE, LEG_DEFAULT_VALUE}

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
    BACKWARD_LEFT
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

static hexapod_leg_t m_leg_seq[MAX_FRAMES*2];

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
    
    m_leg_seq[0] = DEF_POSE_LEG;
    
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
            *leg = hexapod_invert_leg(m_leg_seq[m_seq_count[leg_number]]);
        }
        else
        {
            *leg = m_leg_seq[m_seq_count[leg_number]];
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
    
    //FORWARD MOVEMENT ONLY IMPLEMENTED
    
    //TOP SERVO
    for(int i = 0; i < frames; i++)
    {
        m_leg_seq[i].leg_top = current_point.leg_top + diff_top/frames*i;
        m_leg_seq[frames*2 - i - 1].leg_top = current_point.leg_top + diff_top/frames*(i+1);     
    }
    
    
    //MIDDLE SERVO
    for(int i = 0; i < frames/2; i++)
    {
        m_leg_seq[i].leg_mid = current_point.leg_mid + (diff_mid+2*LEG_LIFT_HEIGHT)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
        m_leg_seq[frames*2 - i - 1].leg_mid = current_point.leg_mid + diff_mid*(i+1)/frames;
    }
    for(int i = 0; i < frames/2; i++)
    {
        //static uint32_t temp;
        //temp = current_point.leg_mid + (diff_mid/2 + LEG_LIFT_HEIGHT) + diff_mid*i/frames - 2*LEG_LIFT_HEIGHT*i/frames;
        m_leg_seq[i + frames/2].leg_mid = current_point.leg_mid + (diff_mid/2 + LEG_LIFT_HEIGHT) + diff_mid*i/frames - 2*LEG_LIFT_HEIGHT*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
        m_leg_seq[frames * 3 / 2 - i - 1].leg_mid = current_point.leg_mid + diff_mid/2 + diff_mid*(i+1)/frames;
    }
    
    //BOTTOM SERVO
    for(int i = 0; i < frames/2; i++)
    {
        //m_leg_seq[i].leg_bot = current_point.leg_bot + (diff_bot+2*LEG_LIFT_HEIGHT)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
        m_leg_seq[i].leg_bot = current_point.leg_bot + (diff_bot)*i/frames;
        m_leg_seq[frames*2 - i - 1].leg_bot = current_point.leg_bot + diff_bot*(i+1)/frames;
    }
    for(int i = 0; i < frames/2; i++)
    {
        //m_leg_seq[i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2 + LEG_LIFT_HEIGHT) + diff_bot*i/frames - 2*LEG_LIFT_HEIGHT*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
        m_leg_seq[i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2) + diff_bot*i/frames;
        m_leg_seq[frames * 3 / 2 - i - 1].leg_bot = current_point.leg_bot + diff_bot/2 + diff_bot*(i+1)/frames;
    }
    
    m_frames = frames*2;
    
    /*
    bool lift_leg = false;
    bool is_left_leg = true;
    
    hexapod_leg_t current_point = m_leg_seq[leg_number][m_seq_count[leg_number]];
    
    int32_t diff_top = endpoint.leg_top - current_point.leg_top;
    int32_t diff_mid = endpoint.leg_mid - current_point.leg_mid;
    int32_t diff_bot = endpoint.leg_bot - current_point.leg_bot;
    
    if(leg_number % 2)
    {
        is_left_leg = false;
    }
    
    //TODO:
    //check if leg is already lifted (check sequence)
    //and implement rest of directions
    switch(direction)
    {
        case FORWARD:
            if( (diff_top < 0) && is_left_leg)
            {
                lift_leg = true;
            }
            if( (diff_top > 0) && !is_left_leg)
            {
                lift_leg = true;
            }
            break;
        case FORWARD_RIGHT:
            break;
        case FORWARD_LEFT:
            break;
        case RIGHT:
            break;
        case LEFT:
            break;
        case BACKWARD:
            if( (diff_top < 0) && !is_left_leg)
            {
                lift_leg = true;
            }
            if( (diff_top > 0) && is_left_leg)
            {
                lift_leg = true;
            }
            break;
        case BACKWARD_RIGHT:
            break;
        case BACKWARD_LEFT:
            break;
    }
    
    uint32_t diff_top_abs = abs_int32(diff_top);
    uint32_t diff_mid_abs = abs_int32(diff_mid);
    uint32_t diff_bot_abs = abs_int32(diff_bot);
    
    uint32_t max_diff = max(diff_top_abs, diff_mid_abs, diff_bot_abs);
    frames = max_diff/m_speed;
    
    //TOP SERVO
    for(int i = 1; i < frames; i++)
    {
        m_leg_seq[leg_number][i-1].leg_top = current_point.leg_top + diff_top/frames*i;
    }
    
    uint32_t leg_lift_height = 0;
    if(lift_leg)
    {
        leg_lift_height = is_left_leg ? LEG_LIFT_HEIGHT_LEFT : LEG_LIFT_HEIGHT_RIGHT;
    }
    //MIDDLE SERVO
    for(int i = 1; i < frames/2; i++)
    {
        m_leg_seq[leg_number][i-1].leg_mid = current_point.leg_mid + (diff_mid+2*leg_lift_height)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
    }
    for(int i = 0; i <= frames/2; i++)
    {
        m_leg_seq[leg_number][i + frames/2].leg_mid = current_point.leg_mid + (diff_mid/2 + leg_lift_height) + (diff_mid-2*leg_lift_height)*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
    }
    
    //BOTTOM SERVO
    for(int i = 1; i < frames/2; i++)
    {
        m_leg_seq[leg_number][i-1].leg_mid = current_point.leg_bot + (diff_bot+2*leg_lift_height)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
    }
    for(int i = 0; i <= frames/2; i++)
    {
        m_leg_seq[leg_number][i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2 + leg_lift_height) + (diff_mid-2*leg_lift_height)*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
    }
    
    m_frames = frames;
    */
}

void hexapod_move_forward(uint8_t speed)
{
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = false;
    }
    
    hexapod_leg_t endpoint_front = {1750, 1500, 1200};
    hexapod_leg_t endpoint_back = {1250, 1500, 1200};
    
    //uint32_t frames = (1750 - 1250) / speed;
    //calculate_trajectory(endpoint_front, endpoint_back, FORWARD, frames);
    
    hexapod_leg_t endpoint_right = {1500, 1700, 1700};
    hexapod_leg_t endpoint_left = {1500, 1500, 1200};
    
    uint32_t frames = (1700 - 1200) / speed;
    calculate_trajectory(endpoint_right, endpoint_left, RIGHT, frames);
    
    /*
    SEGGER_RTT_printf(0, "Trajectory:\tTop\tMid\tBot\n");
    for(int i = 0; i < frames*2; i++)
    {
        nrf_delay_ms(10);
        SEGGER_RTT_printf(0, "\t\t\t\t%d\t%d\t%d\n", m_leg_seq[i].leg_top, m_leg_seq[i].leg_mid, m_leg_seq[i].leg_bot);
    }
    SEGGER_RTT_printf(0, "\n");
    */
    
    //group 1
    m_seq_count[0] = 0;
    m_seq_count[3] = 0;
    m_seq_count[4] = 0;
    
    //group 2
    m_seq_count[1] = frames;
    m_seq_count[2] = frames;
    m_seq_count[5] = frames;
    
    for(int i = 0; i < 6; i++)
    {
        m_seq_updated[i] = true;
    }
}

void hexapod_move(uint8_t x, uint8_t y)
{
    uint8_t speed = sqrt(x*x + y*y);
    
    
    
}
