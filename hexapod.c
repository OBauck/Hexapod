
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "hexapod.h"
#include "pwm_driver.h"

#define MAX_SPEED 100 //movements per cycle (1000)
#define MIN_SPEED 10
#define MAX_FRAMES 100

#define DEF_POSE_LEFT_LEG (hexapod_leg_t){1500, 1500, 1500}
#define DEF_POSE_RIGHT_LEG (hexapod_leg_t){1500, 1500, 1500}

#define LEG_LIFT_HEIGHT_LEFT 50
#define LEG_LIFT_HEIGHT_RIGHT -50

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

#define BACK_RIGHT_BOT   3
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

*/

static uint32_t m_speed = MIN_SPEED;

static hexapod_leg_t m_leg_seq[6][MAX_FRAMES];

static uint32_t m_seq_count = 0;
static uint32_t m_frames = 0;
static bool m_seq_updated = false;

void hexapod_init()
{
    hexapod_leg_t leg_pins[6];
    
    leg_pins[0] = (hexapod_leg_t){FRONT_LEFT_TOP, FRONT_LEFT_MID, FRONT_LEFT_BOT};
    leg_pins[1] = (hexapod_leg_t){FRONT_RIGHT_TOP, FRONT_RIGHT_MID, FRONT_RIGHT_BOT};
    leg_pins[2] = (hexapod_leg_t){MID_LEFT_TOP, MID_LEFT_MID, MID_LEFT_BOT};
    leg_pins[3] = (hexapod_leg_t){MID_RIGHT_TOP, MID_RIGHT_MID, MID_RIGHT_BOT};
    leg_pins[4] = (hexapod_leg_t){BACK_LEFT_TOP, BACK_LEFT_MID, BACK_LEFT_BOT};
    leg_pins[5] = (hexapod_leg_t){BACK_RIGHT_TOP, BACK_RIGHT_MID, BACK_RIGHT_BOT};
    
    m_leg_seq[0][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[1][0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq[2][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[3][0] = DEF_POSE_RIGHT_LEG;
    m_leg_seq[4][0] = DEF_POSE_LEFT_LEG;
    m_leg_seq[5][0] = DEF_POSE_RIGHT_LEG;
    
    m_seq_count = 0;
    m_frames = 1;
    m_seq_updated = true;
    
    hexapod_servo_pwm_init(leg_pins);
}

//Correct use of *leg?
int32_t hexapod_get_next_seq_value(uint32_t leg_number, hexapod_leg_t *leg)
{
    if(m_seq_updated)
    {
        if(m_seq_count == m_frames)
        {
            m_seq_updated = false;
        }
        leg = &m_leg_seq[leg_number][m_seq_count];
        int32_t ret_val = m_seq_count;
        m_seq_count++;
        return ret_val;
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

//does not work for diagonal movement
void calculate_trajectory(hexapod_leg_t endpoint, hexapod_leg_t current_point, hexapod_dir_t direction, uint32_t leg_nr)
{
    m_seq_updated = false;
    
    int32_t diff_top = endpoint.leg_top - current_point.leg_top;
    int32_t diff_mid = endpoint.leg_mid - current_point.leg_mid;
    int32_t diff_bot = endpoint.leg_bot - current_point.leg_bot;

    static uint32_t frames = 0;
    bool lift_leg = false;
    bool is_left_leg = true;
    
    if(leg_nr % 2)
    {
        is_left_leg = false;
    }
    
    //TODO:
    //check if leg is already
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
        m_leg_seq[leg_nr][i-1].leg_top = current_point.leg_top + diff_top/frames*i;
    }
    
    
    uint32_t leg_lift_height = 0;
    if(lift_leg)
    {
        leg_lift_height = is_left_leg ? LEG_LIFT_HEIGHT_LEFT : LEG_LIFT_HEIGHT_RIGHT;
    }
    //MIDDLE SERVO
    for(int i = 1; i < frames/2; i++)
    {
        m_leg_seq[leg_nr][i-1].leg_mid = current_point.leg_mid + (diff_mid+2*leg_lift_height)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
    }
    for(int i = 0; i <= frames/2; i++)
    {
        m_leg_seq[leg_nr][i + frames/2].leg_mid = current_point.leg_mid + (diff_mid/2 + leg_lift_height) + (diff_mid-2*leg_lift_height)*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
    }
    
    //BOTTOM SERVO
    for(int i = 1; i < frames/2; i++)
    {
        m_leg_seq[leg_nr][i-1].leg_mid = current_point.leg_bot + (diff_bot+2*leg_lift_height)*i/frames;     //curr + (diff/2+height)*i/(frames/2)
    }
    for(int i = 0; i <= frames/2; i++)
    {
        m_leg_seq[leg_nr][i + frames/2].leg_bot = current_point.leg_bot + (diff_bot/2 + leg_lift_height) + (diff_mid-2*leg_lift_height)*i/frames; //curr + (diff/2+height) + (diff-diff/2-height)*i/(frames/2)
    }
    
    m_frames = frames;
    m_seq_count = 0;
    
    m_seq_updated = true;
}

void calc_next_point()
{
    
}

