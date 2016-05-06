
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

static uint32_t m_speed = MIN_SPEED;

//static hexapod_leg_t m_leg_seq[6][MAX_FRAMES];

static hexapod_leg_t m_leg_seq_right[MAX_FRAMES*2];
static hexapod_leg_t m_leg_seq_left[MAX_FRAMES*2];

static uint32_t m_seq_count[6] = {0, 0, 0, 0, 0, 0};
static int32_t m_frames = 0;
static bool m_seq_updated[6] = {false, false, false, false, false, false};

static bool m_lift_group1 = false;
static bool m_repeat_seq = true;
static bool m_change_direction = false;
static hexapod_dir_t m_direction = STOP;

////////////////
static int32_t m_count[6] = {0, 0, 0, 0, 0, 0};
static bool direction[6];
static bool new_direction[6];
static hexapod_leg_t m_current_point[6];
static uint32_t m_endpoint_reached = 6;
static bool is_moving = false;

//TODO change this with state variables and state machine
typedef enum
{
    IDLE,
    TRANSITION,
    NEW_ENDPOINTS,
} hexapod_state_t;

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
    
    
    m_current_point[0] = DEF_POSE_LEFT_LEG;
    m_current_point[1] = DEF_POSE_RIGHT_LEG;
    m_current_point[2] = DEF_POSE_LEFT_LEG;
    m_current_point[3] = DEF_POSE_RIGHT_LEG;
    m_current_point[4] = DEF_POSE_LEFT_LEG;
    m_current_point[5] = DEF_POSE_RIGHT_LEG;
    
    m_endpoint_out_left = DEF_POSE_LEFT_LEG;
    m_endpoint_out_right = DEF_POSE_RIGHT_LEG;
    m_endpoint_in_left = DEF_POSE_LEFT_LEG;
    m_endpoint_in_right = DEF_POSE_RIGHT_LEG;
    
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
    
    if(direction[leg_nr] == true)
    {
        //check if left leg
        if((leg_nr % 2) == 0)
        {
            endpoint_1 = m_endpoint_in_left;
            if(m_state == TRANSITION)
            {
                endpoint_2 = m_new_endpoint_out_left;
            }
            else
            {
                endpoint_2 = m_endpoint_out_left;
            }
            leg_lift_height = -LEG_LIFT_HEIGHT;
        }
        else
        {
            endpoint_1 = m_endpoint_in_right;
            if(m_state == TRANSITION)
            {
                endpoint_2 = m_new_endpoint_out_right;
            }
            else
            {
                endpoint_2 = m_endpoint_out_right;
            }
            leg_lift_height = LEG_LIFT_HEIGHT;
        }
    }
    else
    {
        //check if left leg
        if((leg_nr % 2) == 0)
        {
            endpoint_1 = m_endpoint_out_left;
            if(m_state == TRANSITION)
            {
                endpoint_2 = m_new_endpoint_in_left;
            }
            else
            {
                endpoint_2 = m_endpoint_in_left;
            }
        }
        else
        {
            endpoint_1 = m_endpoint_out_right;
            if(m_state == TRANSITION)
            {
                endpoint_2 = m_new_endpoint_in_right;
            }
            else
            {
                endpoint_2 = m_endpoint_in_right;
            }
        }
    }
    
    //going from endpoint_1 to endpoint_2
            
    //top leg
    m_current_point[leg_nr].leg_top = endpoint_1.leg_top + (endpoint_2.leg_top - endpoint_1.leg_top) * m_count[leg_nr] / m_frames;
    
    //mid leg
    //calculate lift height
    if(leg_lift_height != 0)
    {
        if(m_count[leg_nr] < m_frames/2)
        {
            point_lift_height = 2 * m_count[leg_nr] * leg_lift_height / m_frames;
        }
        else
        {
            point_lift_height = leg_lift_height - (2 * m_count[leg_nr] - m_frames) * leg_lift_height / m_frames;
        }
    }
    
    m_current_point[leg_nr].leg_mid = endpoint_1.leg_mid + (endpoint_2.leg_mid - endpoint_1.leg_mid) * m_count[leg_nr] / m_frames + point_lift_height;
    
    //bottom leg
    m_current_point[leg_nr].leg_bot = endpoint_1.leg_bot + (endpoint_2.leg_bot - endpoint_1.leg_bot) * m_count[leg_nr] / m_frames;
    
    //increase count value
    m_count[leg_nr]++;
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
    if(is_moving == false)
    {
        return -1;
    }
    
    *leg = m_current_point[leg_nr];
    calc_next_points_leg(leg_nr);
    /*
    if(leg_nr == 4)
    {
        NRF_LOG_PRINTF("%d\t%d\t%d\n", leg->leg_top, leg->leg_mid, leg->leg_bot);
        NRF_LOG_PRINTF("Count: %d\n", m_count[leg_nr]);
    }
    */
    if(m_count[leg_nr] >= m_frames)
    {
        //we are at a enpoint
        
        //change direction
        direction[leg_nr] = !direction[leg_nr];
        
        switch(m_state)
        {
            case IDLE:
                break;
            case TRANSITION:
                m_state = NEW_ENDPOINTS;
                break;
            case NEW_ENDPOINTS:
                
                //CHANGE THIS
                m_frames = 400;
            
                m_endpoint_out_left = m_new_endpoint_out_left;
                m_endpoint_out_right = m_new_endpoint_out_right;
                m_endpoint_in_left = m_new_endpoint_in_left;
                m_endpoint_in_right = m_new_endpoint_in_right;
                
                for(int i = 0; i < NUMBER_OF_LEGS; i++)
                {
                    direction[i] = new_direction[i];
                }
                m_state = IDLE;
                break;
            default:
                //ERROR!
                break;
        }

        m_count[leg_nr] = 0;
    }
    return 0;
}

void print_trajectory(uint32_t leg_nr)
{
    hexapod_leg_t leg;
    NRF_LOG_PRINTF("directory:\nleg number: %d\nT\tM\tB\n", leg_nr);
    for(int i = 0; i < m_frames; i++)
    {
        hexapod_get_next_seq_value(leg_nr, &leg);
        NRF_LOG_PRINTF("%d\t%d\t%d\n", 
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

void hexapod_move_forward2(uint8_t speed)
{
    
    //CHANGE THIS!
    m_frames = 200 / speed;
    
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
    
    
    
    new_direction[0] = false;
    new_direction[1] = true;
    new_direction[2] = true;
    new_direction[3] = false;
    new_direction[4] = false;
    new_direction[5] = true;
    
    for(int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        m_count[i] = 0;
    }
    
    is_moving = true;
    
    //print_trajectory(0);
}



///old using sequence:

void change_direction(hexapod_dir_t direction)
{
    m_direction = direction;
    m_change_direction = true;
}
/*
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
*/


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

void hexapod_move(uint8_t x, uint8_t y)
{
    uint8_t speed = sqrt(x*x + y*y);
    
    
    
}

