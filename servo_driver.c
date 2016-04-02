
#include <stdio.h>
#include <string.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "app_button.h"
#include "app_timer.h"

#include "pwm_driver.h"

#define NR_OF_SERVOS        18

#define FRONT_LEFT_BOTTOM   5
#define FRONT_LEFT_MID      6
#define FRONT_LEFT_TOP      7

#define FRONT_RIGHT_BOTTOM  31
#define FRONT_RIGHT_MID     30
#define FRONT_RIGHT_TOP     29

#define MID_LEFT_BOTTOM     8
#define MID_LEFT_MID        12
#define MID_LEFT_TOP        11

#define MID_RIGHT_BOTTOM    4
#define MID_RIGHT_MID       13
#define MID_RIGHT_TOP       28

#define BACK_LEFT_BOTTOM    18
#define BACK_LEFT_MID       16
#define BACK_LEFT_TOP       17

#define BACK_RIGHT_BOTTOM   3
#define BACK_RIGHT_MID      15
#define BACK_RIGHT_TOP      14

/*
uint8_t servo_pins[] = {FRONT_LEFT_BOTTOM, FRONT_LEFT_MID, FRONT_LEFT_TOP, 
                        FRONT_RIGHT_BOTTOM, FRONT_RIGHT_MID, FRONT_RIGHT_TOP, 
                        MID_LEFT_BOTTOM, MID_LEFT_MID, MID_LEFT_TOP,
                        MID_RIGHT_BOTTOM, MID_RIGHT_MID, MID_RIGHT_TOP,
                        BACK_LEFT_BOTTOM, BACK_LEFT_MID, BACK_LEFT_TOP,
                        BACK_RIGHT_BOTTOM, BACK_RIGHT_MID, BACK_RIGHT_TOP};

#define NR_OF_STEPS 100
#define PWM_TOP_VALUE 2500
#define BOTTOM_MID_LEG_LOW_VALUE    (PWM_TOP_VALUE/20 + PWM_TOP_VALUE/20/3)
#define BOTTOM_MID_LEG_HIGH_VALUE    (PWM_TOP_VALUE/20 + PWM_TOP_VALUE/20*2/3)
#define TOP_LEG_LOW_VALUE           (PWM_TOP_VALUE/20 + PWM_TOP_VALUE/20/3)
#define TOP_LEG_HIGH_VALUE           (PWM_TOP_VALUE/20 + PWM_TOP_VALUE/20*2/3)

typedef struct
{
    uint16_t channel_0; ///< Duty cycle value for channel 0.
    uint16_t channel_1; ///< Duty cycle value for channel 1.
    uint16_t channel_2; ///< Duty cycle value for channel 2.
    uint16_t channel_3; ///< Duty cycle value for channel 3.
} nrf_pwm_values_individual_t;

static nrf_pwm_values_individual_t seq0[NR_OF_STEPS];
static nrf_pwm_values_individual_t seq1[NR_OF_STEPS];

void servo_init()
{
    
}

void leg_test_no_driver()
{
    NRF_PWM0->PSEL.OUT[0] = (BACK_LEFT_BOTTOM << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM0->PSEL.OUT[1] = (BACK_LEFT_MID << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM0->PSEL.OUT[2] = (BACK_LEFT_TOP << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    
    NRF_PWM0->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM0->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_128 << PWM_PRESCALER_PRESCALER_Pos);
    
    NRF_PWM0->COUNTERTOP  = (2500 << PWM_COUNTERTOP_COUNTERTOP_Pos);
    NRF_PWM0->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    NRF_PWM0->DECODER   = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | 
                          (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->SEQ[0].PTR  = ((uint32_t)(seq0) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT  = ((sizeof(seq0) / sizeof(uint16_t)) <<
                                                     PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].REFRESH  = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    
    NRF_PWM0->SEQ[1].PTR  = ((uint32_t)(seq1) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[1].CNT  = ((sizeof(seq1) / sizeof(uint16_t)) <<
                                                     PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[1].REFRESH  = 0;
    NRF_PWM0->SEQ[1].ENDDELAY = 0;
    
    //NRF_PWM0->SHORTS = (PWM_SHORTS_LOOPSDONE_STOP_Enabled << PWM_SHORTS_LOOPSDONE_STOP_Pos);
    
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_PWM0->TASKS_SEQSTART[1];
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_PWM0->EVENTS_SEQEND[0];
    //NRF_PPI->CHENSET = (PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos);
    
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_PWM0->TASKS_SEQSTART[0];
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_PWM0->EVENTS_SEQEND[1];
    //NRF_PPI->CHENSET = (PPI_CHENSET_CH1_Enabled << PPI_CHENSET_CH1_Pos);
    
    NRF_PPI->CHG[0] = (PPI_CHG0_CH0_Included << PPI_CHG0_CH0_Pos)
                    | (PPI_CHG0_CH1_Included << PPI_CHG0_CH1_Pos);
    
    uint32_t value;
    
    //bottom and mid leg forward
    for(int i = 0; i < NR_OF_STEPS/2; i++)
    {
        value = BOTTOM_MID_LEG_LOW_VALUE + (BOTTOM_MID_LEG_HIGH_VALUE - BOTTOM_MID_LEG_LOW_VALUE)*i*2/NR_OF_STEPS;
        seq0[i].channel_0 = value | 0x8000;     //invert
        seq0[i].channel_1 = value | 0x8000;     //invert
    }
    for(int i = 0; i < NR_OF_STEPS/2; i++)
    {
        value = BOTTOM_MID_LEG_HIGH_VALUE - (BOTTOM_MID_LEG_HIGH_VALUE - BOTTOM_MID_LEG_LOW_VALUE)*i*2/NR_OF_STEPS;
        seq0[i + NR_OF_STEPS/2].channel_0 = value | 0x8000;     //invert
        seq0[i + NR_OF_STEPS/2].channel_1 = value | 0x8000;     //invert
    }
    
    //top leg forward
    for(int i = 0; i < NR_OF_STEPS; i++)
    {
        value = TOP_LEG_HIGH_VALUE - (TOP_LEG_HIGH_VALUE - TOP_LEG_LOW_VALUE) * i / NR_OF_STEPS;
        seq0[i].channel_2 = value | 0x8000;
    }
    
    //all legs backward
    for(int i = 0; i < NR_OF_STEPS; i++)
    {
        value = TOP_LEG_LOW_VALUE + (TOP_LEG_HIGH_VALUE - TOP_LEG_LOW_VALUE) * i / NR_OF_STEPS;
        seq1[i].channel_2 = value | 0x8000;
        
        seq1[i].channel_0 = 0 | 0x8000;
        seq1[i].channel_1 = 0 | 0x8000;
    }
    
    NVIC_EnableIRQ(PWM0_IRQn);
    NVIC_SetPriority(PWM0_IRQn, 3);
}

static volatile uint8_t last_sequence_played = 0;

void PWM0_IRQHandler(void)
{
    if(NRF_PWM0->EVENTS_SEQEND[0])
    {
        NRF_PWM0->EVENTS_SEQEND[0] = 0;
        last_sequence_played = 0;
    }
    if(NRF_PWM0->EVENTS_SEQEND[1])
    {
        NRF_PWM0->EVENTS_SEQEND[1] = 0;
        last_sequence_played = 1;
    }
}

#define BUTTON_PIN 27

#define NUM_OF_BUTTONS 2
#define APP_TIMER_PRESCALER 0
#define APP_TIMER_OP_QUEUE_SIZE 6

static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
    {
        NRF_PWM0->INTENSET = (PWM_INTENSET_SEQEND0_Enabled << PWM_INTENSET_SEQEND0_Pos)
                            | (PWM_INTENSET_SEQEND1_Enabled << PWM_INTENSET_SEQEND1_Pos);
        if(last_sequence_played == 0)
        {
            NRF_PWM0->TASKS_SEQSTART[0] = 1;
        }
        else
        {
            NRF_PWM0->TASKS_SEQSTART[1] = 1;
        }
        NRF_PPI->TASKS_CHG[0].EN = 1;
    }
    else
    {
        NRF_PPI->TASKS_CHG[0].DIS = 1;
        NRF_PWM0->INTENCLR = (PWM_INTENCLR_SEQEND0_Enabled << PWM_INTENCLR_SEQEND0_Pos)
                            | (PWM_INTENCLR_SEQEND1_Enabled << PWM_INTENCLR_SEQEND1_Pos);
    }
}

static const app_button_cfg_t app_buttons[NUM_OF_BUTTONS] = 
{
    {BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
};

void button_init()
{
    uint32_t err_code;
    
    //start LFCLK, if SoftDevice is used this is started in sd_ble_enable
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    //app_button uses app_timer, if this is not initialize, then initialize it here
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    //init app_button module, 50ms detection delay (button debouncing)
    err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                   NUM_OF_BUTTONS,
                                   APP_TIMER_TICKS(50, APP_TIMER_PRESCALER));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
*/
