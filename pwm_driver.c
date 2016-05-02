

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "app_util_platform.h"
#include "pwm_driver.h"
#include "hexapod.h"
#include "nrf_delay.h"

typedef struct
{
    uint16_t channel_0; ///< Duty cycle value for channel 0.
    uint16_t channel_1; ///< Duty cycle value for channel 1.
    uint16_t channel_2; ///< Duty cycle value for channel 2.
    uint16_t channel_3; ///< Duty cycle value for channel 3.
} nrf_pwm_values_individual_t;

nrf_pwm_values_individual_t pwm0_seq0;
nrf_pwm_values_individual_t pwm0_seq1;
nrf_pwm_values_individual_t pwm1_seq0;
nrf_pwm_values_individual_t pwm1_seq1;
nrf_pwm_values_individual_t pwm2_seq0;
nrf_pwm_values_individual_t pwm2_seq1;

static void pwm_gpiote_setup(uint8_t channel, uint32_t pin)
{
    NRF_GPIOTE->CONFIG[channel] =   (GPIOTE_CONFIG_MODE_Task        << GPIOTE_CONFIG_MODE_Pos) |
                                    (pin                            << GPIOTE_CONFIG_PSEL_Pos) |
                                    (GPIOTE_CONFIG_POLARITY_Toggle  << GPIOTE_CONFIG_POLARITY_Pos) |
                                    (GPIOTE_CONFIG_OUTINIT_High     << GPIOTE_CONFIG_OUTINIT_Pos);
}

void hexapod_servo_soft_pwm_init(hexapod_leg_t leg_pins[2])
{
    //TIMER setup
    
    //Setup shorts and period for all timers
    NRF_TIMER4->MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
    NRF_TIMER4->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER4->CC[0] = 20000;
    
    NRF_TIMER3->MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
    NRF_TIMER3->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER3->CC[0] = 20000;
    
    //channel used for updating values (at 2ms when clear task should have been executed)
    NRF_TIMER4->CC[4] = 2000;
    //turn on interrupt on this channel at highest priority
    NRF_TIMER4->INTENSET = (TIMER_INTENSET_COMPARE4_Enabled << TIMER_INTENSET_COMPARE4_Pos);
    NVIC_SetPriority(TIMER4_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_EnableIRQ(TIMER4_IRQn);
    
    //GPIOTE setup
    pwm_gpiote_setup(0, leg_pins[0].leg_top);
    pwm_gpiote_setup(1, leg_pins[0].leg_mid);
    pwm_gpiote_setup(2, leg_pins[0].leg_bot);
    pwm_gpiote_setup(3, leg_pins[1].leg_top);
    pwm_gpiote_setup(4, leg_pins[1].leg_mid);
    pwm_gpiote_setup(5, leg_pins[1].leg_bot);
    
    //PPI setup
    
    //Set output on start of every period
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[0];
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];

    NRF_PPI->FORK[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[1];

    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[2];
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];

    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[3];
    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[0];
    
    NRF_PPI->FORK[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[4];
    
    NRF_PPI->CH[3].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[5];
    NRF_PPI->CH[3].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[0];
    
    //clear output on start of CC compare
    NRF_PPI->CH[4].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[0];
    NRF_PPI->CH[4].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[1];
    
    NRF_PPI->CH[5].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[1];
    NRF_PPI->CH[5].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[2];
    
    NRF_PPI->CH[6].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[2];
    NRF_PPI->CH[6].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[3];
    
    NRF_PPI->CH[7].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[3];
    NRF_PPI->CH[7].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[1];
    
    NRF_PPI->CH[8].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[4];
    NRF_PPI->CH[8].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[2];
    
    NRF_PPI->CH[9].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[5];
    NRF_PPI->CH[9].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[3];
    
    NRF_PPI->CHENSET =  (PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos) |
                        (PPI_CHENSET_CH1_Enabled << PPI_CHENSET_CH1_Pos) |
                        (PPI_CHENSET_CH2_Enabled << PPI_CHENSET_CH2_Pos) |
                        (PPI_CHENSET_CH3_Enabled << PPI_CHENSET_CH3_Pos) |
                        (PPI_CHENSET_CH4_Enabled << PPI_CHENSET_CH4_Pos) |
                        (PPI_CHENSET_CH5_Enabled << PPI_CHENSET_CH5_Pos) |
                        (PPI_CHENSET_CH6_Enabled << PPI_CHENSET_CH6_Pos) |
                        (PPI_CHENSET_CH7_Enabled << PPI_CHENSET_CH7_Pos) |
                        (PPI_CHENSET_CH8_Enabled << PPI_CHENSET_CH8_Pos) |
                        (PPI_CHENSET_CH9_Enabled << PPI_CHENSET_CH9_Pos);
    
}

void TIMER4_IRQHandler(void)
{
    NRF_TIMER4->EVENTS_COMPARE[4] = 0;

    //update leg number 4 and 5 (back legs)
    
    hexapod_leg_t leg4;
    hexapod_leg_t leg5;

    if(hexapod_get_next_seq_value(4, &leg4) != -1)
    {
        NRF_TIMER3->CC[1] = leg4.leg_top;
        NRF_TIMER3->CC[2] = leg4.leg_mid;
        NRF_TIMER3->CC[3] = leg4.leg_bot;
    }
    if(hexapod_get_next_seq_value(5, &leg5) != -1)
    {
        NRF_TIMER4->CC[1] = leg5.leg_top;
        NRF_TIMER4->CC[2] = leg5.leg_mid;
        NRF_TIMER4->CC[3] = leg5.leg_bot;
    }
}

void hard_pwm_setup(NRF_PWM_Type *pwm_reg, uint32_t pins[4])
{
    pwm_reg->PSEL.OUT[0] = (pins[0] << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    pwm_reg->PSEL.OUT[1] = (pins[1] << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    pwm_reg->PSEL.OUT[2] = (pins[2] << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    pwm_reg->PSEL.OUT[3] = (pins[3] << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    
    pwm_reg->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    pwm_reg->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    pwm_reg->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
    
    pwm_reg->COUNTERTOP  = (20000 << PWM_COUNTERTOP_COUNTERTOP_Pos);
    pwm_reg->LOOP        = 0xFFFF;
    pwm_reg->SEQ[0].REFRESH  = 0;
    pwm_reg->SEQ[0].ENDDELAY = 0;
    
    pwm_reg->SEQ[1].REFRESH  = 0;
    pwm_reg->SEQ[1].ENDDELAY = 0;
    
    pwm_reg->DECODER   = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | 
                          (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    
    pwm_reg->INTENSET = (PWM_INTENSET_SEQEND0_Enabled << PWM_INTENSET_SEQEND0_Pos) |
                        (PWM_INTENSET_SEQEND1_Enabled << PWM_INTENSET_SEQEND1_Pos);
    
}

void hexapod_servo_hard_pwm_init(hexapod_leg_t leg_pins[4])
{
    uint32_t pwm0_pins[4] = {leg_pins[0].leg_top, leg_pins[0].leg_mid, leg_pins[0].leg_bot, leg_pins[3].leg_top};
    uint32_t pwm1_pins[4] = {leg_pins[1].leg_top, leg_pins[1].leg_mid, leg_pins[1].leg_bot, leg_pins[3].leg_mid};
    uint32_t pwm2_pins[4] = {leg_pins[2].leg_top, leg_pins[2].leg_mid, leg_pins[2].leg_bot, leg_pins[3].leg_bot};
    
    hard_pwm_setup(NRF_PWM0, pwm0_pins);
    hard_pwm_setup(NRF_PWM1, pwm1_pins);
    hard_pwm_setup(NRF_PWM2, pwm2_pins);
    
    NRF_PWM0->SEQ[0].PTR  = ((uint32_t)&(pwm0_seq0) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT  = ((sizeof(pwm0_seq0) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM0->SEQ[1].PTR  = ((uint32_t)&(pwm0_seq1) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[1].CNT  = ((sizeof(pwm0_seq1) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM1->SEQ[0].PTR  = ((uint32_t)&(pwm1_seq0) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM1->SEQ[0].CNT  = ((sizeof(pwm1_seq0) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM1->SEQ[1].PTR  = ((uint32_t)&(pwm1_seq1) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM1->SEQ[1].CNT  = ((sizeof(pwm1_seq1) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM2->SEQ[0].PTR  = ((uint32_t)&(pwm2_seq0) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM2->SEQ[0].CNT  = ((sizeof(pwm2_seq0) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM2->SEQ[1].PTR  = ((uint32_t)&(pwm2_seq1) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM2->SEQ[1].CNT  = ((sizeof(pwm2_seq1) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);    
    
    NVIC_SetPriority(PWM0_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_SetPriority(PWM1_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_SetPriority(PWM2_IRQn, APP_IRQ_PRIORITY_HIGH);
    
    NVIC_EnableIRQ(PWM0_IRQn);
    NVIC_EnableIRQ(PWM1_IRQn);
    NVIC_EnableIRQ(PWM2_IRQn);
}

void hexapod_servo_pwm_start()
{
    hexapod_leg_t leg_init_values;
    
    //leg 0
    if(hexapod_get_next_seq_value(0, &leg_init_values) != -1)
    {
        pwm0_seq0.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm0_seq0.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm0_seq0.channel_2 = leg_init_values.leg_bot | 0x8000;
        
        pwm0_seq1.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm0_seq1.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm0_seq1.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 1
    if(hexapod_get_next_seq_value(1, &leg_init_values) != -1)
    {
        pwm1_seq0.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm1_seq0.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm1_seq0.channel_2 = leg_init_values.leg_bot | 0x8000;
         
        pwm1_seq1.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm1_seq1.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm1_seq1.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 2
    if(hexapod_get_next_seq_value(2, &leg_init_values) != -1)
    {
        pwm2_seq0.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm2_seq0.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm2_seq0.channel_2 = leg_init_values.leg_bot | 0x8000;
        
        pwm2_seq1.channel_0 = leg_init_values.leg_top | 0x8000;
        pwm2_seq1.channel_1 = leg_init_values.leg_mid | 0x8000;
        pwm2_seq1.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 3
    if(hexapod_get_next_seq_value(3, &leg_init_values) != -1)
    {
        pwm0_seq0.channel_3 = leg_init_values.leg_top | 0x8000;
        pwm1_seq0.channel_3 = leg_init_values.leg_mid | 0x8000;
        pwm2_seq0.channel_3 = leg_init_values.leg_bot | 0x8000;
        
        pwm0_seq1.channel_3 = leg_init_values.leg_top | 0x8000;
        pwm1_seq1.channel_3 = leg_init_values.leg_mid | 0x8000;
        pwm2_seq1.channel_3 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 4
    if(hexapod_get_next_seq_value(4, &leg_init_values) != -1)
    {
        //leg 4 channels
        NRF_TIMER3->CC[1] = leg_init_values.leg_top;
        NRF_TIMER3->CC[2] = leg_init_values.leg_mid;
        NRF_TIMER3->CC[3] = leg_init_values.leg_bot;
    }
    
    //leg 5
    if(hexapod_get_next_seq_value(5, &leg_init_values) != -1)
    {
        //leg 5 channels
        NRF_TIMER4->CC[1] = leg_init_values.leg_top;
        NRF_TIMER4->CC[2] = leg_init_values.leg_mid;
        NRF_TIMER4->CC[3] = leg_init_values.leg_bot;
    }
    
    //Start timers and PWM with delay to limit current

    NRF_TIMER4->TASKS_START = 1;
    
    nrf_delay_ms(1000);
    
    //ppi to allow timer 3 to be in sync with timer 4
    NRF_PPI->CH[10].TEP = (uint32_t)&NRF_TIMER3->TASKS_START;
    NRF_PPI->CH[10].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[0];
    NRF_PPI->CHENSET = (PPI_CHENSET_CH10_Enabled << PPI_CHENSET_CH10_Pos);
    
    while(NRF_TIMER4->EVENTS_COMPARE[0] == 0);
    nrf_delay_ms(100);
        
    NRF_PPI->CHENCLR = (PPI_CHENCLR_CH10_Enabled << PPI_CHENCLR_CH10_Pos);
    
    NRF_PWM0->TASKS_SEQSTART[0] = 1;

    NRF_PWM1->TASKS_SEQSTART[0] = 1;

    NRF_PWM2->TASKS_SEQSTART[0] = 1;
}
    
void hexapod_servo_pwm_init(hexapod_leg_t leg_pins[6])
{
    hexapod_leg_t hard_pwm_pins[] = {leg_pins[0], leg_pins[1], leg_pins[2], leg_pins[3]};
    hexapod_leg_t soft_pwm_pins[] = {leg_pins[4], leg_pins[5]};
    
    hexapod_servo_hard_pwm_init(hard_pwm_pins);
    hexapod_servo_soft_pwm_init(soft_pwm_pins);
    
    hexapod_servo_pwm_start();
}

void PWM0_IRQHandler(void)
{
    hexapod_leg_t leg0;
    hexapod_leg_t leg3;
    
    if(NRF_PWM0->EVENTS_SEQEND[0] == 1)
    {
        NRF_PWM0->EVENTS_SEQEND[0] = 0;
        NRF_PWM0->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(0, &leg0) != -1)
        {
            pwm0_seq0.channel_0 = leg0.leg_top | 0x8000;
            pwm0_seq0.channel_1 = leg0.leg_mid | 0x8000;
            pwm0_seq0.channel_2 = leg0.leg_bot | 0x8000;
        }
        else
        {
            pwm0_seq0 = pwm0_seq1;  //or memcpy?
        }
        if(hexapod_get_next_seq_value(3, &leg3) != -1)
        {
            pwm0_seq0.channel_3 = leg3.leg_top | 0x8000;
            pwm1_seq0.channel_3 = leg3.leg_mid | 0x8000;
            pwm2_seq0.channel_3 = leg3.leg_bot | 0x8000;
            
        }
    }
    if(NRF_PWM0->EVENTS_SEQEND[1] == 1)
    {
        NRF_PWM0->EVENTS_SEQEND[1] = 0;
        NRF_PWM0->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(0, &leg0) != -1)
        {
            pwm0_seq1.channel_0 = leg0.leg_top | 0x8000;
            pwm0_seq1.channel_1 = leg0.leg_mid | 0x8000;
            pwm0_seq1.channel_2 = leg0.leg_bot | 0x8000;
        }
        else
        {
            pwm0_seq1 = pwm0_seq0;  //or memcpy?
        }
        if(hexapod_get_next_seq_value(3, &leg3) != -1)
        {
            pwm0_seq1.channel_3 = leg3.leg_top | 0x8000;
            pwm1_seq1.channel_3 = leg3.leg_mid | 0x8000;
            pwm2_seq1.channel_3 = leg3.leg_bot | 0x8000;
        }
    }
}

void PWM1_IRQHandler(void)
{
    hexapod_leg_t leg1;
    
    if(NRF_PWM1->EVENTS_SEQEND[0] == 1)
    {
        NRF_PWM1->EVENTS_SEQEND[0] = 0;
        NRF_PWM1->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(1, &leg1) != -1)
        {
            pwm1_seq0.channel_0 = leg1.leg_top | 0x8000;
            pwm1_seq0.channel_1 = leg1.leg_mid | 0x8000;
            pwm1_seq0.channel_2 = leg1.leg_bot | 0x8000;
        }
        else
        {
            pwm1_seq0 = pwm1_seq1;  //or memcpy?
        }
    }
    if(NRF_PWM1->EVENTS_SEQEND[1] == 1)
    {
        NRF_PWM1->EVENTS_SEQEND[1] = 0;
        NRF_PWM1->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(1, &leg1) != -1)
        {
            pwm1_seq1.channel_0 = leg1.leg_top | 0x8000;
            pwm1_seq1.channel_1 = leg1.leg_mid | 0x8000;
            pwm1_seq1.channel_2 = leg1.leg_bot | 0x8000;
        }
        else
        {
            pwm1_seq1 = pwm1_seq0;  //or memcpy?
        }
    }
}

void PWM2_IRQHandler(void)
{
    hexapod_leg_t leg2;
    
    if(NRF_PWM2->EVENTS_SEQEND[0] == 1)
    {
        NRF_PWM2->EVENTS_SEQEND[0] = 0;
        NRF_PWM2->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(2, &leg2) != -1)
        {
            pwm2_seq0.channel_0 = leg2.leg_top | 0x8000;
            pwm2_seq0.channel_1 = leg2.leg_mid | 0x8000;
            pwm2_seq0.channel_2 = leg2.leg_bot | 0x8000;
        }
        else
        {
            pwm2_seq0 = pwm2_seq1;  //or memcpy?
        }
    }
    if(NRF_PWM2->EVENTS_SEQEND[1] == 1)
    {
        NRF_PWM2->EVENTS_SEQEND[1] = 0;
        NRF_PWM2->LOOP = 0xFFFF;

        if(hexapod_get_next_seq_value(2, &leg2) != -1)
        {
            pwm2_seq1.channel_0 = leg2.leg_top | 0x8000;
            pwm2_seq1.channel_1 = leg2.leg_mid | 0x8000;
            pwm2_seq1.channel_2 = leg2.leg_bot | 0x8000;
        }
        else
        {
            pwm2_seq1 = pwm2_seq0;  //or memcpy?
        }
    }
}
