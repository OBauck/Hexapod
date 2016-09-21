

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "app_util_platform.h"
#include "servo_driver.h"
#include "hexapod.h"
#include "nrf_delay.h"
#include "nrf_drv_pwm.h"

#define PWM_TOP_VALUE 20000     //20000us

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);

nrf_pwm_values_individual_t m_pwm0_seq_values;
nrf_pwm_values_individual_t m_pwm1_seq_values;
nrf_pwm_values_individual_t m_pwm2_seq_values;

static nrf_pwm_sequence_t const m_pwm0_seq =
{
    .values.p_individual = &m_pwm0_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm0_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static nrf_pwm_sequence_t const m_pwm1_seq =
{
    .values.p_individual = &m_pwm1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static nrf_pwm_sequence_t const m_pwm2_seq =
{
    .values.p_individual = &m_pwm2_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm2_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

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
    NRF_TIMER4->CC[0] = PWM_TOP_VALUE;
    
    NRF_TIMER3->MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
    NRF_TIMER3->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER3->CC[0] = PWM_TOP_VALUE;
    
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

static void pwm0_handler(nrf_drv_pwm_evt_type_t event_type)
{
    hexapod_leg_t leg0;
    hexapod_leg_t leg3;

    if(hexapod_get_next_seq_value(0, &leg0) != -1)
    {
        m_pwm0_seq_values.channel_0 = leg0.leg_top | 0x8000;
        m_pwm0_seq_values.channel_1 = leg0.leg_mid | 0x8000;
        m_pwm0_seq_values.channel_2 = leg0.leg_bot | 0x8000;
    }

    if(hexapod_get_next_seq_value(3, &leg3) != -1)
    {
        m_pwm0_seq_values.channel_3 = leg3.leg_top | 0x8000;
        m_pwm1_seq_values.channel_3 = leg3.leg_mid | 0x8000;
        m_pwm2_seq_values.channel_3 = leg3.leg_bot | 0x8000;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &m_pwm0_seq, 1, 0);
}

static void pwm1_handler(nrf_drv_pwm_evt_type_t event_type)
{
    hexapod_leg_t leg1;
    
    if(hexapod_get_next_seq_value(1, &leg1) != -1)
    {
        m_pwm1_seq_values.channel_0 = leg1.leg_top | 0x8000;
        m_pwm1_seq_values.channel_1 = leg1.leg_mid | 0x8000;
        m_pwm1_seq_values.channel_2 = leg1.leg_bot | 0x8000;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm1, &m_pwm1_seq, 1, 0);
}

static void pwm2_handler(nrf_drv_pwm_evt_type_t event_type)
{
    hexapod_leg_t leg2;

    if(hexapod_get_next_seq_value(2, &leg2) != -1)
    {
        m_pwm2_seq_values.channel_0 = leg2.leg_top | 0x8000;
        m_pwm2_seq_values.channel_1 = leg2.leg_mid | 0x8000;
        m_pwm2_seq_values.channel_2 = leg2.leg_bot | 0x8000;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm2, &m_pwm2_seq, 1, 0);
}

void hard_pwm_setup(nrf_drv_pwm_t const * const p_instance, uint32_t pins[4], nrf_drv_pwm_handler_t handler)
{
    uint32_t err_code;
    
    nrf_drv_pwm_config_t const config =
    {
        .output_pins =
        {
            pins[0], // channel 0
            pins[1], // channel 1
            pins[2], // channel 2
            pins[3]  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = PWM_TOP_VALUE,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(p_instance, &config, handler);
    APP_ERROR_CHECK(err_code);
}

void hexapod_servo_hard_pwm_init(hexapod_leg_t leg_pins[4])
{
    uint32_t pwm0_pins[4] = {leg_pins[0].leg_top, leg_pins[0].leg_mid, leg_pins[0].leg_bot, leg_pins[3].leg_top};
    uint32_t pwm1_pins[4] = {leg_pins[1].leg_top, leg_pins[1].leg_mid, leg_pins[1].leg_bot, leg_pins[3].leg_mid};
    uint32_t pwm2_pins[4] = {leg_pins[2].leg_top, leg_pins[2].leg_mid, leg_pins[2].leg_bot, leg_pins[3].leg_bot};
    
    hard_pwm_setup(&m_pwm0, pwm0_pins, pwm0_handler);
    hard_pwm_setup(&m_pwm1, pwm1_pins, pwm1_handler);
    hard_pwm_setup(&m_pwm2, pwm2_pins, pwm2_handler);

}

void hexapod_servo_pwm_start()
{
    hexapod_leg_t leg_init_values;
    
    //leg 0
    if(hexapod_get_next_seq_value(0, &leg_init_values) != -1)
    {
        m_pwm0_seq_values.channel_0 = leg_init_values.leg_top | 0x8000;
        m_pwm0_seq_values.channel_1 = leg_init_values.leg_mid | 0x8000;
        m_pwm0_seq_values.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 1
    if(hexapod_get_next_seq_value(1, &leg_init_values) != -1)
    {
        m_pwm1_seq_values.channel_0 = leg_init_values.leg_top | 0x8000;
        m_pwm1_seq_values.channel_1 = leg_init_values.leg_mid | 0x8000;
        m_pwm1_seq_values.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 2
    if(hexapod_get_next_seq_value(2, &leg_init_values) != -1)
    {
        m_pwm2_seq_values.channel_0 = leg_init_values.leg_top | 0x8000;
        m_pwm2_seq_values.channel_1 = leg_init_values.leg_mid | 0x8000;
        m_pwm2_seq_values.channel_2 = leg_init_values.leg_bot | 0x8000;
    }
    
    //leg 3
    if(hexapod_get_next_seq_value(3, &leg_init_values) != -1)
    {
        m_pwm0_seq_values.channel_3 = leg_init_values.leg_top | 0x8000;
        m_pwm1_seq_values.channel_3 = leg_init_values.leg_mid | 0x8000;
        m_pwm2_seq_values.channel_3 = leg_init_values.leg_bot | 0x8000;
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
    
    //ppi to allow timer 3 to be in sync with timer 4
    NRF_PPI->CH[10].TEP = (uint32_t)&NRF_TIMER3->TASKS_START;
    NRF_PPI->CH[10].EEP = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[0];
    NRF_PPI->CHENSET = (PPI_CHENSET_CH10_Enabled << PPI_CHENSET_CH10_Pos);
    
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    while(NRF_TIMER4->EVENTS_COMPARE[0] == 0);
    //nrf_delay_ms(100);
        
    NRF_PPI->CHENCLR = (PPI_CHENCLR_CH10_Enabled << PPI_CHENCLR_CH10_Pos);

    nrf_drv_pwm_simple_playback(&m_pwm0, &m_pwm0_seq, 1, 0);
    nrf_drv_pwm_simple_playback(&m_pwm1, &m_pwm1_seq, 1, 0);
    nrf_drv_pwm_simple_playback(&m_pwm2, &m_pwm2_seq, 1, 0);
}

void hexapod_servo_pwm_stop()
{
    nrf_drv_pwm_stop(&m_pwm0, false);
    nrf_drv_pwm_stop(&m_pwm1, false);
    nrf_drv_pwm_stop(&m_pwm2, false);
    
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER4->TASKS_STOP = 1;
}
    
void hexapod_servo_pwm_init(hexapod_leg_t leg_pins[6])
{
    hexapod_leg_t hard_pwm_pins[] = {leg_pins[0], leg_pins[1], leg_pins[2], leg_pins[3]};
    hexapod_leg_t soft_pwm_pins[] = {leg_pins[4], leg_pins[5]};
    
    hexapod_servo_hard_pwm_init(hard_pwm_pins);
    hexapod_servo_soft_pwm_init(soft_pwm_pins);
    
    hexapod_servo_pwm_start();
}

