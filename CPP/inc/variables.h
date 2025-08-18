/*
 * variables.h
 *
 *  Created on: Sep 16, 2024
 *      Author: Gil Vargas
 */

#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_

#define DSHOT_6_BIT (0x52)
#define DSHOT_3_BIT (0x9C)

#define DIGITAL_CMD_BEEP1 (0x01)
#define DIGITAL_CMD_BEEP2 (0x02)
#define DIGITAL_CMD_BEEP3 (0x03)
#define DIGITAL_CMD_BEEP4 (0x04)
#define DIGITAL_CMD_BEEP5 (0x05)
#define DIGITAL_CMD_SPIN_DIRECTION_1 	(0x07)
#define DIGITAL_CMD_SPIN_DIRECTION_2 	(0x08)
#define DIGITAL_CMD_SAVE_SETTINGS 		(0x0C)
#define DIGITAL_CMD_EDT_ENABLED 		(0x0D)
#define DIGITAL_CMD_EDT_DISABLED 		(0x0E)
#define DIGITAL_CMD_SPIN_DIRECTION_NORMAL_TURTLE 	(0x14)
#define DIGITAL_CMD_SPIN_DIRECTION_REVERSED_TURTLE 	(0x15)
#define DSHOT_EDT_DISABLE (0xEFF)
#define DSHOT_EDT_ENABLE  (0xE00)
#define DSHOT_EDT_STATUS  (0xE00)

#define MAX_NO_SIGNAL_COUNT (0x1388)
#define COMMUTATION_STEPS 	(0x06)
#define CLK_FREQUENCY 		(0x3D09000)
#define MAX_THROTTLE_VALUE 	(0x7FF)
#define MAX_NUMBER_OF_COMMANDS (0x2F)

#define DSHOT_MSB 	(0x05)
#define RPM_659 	(0x32C8)
#define RPM_612 	(0x36B0)
#define BLANKING_1_5MS (0x60)

#define ADC_TEMP hadc1
#define ADC_TEMP_DMA hdma_adc1

#define ADC_TEMP_REF_3V (0xCD0) // no need to read real ref.

#define ZC_COMP COMP2

#define ZC_COMP_PHASE_A (COMP_CSR_INMSEL_0 | COMP_CSR_INMSEL_1 | COMP_CSR_INMSEL_2)
#define ZC_COMP_PHASE_B (COMP_CSR_INMSEL_1 | COMP_CSR_INMSEL_2)
#define ZC_COMP_PHASE_C (COMP_CSR_INMSEL_3)

#define PWM_A_DIER_H TIM_DIER_CC3DE
#define PWM_B_DIER_H TIM_DIER_CC2DE
#define PWM_C_DIER_H TIM_DIER_CC1DE

#define PWM_A_CCER_H TIM_CCER_CC3E
#define PWM_B_CCER_H TIM_CCER_CC2E
#define PWM_C_CCER_H TIM_CCER_CC1E

#define PWM_BLANKING TIM_CCER_CC5E

#define PWM_A_CCER_LN TIM_CCER_CC3NE
#define PWM_B_CCER_LN TIM_CCER_CC2NE
#define PWM_C_CCER_LN TIM_CCER_CC1NE

#define PWM_A_CCER_L TIM_CCER_CC3E
#define PWM_B_CCER_L TIM_CCER_CC2E
#define PWM_C_CCER_L TIM_CCER_CC1E

#define MOTOR_TIMER TIM1
#define CCR_A  MOTOR_TIMER->CCR3
#define CCR_B  MOTOR_TIMER->CCR2
#define CCR_C  MOTOR_TIMER->CCR1

#define E_RPM_TIMER htim6

#define COMP_VALIDATION_CYCLES (0x08)

#define ADC_CAL_VALUE (0.000806);

#define DSHOT_FILTER  TIM_CCMR1_IC1F_2 //TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 // IC1F0 - IC1F1.

#define DIER_OFF ~(PWM_A_DIER_H | PWM_B_DIER_H | PWM_C_DIER_H)
#define CCER_OFF ~(PWM_A_CCER_L | PWM_B_CCER_L | PWM_C_CCER_L | PWM_A_CCER_LN |PWM_B_CCER_LN | PWM_C_CCER_LN)

typedef struct dshot_signal_s{
	uint8_t protocol_detected 				= 0;
	uint8_t protocol_validation_counter 	= 0;
	uint8_t protocol_valid_counter_value 	= 5;
	uint16_t protocol_previous 				= 0;
	uint16_t protocol 	= 0;
	uint8_t beeped 		= 1;

	uint16_t protocol_dshot600_bi 	= 0x06;
	uint16_t protocol_dshot600 		= 0x06; // Same as bi.
	uint16_t protocol_dshot300_bi 	= 0x0D;
	uint16_t protocol_dshot300 		= 0x0C;
	uint8_t speed_pwm = 1;

	uint8_t protocol_dshot_bi 				= 0;
	uint8_t protocol_dshot_bi_counter 		= 0;
	uint8_t protocol_dshot_bi_valid_counter = 4;
	uint16_t timing 		= 0;
	uint16_t fc_throttle 	= 0;
	uint16_t motor_beep_frequency 	= 0;
	uint16_t motor_beep_delay 		= 0;
	uint8_t motor_master_direction 	= 0;
	uint8_t motor_direction 		= 0;
	uint8_t protocol_edt_enable 	= true;
	uint8_t protocol_edt_update 	= false;
} dshot_signal_t;

uint16_t tone_buffer[10][2] = {
		{1244,200},
		{1000,220},
		{980,400},
		{1861,180},
		{1561,160},
		{1000,210},
		{1000,300},
		{1261,200},
		{1000,500},
		{1261,200}
};
extern "C" COMP_HandleTypeDef hcomp2;

extern "C" TIM_HandleTypeDef htim1; // PWM Drive lines.
extern "C" TIM_HandleTypeDef htim3; // DMA Signal.

extern "C" TIM_HandleTypeDef htim7; // General timer for processing.

extern "C" TIM_HandleTypeDef htim6; // eRPM.
extern "C" TIM_HandleTypeDef htim16; // Startup and commutation.
extern "C" TIM_HandleTypeDef htim17; // Bumps motor at startup.
extern "C" DMA_HandleTypeDef hdma_tim3_ch1;
extern "C" DMA_HandleTypeDef hdma_tim3_ch2;

extern "C" DMA_HandleTypeDef hdma_tim1_ch3;
extern "C" DMA_HandleTypeDef hdma_tim1_ch2;
extern "C" DMA_HandleTypeDef hdma_tim1_ch1;

extern ADC_HandleTypeDef hadc1;	// battery voltages.
extern DMA_HandleTypeDef hdma_adc1;

volatile uint16_t dma_signal[32] = {0};
volatile uint16_t dshot_bits[16] = {0};

volatile uint16_t dma_tlm_buffer[22] = { 160, 160, 0, 160, 0, 160, 160, 0, 160, 0, 160, 0, 160, 0, 160, 0, 160, 160, 160, 0, 0};
volatile uint16_t dma_tlm_buffer2[22] = {80, 80, 0, 0, 80, 0, 0, 80, 80, 0, 80, 0, 0, 0, 80, 0, 80, 0, 80, 0, 0, 0};
volatile uint8_t dma_tlm_buffer_ready[2] = {0,0};

volatile uint16_t throttle = 0;
volatile uint8_t allow_commutation = 0;
volatile uint16_t beeping = 0;

volatile uint32_t commutation_counter = 0;
volatile uint16_t dma_frequency_value[4] = {0};

volatile uint16_t commutation_ticks = 0;

volatile uint8_t commutation_step = 0;
volatile uint8_t power_step_current = 0;

volatile uint8_t phase_rising[2][6] = { {1,0,1,0,1,0}, {0,1,0,1,0,1}};
volatile uint16_t electrical_rpm_in_us = 0;
uint8_t telemetry = 0;

volatile uint32_t no_signal_counter = 0;

volatile uint8_t commutated = 0;

enum ESC_STATUS{
	init = 0,
	running,
	crashedDetected,
	stopped,
	startup
} volatile esc_status;

__IO uint32_t *ccrOff[6] = { //
		&CCR_B, // B
		&CCR_A, // A
		&CCR_C, // C
		&CCR_B, // B
		&CCR_A, // A
		&CCR_C, // C
};
__IO uint32_t *ccrLow[6] = { //
		&CCR_C, // C
		&CCR_C, // C
		&CCR_A, // A
		&CCR_A, // A
		&CCR_B, // B
		&CCR_B, // B
};

const uint32_t pwmDIER[6] = { //
		uint32_t(PWM_A_DIER_H),//
		uint32_t(PWM_B_DIER_H),//
		uint32_t(PWM_B_DIER_H),//
		uint32_t(PWM_C_DIER_H),//
		uint32_t(PWM_C_DIER_H),//
		uint32_t(PWM_A_DIER_H),//
};
const uint32_t pwmCCER[6] = { //
		uint32_t(PWM_A_CCER_H | PWM_A_CCER_LN | PWM_C_CCER_LN | PWM_C_CCER_H | PWM_BLANKING),//
		uint32_t(PWM_B_CCER_H | PWM_B_CCER_LN | PWM_C_CCER_LN | PWM_C_CCER_H | PWM_BLANKING),//
		uint32_t(PWM_B_CCER_H | PWM_B_CCER_LN | PWM_A_CCER_LN | PWM_A_CCER_H | PWM_BLANKING),//
		uint32_t(PWM_C_CCER_H | PWM_C_CCER_LN | PWM_A_CCER_LN | PWM_A_CCER_H | PWM_BLANKING),//
		uint32_t(PWM_C_CCER_H | PWM_C_CCER_LN | PWM_B_CCER_LN | PWM_B_CCER_H | PWM_BLANKING),//
		uint32_t(PWM_A_CCER_H | PWM_A_CCER_LN | PWM_B_CCER_LN | PWM_B_CCER_H | PWM_BLANKING),//
		};

const uint16_t comp2Off[6] = {// COMP2_CSR INM_SEL
		 ZC_COMP_PHASE_B,//0b0110<<4, // B
		 ZC_COMP_PHASE_A,//0b0111<<4, // A
		 ZC_COMP_PHASE_C,//0b1000<<4, // C
		 ZC_COMP_PHASE_B,//0b0110<<4, // B
		 ZC_COMP_PHASE_A,//0b0111<<4, // A
		 ZC_COMP_PHASE_C,//0b1000<<4, // C
};

#endif /* INC_VARIABLES_H_ */
