/*
 * eeprom.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Gil Vargas
 */
#include "main.h"
#include "main_esc.h"
#include "variables.h"
#include "string.h"

#include "stm32g0xx_hal_adc.h"

uint16_t to_big_endiean(uint16_t value)
{
	return  (uint16_t)( ((value>>8) & 0xff) | ((value<<8) & 0xffff));
}
void TIM_DMADelayPulseCplt2(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htim->hdma[TIM_DMA_ID_CC1])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
  {
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
    }
  }

  else
  {
    /* nothing to do */
  }

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
  htim->PWM_PulseFinishedCallback(htim);
#else
  HAL_TIM_PWM_PulseFinishedCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

  htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(TIM3 == htim->Instance)
	{
		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			signal_heartbeat = true;

			hdma_tim3_ch1.Instance->CCR &= ~(1);

			if( pdshot_settings->protocol == pdshot_settings->protocol_dshot600_bi)
				TIM3->CCR4 = 330;// TODO: Magic value
			else
			{
				TIM3->CCR4 = 480;// TODO: Magic value
			}

			// Que next TLM.
			TIM3->EGR |= 1;
			TIM3->DIER |= TIM_DIER_CC4IE;
			TIM3->SR = ~TIM_SR_CC4IF;

			protocol_timer_psc = TIM3->PSC;
			pdshot_settings->timing = (dma_signal[31] - dma_signal[0]);



			if( pdshot_settings->protocol_detected )
			{

				for( uint8_t i = 0; i < 32; i += 2)
					dshot_bits[i>>1] = (dma_signal[i+1] - dma_signal[i])>>DSHOT_MSB;

				volatile uint16_t value = 0;
				for( uint8_t i = 0 ; i < 12; i++)
					value |= (dshot_bits[i]<<(11-i));

				uint8_t calculated_crc  = 0;
				uint8_t telemetryBit = dshot_bits[11];

				uint8_t received_crc = (dshot_bits[12]<<3 | dshot_bits[13]<<2 | dshot_bits[14]<<1 | dshot_bits[15] );
				if( pdshot_settings->protocol_dshot_bi)
				{
					calculated_crc  = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
				}
				else
				{
					calculated_crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
				}
				value >>= 1;

				if( received_crc != calculated_crc )
				{
					if( pdshot_settings->protocol_dshot_bi)
					{
						calculated_crc  = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
					}
					else
					{
						calculated_crc  = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
					}

					if( received_crc == calculated_crc)
					{
						if(pdshot_settings->protocol_dshot_bi_counter++ >= pdshot_settings->protocol_dshot_bi_valid_counter)
							pdshot_settings->protocol_dshot_bi = !pdshot_settings->protocol_dshot_bi;
					}
					else
					{
						pdshot_settings->protocol_dshot_bi_counter = 0;
					}

				}
				else if( received_crc == calculated_crc )
				{
					pdshot_settings->protocol_dshot_bi_counter = 0;

					if( telemetryBit == 1 && pdshot_settings->fc_throttle == 0 && value <= MAX_NUMBER_OF_COMMANDS)
					{
						process_dshot_command(value, pdshot_settings);

					}else if( value > MAX_NUMBER_OF_COMMANDS && value <= MAX_THROTTLE_VALUE && beeping == 0 )
					{
						process_throttle_input((uint16_t*)&value,pdshot_settings);
					}else if( beeping == 0)
					{
						pdshot_settings->fc_throttle = 0;

						set_motor_throttle(pdshot_settings->fc_throttle);
					}
					else
					{

					}
				}

				else
				{

				}

				if( pdshot_settings->fc_throttle < MAX_NUMBER_OF_COMMANDS)
					electrical_rpm_in_us = 0xffff;
			}

			telemetry = 1;

		}
		else
		{
			// TODO: nothing.
		}
	}

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM3 == htim->Instance)
	{

		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{

			set_receive_signal_from_fc();

		}

	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM3 == htim->Instance)
	{
		if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{



			TIM3->DIER &= ~(TIM_DIER_CC4IE); // Diasble OC interrupt for ch4;

			TIM3->CR1 &= ~(TIM_CR1_CEN);

			if( pdshot_settings->protocol_dshot_bi )
			{
				set_send_rpm_to_fc(pdshot_settings);
			}
			else
			{
				set_receive_signal_from_fc();
			}
		}

	}
	else if( TIM17 == htim->Instance)
	{
		static uint16_t crash_detected_counter = 0;
		const uint16_t MIN_THROTTLE_FOR_CRASH_DETECTION = 1000;
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{

			if( esc_status == ESC_STATUS::running || esc_status == ESC_STATUS::startup)
				TIM17->CNT = 0;
			else
				return;


			if (throttle > MAX_NUMBER_OF_COMMANDS && allow_commutation)
			{

				/*
				 * Bump motor if not spinning.
				 * this commutates motor if timer expired and throttle > 0.
				 *
				 *	For a crash to be consider a true event the esc needs to be in a state of running, throttle needs to be high for a number
				 *	of cycles.
				 *
				 */
				if( peeprom_settings->crash_detection > 0 && throttle > MIN_THROTTLE_FOR_CRASH_DETECTION && esc_status == ESC_STATUS::running)
				{
					if( crash_detected_counter++ > peeprom_settings->crash_detection )
					{
						pdshot_settings->fc_throttle = 0;
						set_motor_throttle(0);

						disconnect_motor_phases();
						esc_status = ESC_STATUS::stopped;

					}
				}else
				{
					crash_detected_counter = 0;
				}
				if( motor_not_spinning == 0 && throttle > 300 )// TODO: Magic value needs config option.
				{
//						esc_status = ESC_STATUS::crashedDetected;
						set_motor_throttle(0);
				}
				commutation_counter = 0; // Reset on bump.
				motor_not_spinning = 1;

				commutate(pdshot_settings);
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( TIM17 == htim->Instance)
	{
		if( esc_status == ESC_STATUS::crashedDetected)
			esc_status = ESC_STATUS::running;
	}
}

void process_dshot_command(uint8_t cmd, dshot_signal_t *pdshot)
{
	static uint8_t command_counter[48] = {0};
	const uint8_t VALID_CMD_TIMES = 6;

	const uint8_t CMD_RESET = 0;
	command_counter[cmd] += 1;

	switch (cmd)
	{
	case DIGITAL_CMD_BEEP1:
	case DIGITAL_CMD_BEEP2:
		beeping = 260;
		break;
	case DIGITAL_CMD_BEEP3:
	case DIGITAL_CMD_BEEP4:
		beeping = 280;
		break;
	case DIGITAL_CMD_BEEP5:
		beeping = 1020;
		break;

	case DIGITAL_CMD_SPIN_DIRECTION_1:
		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;
			pdshot->motor_master_direction = 0;
			eeprom_settings.motor_direction_master = pdshot->motor_master_direction;
			write_memory((uint8_t*) &eeprom_settings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);

			pdshot->motor_direction = pdshot->motor_master_direction;
		}
		break;
	case DIGITAL_CMD_SPIN_DIRECTION_2:
		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;

			pdshot->motor_master_direction = 1;
			eeprom_settings.motor_direction_master = pdshot->motor_master_direction;

			write_memory((uint8_t*) &eeprom_settings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);
			pdshot->motor_direction = pdshot->motor_master_direction;
		}
		break;
	case DIGITAL_CMD_SAVE_SETTINGS:
		//Save something;
		break;
	case DIGITAL_CMD_EDT_ENABLED:

		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;
			pdshot->protocol_edt_enable = true;
			pdshot->protocol_edt_update = true;

		}
		break;
	case DIGITAL_CMD_EDT_DISABLED:

		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;
			pdshot->protocol_edt_enable = false;
			pdshot->protocol_edt_update = true;

		}
		break;
	case DIGITAL_CMD_SPIN_DIRECTION_NORMAL_TURTLE:
		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;
			pdshot->motor_direction = pdshot->motor_master_direction;
		}
		break;
	case DIGITAL_CMD_SPIN_DIRECTION_REVERSED_TURTLE:
		if (command_counter[cmd] >= VALID_CMD_TIMES) {
			command_counter[cmd] = CMD_RESET;
			pdshot->motor_direction = pdshot->motor_master_direction == 0 ? 1 : 0;
		}
		break;

	default:
		break;
	}

	for (uint8_t index = 0; index <= MAX_NUMBER_OF_COMMANDS; index++) {
		if (index != cmd) {
			command_counter[index] = CMD_RESET;
		}
	}
}

void process_throttle_input(uint16_t *throttle_value,dshot_signal_t* pdshot)
{
	no_signal_counter = 0;
	pdshot->fc_throttle = *throttle_value;

	volatile uint16_t rampup = to_big_endiean( peeprom_settings->rampup);
	volatile uint16_t turtle_rampup = to_big_endiean( peeprom_settings->turtle_rampup);

	if (pdshot->motor_direction != pdshot->motor_master_direction && pdshot->fc_throttle > MAX_NUMBER_OF_COMMANDS)
	{
		if ( (pdshot->fc_throttle - throttle) > turtle_rampup)
		{
			if (throttle == 0)
				pdshot->fc_throttle = turtle_rampup + MAX_NUMBER_OF_COMMANDS;
			else
				pdshot->fc_throttle = throttle + turtle_rampup;

		}
	} else if ( (pdshot->fc_throttle - throttle) > rampup && pdshot->fc_throttle > MAX_NUMBER_OF_COMMANDS)
	{
		if (throttle == 0)
			pdshot->fc_throttle = rampup + MAX_NUMBER_OF_COMMANDS;
		else
			pdshot->fc_throttle = throttle + rampup;

	} else {
	}
	allow_commutation = 1;

	if (motor_not_spinning && electrical_rpm_in_us < 4500) { // TODO: Magic numbers
		if (commutation_counter > 400) {
			commutation_counter = 2500; // TODO: magic numbers.
			motor_not_spinning = 0;
		}

	}

	if (esc_status == ESC_STATUS::running)
	{
		set_motor_throttle(pdshot->fc_throttle);
	}
}

void set_motor_throttle_beep( uint16_t duty)
{
	static uint16_t remainder = 0;

	duty = duty < MAX_NUMBER_OF_COMMANDS ? 0 : duty;

	if( duty == 0)
	{
		disconnect_motor_phases();
	}
#ifndef PMW_124Hz
	remainder = duty & 1;
	dma_frequency_value[0] = duty>>1 | remainder;
	dma_frequency_value[1] = duty>>1;
#else
	remainder = duty & 3;
	const uint32_t value = duty>>2;
	dma_frequency_value[0] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[1] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[2] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[3] = value + (remainder > 0 ? remainder-- : 0);
#endif
}

void set_motor_throttle( uint16_t duty)
{
	static uint16_t remainder = 0;

	duty = duty < MAX_NUMBER_OF_COMMANDS ? 0 : duty;
	throttle = duty;
	if( duty == 0 || (esc_status != ESC_STATUS::running && esc_status != ESC_STATUS::startup))
	{
		if( disableFetsCounter++ >= 1200) // TODO: magic number.s
		{ // TODO: BrennanG requested hard stop but no locked motors.

			disconnect_motor_phases();
			disableFetsCounter = 2000; // TODO: magic numbers.
		}

		duty = 0;

		commutation_counter = 0;
		motor_not_spinning = 1;
		allow_commutation = 0;

		esc_status = ESC_STATUS::stopped;
	}else
	{
		disableFetsCounter = 0;
//		debug[2]++;
	}
#ifndef PMW_124Hz
	remainder = duty & 1;
	dma_frequency_value[0] = duty>>1 | remainder;
	dma_frequency_value[1] = duty>>1;
#else
	remainder = duty & 3;
	const uint32_t value = duty>>2;
	dma_frequency_value[0] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[1] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[2] = value + (remainder > 0 ? remainder-- : 0);
	dma_frequency_value[3] = value + (remainder > 0 ? remainder-- : 0);
#endif
}

void set_receive_signal_from_fc(void)
{
	if (dma_tlm_buffer_ready[0] == 2)
		dma_tlm_buffer_ready[0] = 0;
	hdma_tim3_ch1.Instance->CCR &= ~(DMA_CCR_EN);
	TIM3->DIER &= ~(TIM_DIER_CC1DE); // Disable DMA transfer interrupt for channel 1;

	hdma_tim3_ch1.Instance->CCR &= ~(DMA_CCR_EN); // Disable DMA. EN.

	TIM3->CR1 &= ~(TIM_CR1_CEN);
	TIM3->CCR1 = 0;

	TIM3->CCER = (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC4E);

	TIM3->CCMR1 = (DSHOT_FILTER | TIM_CCMR1_CC1S_0); // Filter.

	TIM3->CCMR1 &= ~(0x03 << 2); // IC1PSC // TODO: magic number.

	hdma_tim3_ch1.Instance->CMAR = (uint32_t) dma_signal;
	hdma_tim3_ch1.Instance->CNDTR = 32;

	static uint32_t ccr = ( DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0); // ccr;
	hdma_tim3_ch1.Instance->CCR = ccr;

	TIM3->CR1 |= (1 << 0);

	TIM3->CNT = 0;

	if (pdshot_settings->protocol == pdshot_settings->protocol_dshot600_bi)
		TIM3->PSC = 1;

	else {
		TIM3->PSC = 3; //1;

	}
	TIM3->ARR = 0xffff;

	TIM3->EGR |= 1;

	hdma_tim3_ch1.Instance->CCR |= (DMA_CCR_PL);
	hdma_tim3_ch1.XferCpltCallback = &TIM_DMACaptureCplt;

	TIM3->DIER |= (TIM_DIER_CC1DE); // Enable DMA trans interrupt.
	hdma_tim3_ch1.Instance->CCR |= (DMA_CCR_EN | DMA_CCR_TCIE); // enable dma and trans complete.

	TIM3->CCER |= TIM_CCER_CC1E;
	return;
}

void set_send_rpm_to_fc(dshot_signal_t *pdshot)
{
	TIM3->CCER = (TIM_CCER_CC1P | TIM_CCER_CC4E);
	TIM3->CCMR1 = ( TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

	hdma_tim3_ch1.XferCpltCallback = &TIM_DMADelayPulseCplt2;
	TIM3->DIER |= (TIM_DIER_CC1DE);

	{

		dma_tlm_buffer_ready[0] = 2;
		hdma_tim3_ch1.Instance->CMAR = (uint32_t) dma_tlm_buffer;

	}

	static uint32_t ccr = (DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0
			| DMA_CCR_MSIZE_0); // ccr;
	hdma_tim3_ch1.Instance->CCR = ccr;

	TIM3->PSC = 0;
	uint16_t dataBit =
			pdshot->protocol == pdshot->protocol_dshot600_bi ?
					DSHOT_6_BIT : DSHOT_3_BIT;

	TIM3->ARR = dataBit;

	TIM3->EGR |= 1;

	TIM3->CCER |= TIM_CCER_CC1E;
	hdma_tim3_ch1.Instance->CNDTR = 22;

	hdma_tim3_ch1.Instance->CCR |= (DMA_CCR_EN | DMA_CCR_TCIE);

	TIM3->CR1 |= (TIM_CR1_CEN);

	return;
}

void convert_dshot_frame_to_dma_buffer(uint32_t value,dshot_signal_t *pdshot)
{

	uint32_t calculated_crc = 0;
	uint32_t gcr_value =  0;
	uint32_t dma_value = 0;

	static const uint8_t nibbles[16] = //
	{
			0x19	,0x1B	,0x12	,0x13
			,0x1D	,0x15	,0x16	,0x17
			,0x1A	,0x09	,0x0A	,0x0B
			,0x1E	,0x0D	,0x0E	,0x0F
	};

	calculated_crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
	value = value<<4 | calculated_crc;

	gcr_value =	nibbles[(value>>12 	& 0x0f)]<<15
			| nibbles[(value>>8 	& 0x0f)]<<10
			| nibbles[(value>>4 	& 0x0f)]<<5
			| nibbles[(value		& 0x0f)];


	for(uint8_t i = 19; i != 0xff ; i-- )
	{
		dma_value |= ( dma_value &  1<<(i+1) )>>1; // Get previous bit.

		if( gcr_value & 1<<i )
			dma_value ^=  (1<<i); // Flip if new bit 1.
	}

	if(dma_tlm_buffer_ready[0] != 0)
		return;

	volatile uint16_t dma_bit = pdshot->protocol == pdshot->protocol_dshot600_bi ? DSHOT_6_BIT : DSHOT_3_BIT;
	if(dma_tlm_buffer_ready[0] == 0)
	{
		dma_tlm_buffer[21] = 0;
		for( uint8_t i = 20; i != 0xff; i--)
		{
			dma_tlm_buffer[20-i] = ( dma_value & 1<<i) ? 0 : dma_bit;
		}
		dma_tlm_buffer_ready[0] = 1;
	}


}

uint32_t pack_dshot_frame(uint32_t value)
{

	const uint8_t EXPO_POS = 0x09;
	uint32_t expo = 0;

	while( (value>>EXPO_POS) )
	{
		expo++;
		value >>=1;
	}

	return value | (expo<<EXPO_POS);
}

void motor_beep(const uint16_t freq, uint16_t *delay)
{
	if( throttle > 0 || delay == 0)
		return;
#ifdef LED_ENABLED
	LED_GPIO_Port->BSRR = LED_Pin;
#endif
	allow_commutation = 0;
	uint16_t psc = MOTOR_TIMER->PSC;
	uint16_t arr = MOTOR_TIMER->ARR;
	MOTOR_TIMER->PSC = 1;
	MOTOR_TIMER->ARR = CLK_FREQUENCY/freq;
	MOTOR_TIMER->CNT = 0;
	MOTOR_TIMER->EGR |= 1;


	set_motor_throttle_beep(250);// TODO: Magic number
	commutate(pdshot_settings);

	HAL_Delay(delay[0]>>1);
#ifdef LED_ENABLED
	LED_GPIO_Port->BRR = LED_Pin;
#endif
	HAL_Delay(delay[0]>>1);

	set_motor_throttle_beep(0);
	MOTOR_TIMER->PSC = psc;
	MOTOR_TIMER->ARR = arr;
	MOTOR_TIMER->EGR |= TIM_EGR_UG;// 1;

	delay[0] = 0;
}

void commutate(dshot_signal_t *pdshot)
{
	power_step_current++;
	power_step_current %= COMMUTATION_STEPS;


	if( pdshot->motor_direction )
		commutation_step = (COMMUTATION_STEPS-1) - power_step_current;
	else
		commutation_step = power_step_current; // forward.

	/*
	 0110: PB3 -> ZC_B
	 0111: PB7 -> ZC_A
	 1000: PA2 -> ZC_C
	 Step 0  1   2  3  4  5      0  1  2  3  4  5
	 High A  B   B  C  C  A      A  B  B  C  C  A
	 Off  B  A   C  B  A  C      B  A  C  B  A  C
	 Low  C  C   A  A  B  B      C  C  A  A  B  B
	 */


	ZC_COMP->CSR &= ~(COMP_CSR_EN | COMP_CSR_INMSEL | COMP_CSR_POLARITY);
	MOTOR_TIMER->CNT = 1000; // TODO: Magic number, facilitates transition.

	MOTOR_TIMER->DIER = (pwmDIER[commutation_step]); // enable DMA on active high side.
	MOTOR_TIMER->CCER = (pwmCCER[commutation_step]); // enable PWM on ALL ATIVE lines high and low.
	*ccrOff[commutation_step] = 0; // remove leftover values.
	*ccrLow[commutation_step] = 0;

	ZC_COMP->CSR |= (comp2Off[commutation_step]);// enable comparator line on floating phase.
	if(phase_rising[pdshot->motor_direction][commutation_step] )
	{
		ZC_COMP->CSR |= COMP_CSR_POLARITY;// (1<< 15); // invert comparator output to always output high.
	}

	ZC_COMP->CSR |=  COMP_CSR_EN;// (1); // Enable comp2.

	EXTI->RPR1 |= (EXTI_RPR1_RPIF18);// 1<<18); // Clear pending comp interrupts.
	EXTI->FPR1 |= (EXTI_FPR1_FPIF18);// 1<<18); // Clear pending comp interrupts.


	if(no_signal_counter++ > MAX_NO_SIGNAL_COUNT)

	{ // kill throttle if flight conltroller signal stops.
		set_motor_throttle(0);
	}

	commutated = 1; // TODO: add better explanation of what this does??
	TIM17->CNT = 0; // Reset bump timer

	commutation_counter++;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{

	if( allow_commutation == 0)
		return;

	const uint16_t timer_7_cnt = (uint16_t)TIM7->CNT;

	if( timer_7_cnt < commutation_ticks)
	{
		return;
	}

	for( int i = 0; i < COMP_VALIDATION_CYCLES; i++) // 4 seems to work find, needs real flight test.
	{ // Abort commutation if Comparator value changes to zero.

		if( (ZC_COMP->CSR & COMP_CSR_VALUE) == 0)
			return;
	}

	commutation_ticks = TIM7->CNT;
	TIM7->CNT = 0; //
	if( commutation_step == 0)
	{
		electrical_rpm_in_us = E_RPM_TIMER.Instance->CNT;
		E_RPM_TIMER.Instance->CNT = 0;
	}

	if ( esc_status == ESC_STATUS::stopped && electrical_rpm_in_us < RPM_612)
	{
		if(commutation_counter++ > 6)
		{
			esc_status = ESC_STATUS::running;
		}
	}else
	{
		commutation_counter = 0;
	}

	commutate(pdshot_settings);

	commutation_ticks >>= 1;
	commutation_ticks += peeprom_settings->commutation_delay;

	EXTI->RPR1 |= EXTI_RPR1_RPIF18;// 18); // Clear pending comp interrupts.

}

void reset_device()
{
	uint32_t now = HAL_GetTick();
	static uint32_t time_stamp = now;
	const uint32_t TIME_OUT = 100;

	if( (now - time_stamp) > TIME_OUT && time_stamp > 0)
	{
		if( signal_heartbeat)
		{
			signal_heartbeat = false;
			time_stamp = now;
		}
		else
		{
			NVIC_SystemReset();
		}
	}

}
void detect_protocol(dshot_signal_t *pdshot)
{
	if (!pdshot->protocol_detected)
	{
		uint32_t dshot_cc = (pdshot->timing << protocol_timer_psc) >> 9;

		pdshot->protocol_validation_counter++;

		if (dshot_cc == pdshot->protocol_dshot600_bi || dshot_cc == pdshot->protocol_dshot600)
		{
			if (pdshot->protocol_previous != dshot_cc)
			{
				pdshot->protocol_previous = dshot_cc;
				pdshot->protocol_validation_counter = 0;
			}

		}
		else if (dshot_cc == pdshot->protocol_dshot300_bi || dshot_cc == pdshot->protocol_dshot300)
		{
			if (pdshot->protocol_previous != dshot_cc)
			{
				pdshot->protocol_previous = dshot_cc;
				pdshot->protocol_validation_counter = 0;
			}
		}
		else
		{
			pdshot->protocol_validation_counter = 0;
		}

		if( pdshot->protocol_validation_counter >= pdshot->protocol_valid_counter_value)
		{
			pdshot->protocol =  pdshot->protocol_previous;
			pdshot->protocol_detected = 1;
			pdshot->beeped = 0;
		}

	}
	else
	{


	}
}
void motor_beep(dshot_signal_t *pdshot)
{
	if( !pdshot->beeped && motor_not_spinning && pdshot->fc_throttle == 0)
	{
		pdshot->beeped = 1;
		beeping = 1;

		for( int i = 3; i < 6; i++)
		{
			motor_beep(tone_buffer[i][0],(uint16_t*)&tone_buffer[i][1]);
		}
		beeping = 0;
		HAL_Delay(500);
		while(pdshot->fc_throttle > 50)
			HAL_Delay(500);

		esc_status = ESC_STATUS::running;
	}else{
		motor_beep(1244, (uint16_t*)&beeping);
	}
}
void disconnect_motor_phases(void)
{
	MOTOR_TIMER->DIER = 0;
	MOTOR_TIMER->CCER = 0;
}
void motor_startup(dshot_signal_t *pdshot)
 {
	static uint32_t timestamp_startup = 0;
	static uint32_t timestamp_spinning = 0;
	static uint32_t now = HAL_GetTick();
	now = HAL_GetTick();
	if (pdshot->fc_throttle > 70 && esc_status == ESC_STATUS::stopped && (now - timestamp_spinning) > 250)
	{
		// This starts motor spin.
		esc_status = ESC_STATUS::startup;
		timestamp_startup = now;
		timestamp_spinning = now;
		commutation_counter = 0;
		set_motor_throttle(300);

	} else if (pdshot->fc_throttle > MAX_NUMBER_OF_COMMANDS && esc_status == ESC_STATUS::startup && (now - timestamp_startup) > 150)
	{

		pdshot->fc_throttle = 0;
		set_motor_throttle(0);

		disconnect_motor_phases();

		timestamp_spinning = now;
		if ((now - timestamp_spinning) > 200)
		{
			esc_status = ESC_STATUS::stopped;
			timestamp_spinning = now;
		}

	} else if (electrical_rpm_in_us > RPM_659 && esc_status == ESC_STATUS::running)
	{

		esc_status = ESC_STATUS::stopped;
		set_motor_throttle(0);
	}
}

uint32_t encode_temperature(uint32_t temp)
{
	return (1<<9) | (temp & 0xff);
}

void launch_adc(void)
{
	ADC_TEMP.Instance->CR |= ADC_CR_ADSTART;
}

void dshot_rpm(dshot_signal_t *pdshot)
{
	volatile uint16_t electrical_rpm = 0;
	const uint32_t ONE_SECOND = 1000;
	const uint32_t now = HAL_GetTick();
	static volatile uint32_t timestamp_temp_sent = now + 333;
	static volatile uint32_t timestamp_temp_status = now;
	electrical_rpm = electrical_rpm_in_us;
	if (dma_tlm_buffer_ready[0] == 0) {

		volatile uint32_t electrical_rpm_encoded = pack_dshot_frame( (const uint32_t) electrical_rpm);

		if( pdshot->protocol_edt_update == true)
		{
			if( pdshot->protocol_edt_enable)
				electrical_rpm_encoded = DSHOT_EDT_ENABLE;
			else
				electrical_rpm_encoded = DSHOT_EDT_DISABLE;

			pdshot->protocol_edt_update = false;
		}else if( pdshot->protocol_edt_enable)
		{
			if( (now - timestamp_temp_status) >= ONE_SECOND)
			{
				timestamp_temp_status = now;
				timestamp_temp_sent = now + 333;
				electrical_rpm_encoded = DSHOT_EDT_STATUS;
			}
			if( (now - timestamp_temp_sent) > ONE_SECOND)
			{
				timestamp_temp_sent = now;

				uint32_t adc_value = ADC_TEMP.Instance->DR;
				uint32_t temperature = __HAL_ADC_CALC_TEMPERATURE(ADC_TEMP_REF_3V, adc_value, ADC_RESOLUTION_12B);

				electrical_rpm_encoded = encode_temperature(temperature);

				launch_adc();
			}
		}


		convert_dshot_frame_to_dma_buffer(electrical_rpm_encoded,pdshot);
	}
}



void init_eeprom(eeprom_settings_t *psettings, dshot_signal_t *pdshot)
{
	eeprom_settings_t settings_temp;
	const uint8_t MAX_COMMUTATION_DELAY_3MS = 192;
	const uint16_t MAX_STARTUP_THROTTLE = 500;
	const uint8_t MAX_TURTLE_RAMPUP = 250;
	const uint8_t TURTLE_RAMPUP_DEFAULT = 50;
	const uint16_t MAX_RAMPUP = 2048;
	const uint8_t NORMAL_DIRECTION = 1;


	psettings->startup_throttle = to_big_endiean(psettings->startup_throttle);
	psettings->rampup = to_big_endiean(psettings->rampup);

	eeprom_settings_t readmem;
	read_memory((uint8_t*)&settings_temp, sizeof(eeprom_settings_t), EEPROM_ADDRESS);

	if( settings_temp.version == 0xff || settings_temp.sub_version == 0xff)
	{
		settings_temp.version = 0;
		settings_temp.sub_version = 0;
	}

	if( psettings->version > settings_temp.version)
	{
		write_memory((uint8_t*)psettings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);
	}
	else if(psettings->version >= settings_temp.version && psettings->sub_version > settings_temp.sub_version )
	{
		write_memory((uint8_t*)psettings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);
	}

	read_memory((uint8_t*)psettings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);

	psettings->commutation_delay =
			psettings->commutation_delay > MAX_COMMUTATION_DELAY_3MS ?
					MAX_COMMUTATION_DELAY_3MS : psettings->commutation_delay;

	psettings->startup_throttle =
			to_big_endiean( psettings->startup_throttle) > MAX_STARTUP_THROTTLE ?
					to_big_endiean( MAX_STARTUP_THROTTLE) : psettings->startup_throttle;

	psettings->turtle_rampup =
			psettings->turtle_rampup > MAX_TURTLE_RAMPUP ?
					TURTLE_RAMPUP_DEFAULT : psettings->turtle_rampup;

	psettings->rampup =
			to_big_endiean( psettings->rampup) > MAX_RAMPUP ? to_big_endiean( MAX_RAMPUP) : psettings->rampup;

	psettings->motor_direction_master =
			psettings->motor_direction_master > NORMAL_DIRECTION ?
					NORMAL_DIRECTION : psettings->motor_direction_master;

	psettings->motor_direction =
			psettings->motor_direction > NORMAL_DIRECTION ?
					NORMAL_DIRECTION : psettings->motor_direction;

	psettings->crash_detection =
			psettings->crash_detection > 10 ?
					100 : psettings->motor_direction;

	pdshot->motor_master_direction 		= psettings->motor_direction_master;
	pdshot->motor_direction 			= psettings->motor_direction;




}

void wait_for_dshot_signal(void)
{
	while(signal_heartbeat)
	{ reset_device();}

	HAL_Delay(150);
	reset_device();
}

void Setup(void)
{

#ifndef PMW_124Hz
	MOTOR_TIMER->ARR = 1024-1;
#else
	MOTOR_TIMER->ARR = 512-1;
#endif

	HAL_ADCEx_Calibration_Start(&ADC_TEMP);
	HAL_ADC_Start(&ADC_TEMP);

	hdma_tim3_ch1.Instance->CPAR = (uint32_t)&TIM3->CCR1;

	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);

	MOTOR_TIMER->CCR5 = BLANKING_1_5MS;

	if (HAL_TIM_Base_Start(&htim7) != HAL_OK)
		Error_Handler();

	if( HAL_TIM_Base_Start(&E_RPM_TIMER) != HAL_OK)
		Error_Handler();

	HAL_TIM_Base_Start(&htim1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
	MOTOR_TIMER->CCR1 = 0;
	MOTOR_TIMER->CCR2 = 0;
	MOTOR_TIMER->CCR3 = 0;

#ifndef PMW_124Hz
	const uint32_t dmaElements = 2;
#else
	const uint32_t dmaElements = 4;
#endif
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dma_frequency_value, dmaElements); // A

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)dma_frequency_value, dmaElements); // B

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)dma_frequency_value, dmaElements); // C


	MOTOR_TIMER->DIER &= DIER_OFF;// ~(PWM_A_DIER_H | PWM_B_DIER_H | PWM_C_DIER_H);

	MOTOR_TIMER->CCER &= CCER_OFF;/// ~(PWM_A_CCER_H | PWM_B_CCER_H | PWM_C_CCER_H |	PWM_A_CCER_LN | PWM_B_CCER_LN | PWM_C_CCER_LN);

	hdma_tim1_ch3.Instance->CCR &= ~(1<<1);
	hdma_tim1_ch2.Instance->CCR &= ~(1<<1);
	hdma_tim1_ch1.Instance->CCR &= ~(1<<1);

	HAL_COMP_Start(&hcomp2);

	HAL_TIM_Base_Start_IT(&htim17);

	TIM17->CCR1 = 20; //20 Bump timer.

	if(HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

}

void esc_main(void)
{
//	volatile uint8_t s = sizeof(eeprom_settings_t);
	esc_status = ESC_STATUS::init;

	init_eeprom( peeprom_settings,pdshot_settings );

	HAL_Delay(500); // Delay needed for preventing interrupt lockup.

	set_motor_throttle(0);
	Setup();

	wait_for_dshot_signal();


	if( eeprom_settings.motor_direction_master != 1)
		eeprom_settings.motor_direction_master = 0;

	commutate(pdshot_settings);

	for( int i = 0; i < 3; i++)
	{
		motor_beep(tone_buffer[i][0],(uint16_t*)&tone_buffer[i][1]);
	}

	HAL_Delay(500);
	beeping = 0;

	pdshot_settings->fc_throttle = 0;



	while(1)
	{

		detect_protocol(pdshot_settings);
		motor_beep(pdshot_settings);
		motor_startup(pdshot_settings);

		dshot_rpm(pdshot_settings);

		reset_device();
	}
}
