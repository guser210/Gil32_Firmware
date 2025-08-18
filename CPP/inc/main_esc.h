/*
 * main_esc.h
 *
 *  Created on: Aug 16, 2025
 *      Author: gilv2
 */

#ifndef INC_MAIN_ESC_H_
#define INC_MAIN_ESC_H_




#define PMW_124Hz
#undef PMW_124Hz

#include "eeprom.h"
#include "variables.h"

eeprom_settings_t eeprom_settings;
eeprom_settings_t *peeprom_settings = &eeprom_settings;

volatile bool signal_heartbeat = true;
volatile char userID[] = {"Gil Vargasz"};
//volatile uint8_t crashDetected = 0;
volatile uint16_t throttleHalves = 0;

volatile uint16_t disableFetsCounter = 0;
volatile uint8_t motor_not_spinning = 1;

volatile uint8_t protocol_detected = 0;
volatile uint32_t protocol_timer_psc = 0;


dshot_signal_t dshot_settings;
dshot_signal_t *pdshot_settings = &dshot_settings;

volatile uint8_t counter1 = 0;

void Setup(void);
void disconnect_motor_phases(void);
void set_motor_throttle_beep( uint16_t duty);
void set_motor_throttle( uint16_t duty);
void process_dshot_command(uint8_t cmd, dshot_signal_t *pdshot);
void process_throttle_input(uint16_t *throttle_value,dshot_signal_t* pdshot);
void set_receive_signal_from_fc(void);
void set_send_rpm_to_fc(dshot_signal_t *pdshot);
void convert_dshot_frame_to_dma_buffer(const uint32_t rawValue,dshot_signal_t *pdshot);
uint32_t pack_dshot_frame(const uint32_t value);
void motor_beep(const uint16_t freq, uint16_t *delay);
void commutate(dshot_signal_t *pdshot);
void reset_device();
void detect_protocol(dshot_signal_t *pdshot);
void motor_beep(dshot_signal_t *pdshot);
void disconnect_motor_phases(void);
void motor_startup(dshot_signal_t *pdshot);
void dshot_rpm(dshot_signal_t *pdshot);
void init_eeprom(eeprom_settings_t *psettings, dshot_signal_t *pdshot);
void wait_for_dshot_signal(void);



#endif /* INC_MAIN_ESC_H_ */
