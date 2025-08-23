/*
 * eeprom.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Gil Vargas
 */
/*EEPROM
 * xF800
 * Motor direction:1
 * Startup throttle:2
 * Turtle rampup:1
 * Protocol:1 = 0=notset, 1=300,2=300-bi,3=600,4=6000-bi.
 * CommutationDelay:1, 1=0.5us,2=1us,3=1.5us,4=2us,5=2.5us
 *
 */
#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ID 			 0x47
#define BASE_ADDRESS		 0x08000000
#define APPLICATION_ADDRESS  0x1000

#define EEPROM_ADDRESS 		 0xF800

#define  WORD_SIZE  (0x04)
#define PAGE_SIZE  (0x800)
const unsigned short  THROTTLE_DELAY = 300;
const uint16_t THROTTLE_RAMPUP = 2048;

typedef struct eeprom_settings_s {
	char name[12] 			= {"Gil32"};		//		= {"Gil32"};
	char description[64]	= {"ESC Firmware"};//
	uint8_t version	 			= 1;
	uint8_t sub_version 		= 1;
	uint8_t esc_config_layout	= 1;
	uint8_t commutation_delay 	= 64;
	uint16_t startup_throttle = THROTTLE_DELAY ;//0x2c01; // 300=0x12c reversed.
	uint8_t turtle_rampup  		= 50;
	uint16_t rampup  			= THROTTLE_RAMPUP;
	uint8_t motor_direction_master  = 0x1;
	uint8_t motor_direction 	= 0x01;
	uint8_t crash_detection     = 0x00;
	uint8_t feature2  			= 0xff;
	uint8_t feature3 			= 0xff;
	uint8_t feature4  			= 0xff;
	uint8_t feature5  			= 0xff;
	uint8_t feature6  			= 0xff;
	uint8_t feature7  			= 0xff;
	uint8_t feature8  			= 0xff;
	uint8_t feature9  			= 0xff;
	uint8_t feature10  			= 0xff;
	uint8_t feature11  			= 0xff;
	uint8_t feature12  			= 0xff;
	uint8_t feature13  			= 0xff;
	uint8_t feature14  			= 0xff;
	uint8_t feature15  			= 0xff;
	uint8_t feature16  			= 0xff;

}__attribute__((packed)) eeprom_settings_t;



void read_memory(uint8_t *data, const uint16_t size, const uint32_t address);

void write_memory(uint8_t* data,const uint16_t size,const uint32_t address) ;


#endif /* INC_EEPROM_H_ */
