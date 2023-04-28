#ifndef CONSTANT_DEFINITION_H
#define CONSTANT_DEFINITION_H

#include <Arduino.h>

/*======== Hardware Definition ========*/
/*==== Pin Definition ====*/
/*== USB Baud Rate ==*/
#define BAUDRATE_USB					115200

/*== LED Control ==*/
#define PIN_LED_ONBOARD					13

/*== Motor ==*/
/*= Encoder =*/
#define PIN_ENCODER_A					2	// PD2
#define PIN_ENCODER_B					3	// PD3

#define MASK_ENCODER					0x0C //00001100
#define MASK_ENCODER_PLUS				0x04 //00000100

#define DATA_ENCODER					PIND

/*= Driver L298 =*/
#define PIN_INA							6
#define PIN_INB							5
#define PIN_ENABLE					4

/*== Hardware Setup Time ==*/
#define TIME_SETUP_DELAY_MS				500

/*======== Software Definition ========*/
/*==== USB Constant ====*/
#define TIME_USB_MS						5000

/*==== USB Plot Constant ====*/
#define TIME_USB_MS_PLOT				10

/*==== LED Constant ====*/
#define TIME_LED_FLASH_MS				25

#define LED_ONBOARD_ON()				digitalWrite(PIN_LED_ONBOARD, HIGH )
#define LED_ONBOARD_OFF()				digitalWrite(PIN_LED_ONBOARD, LOW )

/*==== Motor Constant ====*/
#define TIME_ENCODER_MS					1
#define TIME_CONTROL_MS					1
#define TIME_SPEED_CONTROL_MS   10
#define TIME_FORCE_CONTROL_MS   10



#define MODE_STOP				0
#define MODE_VOLT				1
#define MODE_REVERSE				2

#define MOTOR_CW(_PWM )					analogWrite(PIN_INA, 0 ), analogWrite(PIN_INB, _PWM ), digitalWrite(PIN_ENABLE, HIGH )
#define MOTOR_CCW(_PWM )				analogWrite(PIN_INA, _PWM ), analogWrite(PIN_INB, 0 ), digitalWrite(PIN_ENABLE, HIGH )
#define MOTOR_STOP()					analogWrite(PIN_INA, 0 ), analogWrite(PIN_INB, 0 ), digitalWrite(PIN_ENABLE, LOW )
#define MAX_DUTY						255

#define DEAD_ZONE						5

#define MOTOR_ENCODER_PPR				32000 // 16(gear ratio)*500(encoder ppr)*4

#define FORCE_CONTROL_P			  150
#define FORCE_CONTROL_I  			0
#define FORCE_CONTROL_D  			0

#define AMPLITUDE_DEFAULT				16000
#define PERIOD_DEFAULT_MS				2000.0

#define SET_VOLT              550 //500g -> 498
#define MAX_VOLT              573 //835g
#define LOAD_CELL_OFFSET      700

#define X_AXIS                A0
#define Y_AXIS                A1

#endif