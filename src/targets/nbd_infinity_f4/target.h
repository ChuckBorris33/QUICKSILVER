#include "config.h"

#define NBD_Infinity_F4

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

// LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_C0
#define LED1_INVERT
#define BUZZER_PIN PIN_B6
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

// GYRO
#define GYRO_SPI_PORT SPI_PORT2
#define GYRO_NSS PIN_B12
#define GYRO_INT PIN_C13

// RADIO
#define USART1_INVERTER_PIN PIN_A0

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT1
#define MAX7456_NSS PIN_A4

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_C4
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_C3
#define IBAT_SCALE 245

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PC9
#define MOTOR_PIN1 MOTOR_PIN_PA8
#define MOTOR_PIN2 MOTOR_PIN_PB1
#define MOTOR_PIN3 MOTOR_PIN_PB0

// BLACKBOX
// spi3 - flash chip
// resource FLASH_CS 1 A15
