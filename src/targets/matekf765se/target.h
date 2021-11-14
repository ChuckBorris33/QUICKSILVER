#include "config.h"
#include "config_helper.h"

#define MatekF765SE

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5    \
  SPI4_PE12PE13PE14

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PD6PD5     \
  USART3_PD8PD9     \
  USART6_PC6PC7     \
  USART7_PE7PE8     \
  USART8_PE0PE1

// LEDS
#define LED_NUMBER 2
#define LED1PIN PIN_D10
#define LED2PIN PIN_D11
#define LED2_INVERT

#define BUZZER_PIN PIN_B9
#define BUZZER_INVERT

// GYRO
#define MPU6XXX_SPI_PORT SPI_PORT4
#define MPU6XXX_NSS PIN_E11
#define MPU6XXX_INT PIN_C13
#define SENSOR_FLIP_180
#define SENSOR_ROTATE_90_CW
#define GYRO_ID_1 0x47

// RADIO
#define SOFTSPI_NONE
#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define DISABLE_ADC

#define BATTERYPIN PIN_C2
#define BATTERY_ADC_CHANNEL LL_ADC_CHANNEL_1

#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif

#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
// S3_OUT
#define MOTOR_PIN0 MOTOR_PIN_PA2
// S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PA3
// S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PA0
// S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PA1
