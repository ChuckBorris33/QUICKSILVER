#include "config.h"
#include "config_helper.h"

#define BetaFPVF411RX

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2

// LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_C14
#define LED1_INVERT

// GYRO
#define GYRO_TYPE ICM20602
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_B6
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x98

// RADIO
#ifdef RX_EXPRESS_LRS
#define USE_SX128X
#define SX12XX_SPI_PORT SPI_PORT3
#define SX12XX_NSS_PIN PIN_A15
#define SX12XX_DIO0_PIN PIN_C13
#define SX12XX_BUSY_PIN PIN_A13
#define SX12XX_RESET_PIN PIN_B9
#endif

#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define BATTERYPIN PIN_A1
#define BATTERY_ADC_CHANNEL LL_ADC_CHANNEL_1

#define VOLTAGE_DIVIDER_R1 10000

#define VOLTAGE_DIVIDER_R2 1000

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB10
#define MOTOR_PIN1 MOTOR_PIN_PB7
#define MOTOR_PIN2 MOTOR_PIN_PB8
#define MOTOR_PIN3 MOTOR_PIN_PA0
