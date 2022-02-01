#include "config.h"
#include "config_helper.h"

#define OmnibusF4

//PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PC10PC11PC12

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

#define USB_DETECT_PIN PIN_C5

//LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_B5
#define LED1_INVERT
#define BUZZER_PIN PIN_B4
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

//GYRO
#define GYRO_TYPE MPU6XXX
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_C4
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x71
#define SENSOR_ROTATE_90_CCW

//RADIO
#define USART1_INVERTER_PIN PIN_C0

#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

//OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT3
#define MAX7456_NSS PIN_A15

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_C2
#define BATTERY_ADC_CHANNEL LL_ADC_CHANNEL_12
#define VOLTAGE_DIVIDER_R1 10000
#define VOLTAGE_DIVIDER_R2 1000

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PA3
#define MOTOR_PIN1 MOTOR_PIN_PA2
#define MOTOR_PIN2 MOTOR_PIN_PB0
#define MOTOR_PIN3 MOTOR_PIN_PB1

//NFE 3in 3100kv
//#define MOTOR_PIN0 MOTOR_PIN_PB0
//#define MOTOR_PIN1 MOTOR_PIN_PB1
//#define MOTOR_PIN2 MOTOR_PIN_PA2
//#define MOTOR_PIN3 MOTOR_PIN_PA3
