#ifndef CONFIG_METER_IMU_H_
#define CONFIG_METER_IMU_H_
/***** compile options start *****/
#define SCALE_9_8

//#define PRINT_ACCEL_CORRECT 1
//#define PRINT_EULER_ANGLE   0

//#define SENSOR_TASK_PERIOD      5
//#define COMMUNICATE_TASK_PERIOD 50

#define UI_Threshold 0.08f

#ifdef SCALE_9_8
#define IMU_CALI_TH 0.1f
#elif 
#define IMU_CALI_TH 60.0f
#endif

//#define LED_PIN 8
//#define IMU_PIN 12
//#define IMU_SPI_SCK 7
//#define IMU_SPI_MISO 10
//#define IMU_SPI_MOSI 3
//#define IMU_SPI_SS 6
/***** compile options end *****/
#endif