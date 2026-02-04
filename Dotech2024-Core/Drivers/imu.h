#pragma once
#include "appConfig.hpp"

#ifndef USE_IMU
#define USE_IMU 0
#endif

#if USE_IMU

#include <math.hpp>
#include "MadgwickAHRS.h"
#include "main.h"

#define ERR_OK             0      ///< No error
#define ERR_DATA_BUS      -1      ///< Data bus error
#define ERR_IC_VERSION    -2      ///< The chip version not match

#define GYRO     0
#define ACCEL    1
#define LN_MODE  3

#define ICM42688_ID  0x47
#define ICM42688_WHO_AM_I               0x75
#define ICM42688_REG_BANK_SEL           0x76 


#define ICM42688_DEVICE_CONFIG          0x11
#define ICM42688_SELF_TEST_CONFIG       0x70
#define ICM42688_SENSOR_CONFIG0         0x03
#define ICM42688_GYRO_CONFIG0           0x4F
#define ICM42688_ACCEL_CONFIG0          0x50
#define ICM42688_GYRO_CONFIG1           0x51
#define ICM42688_GYRO_ACCEL_CONFIG0     0x52
#define ICM42688_ACCEL_CONFIG1          0x53
#define ICM42688_PWR_MGMT0              0x4E

#define ICM42688_TEMP_DATA1             0x1D
#define ICM42688_TEMP_DATA0             0x1E
#define ICM42688_ACCEL_DATA_X1          0x1F
#define ICM42688_ACCEL_DATA_X0          0x20
#define ICM42688_ACCEL_DATA_Y1          0x21
#define ICM42688_ACCEL_DATA_Y0          0x22
#define ICM42688_ACCEL_DATA_Z1          0x23
#define ICM42688_ACCEL_DATA_Z0          0x24
#define ICM42688_GYRO_DATA_X1           0x25
#define ICM42688_GYRO_DATA_X0           0x26
#define ICM42688_GYRO_DATA_Y1           0x27
#define ICM42688_GYRO_DATA_Y0           0x28
#define ICM42688_GYRO_DATA_Z1           0x29
#define ICM42688_GYRO_DATA_Z0           0x30

#define ODR_32KHZ         1
#define ODR_16KHZ         2
#define ODR_8KHZ          3
#define ODR_4KHZ          4
#define ODR_2KHZ          5
#define ODR_1KHZ          6
#define ODR_200HZ         7
#define ODR_100HZ         8
#define ODR_50HZ          9
#define ODR_25KHZ         10
#define ODR_12_5KHZ       11
#define ODR_6_25KHZ       12
#define ODR_3_125HZ       13
#define ODR_1_5625HZ      14
#define ODR_500HZ         15

#define FSR_0             0
#define FSR_1             1
#define FSR_2             2
#define FSR_3             3
#define FSR_4             4
#define FSR_5             5
#define FSR_6             6
#define FSR_7             7

/***** class start *****/
enum detect_orientation_return {
  ORIENTATION_TOP_DOWN,
  ORIENTATION_BOTTOM_DOWN,
  ORIENTATION_LEFT_DOWN,
  ORIENTATION_RIGHT_DOWN,
  ORIENTATION_BACK_DOWN,
  ORIENTATION_FRONT_DOWN,
  ORIENTATION_ERROR
};

  /**
   * @struct sGyroConfig1_t
   * @brief  Register:Gyro_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                    Reserved              |      TEMP_DIS      |        IDLE        |                GYRO_MODE            |           ACCEL_MODE        |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TEMP_DIS : 0: Temperature sensor is enabled (default)
   * @n                   1: Temperature sensor is disabled
   * @n        IDLE : 0: When the accelerometer and gyroscope are powered off, the chip will go to OFF state, since the RC oscillator will also be powered off
   * @n               1: RC oscillator is powered on even if the accelerometer and gyroscope are powered off
   * @n        GYRO_MODE :00: Turns gyroscope off (default)
   * @n                   01: Places gyroscope in Standby Mode
   * @n                   10: Reserved
   * @n                   11: Places gyroscope in Low Noise (LN) Mode
   * @n                   Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200�s
   * @n        ACCEL_MODE: 00: Turns accelerometer off (default)
   * @n                    01: Turns accelerometer off
   * @n                    10: Places accelerometer in Low Power (LP) Mode
   * @n                    11: Places accelerometer in Low Noise (LN) Mode                
   * @n                    When transitioning from OFF to any of the other modes, do not issue any register writes for 200�s
   */
  typedef struct {
    uint8_t   accelMode: 2; 
    uint8_t   gyroMode: 2; 
    uint8_t   idle: 1; 
    uint8_t   tempDis:1; 
    uint8_t   reserved:2;
  } __attribute__ ((packed)) sPWRMgmt0_t;

	
  /**
   * @struct sAccelConfig0_t
   * @brief  Register:Accel_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                           ACCEL_FS_SEL                        |      Reserved      |                                ACCEL_ODR                          |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_FS_SEL :Full scale select for accelerometer UI interface output
   * @n                      000: �16g (default)
   * @n                      001: �8g
   * @n                      010: �4g
   * @n                      011: �2g
   * @n                      100: Reserved
   * @n                      101: Reserved
   * @n                      110: Reserved
   * @n                      111: Reserved
   * @n        ACCEL_ODR :Accelerometer ODR selection for UI interface output
   * @n                   0000: Reserved
   * @n                   0001: 32kHz (LN mode)
   * @n                   0010: 16kHz (LN mode)
   * @n                   0011: 8kHz (LN mode)
   * @n                   0100: 4kHz (LN mode)
   * @n                   0101: 2kHz (LN mode)
   * @n                   0110: 1kHz (LN mode) (default)
   * @n                   0111: 200Hz (LP or LN mode) 
   * @n                   1000: 100Hz (LP or LN mode)
   * @n                   1001: 50Hz (LP or LN mode)
   * @n                   1010: 25Hz (LP or LN mode)
   * @n                   1011: 12.5Hz (LP or LN mode)
   * @n                   1100: 6.25Hz (LP mode)
   * @n                   1101: 3.125Hz (LP mode)
   * @n                   1110: 1.5625Hz (LP mode)
   * @n                   1111: 500Hz (LP or LN mode)
   */
  typedef struct {
    uint8_t   accelODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   accelFsSel: 3; 
  } __attribute__ ((packed)) sAccelConfig0_t;
	
  /**
   * @struct sGyroConfig0_t
   * @brief  Register:Gyro_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                            GYRO_FS_SEL                        |      Reserved      |                                 GYRO_ODR                          |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        GYRO_FS_SEL : Full scale select for gyroscope UI interface output
   * @n                      000: �2000dps (default)
   * @n                      001: �1000dps
   * @n                      010: �500dps
   * @n                      011: �250dps
   * @n                      100: �125dps
   * @n                      101: �62.5dps
   * @n                      110: �31.25dps
   * @n                      111: �15.625dps
   * @n        GYRO_ODR :Gyroscope ODR selection for UI interface output
   * @n                  0000: Reserved
   * @n                  0001: 32kHz
   * @n                  0010: 16kHz
   * @n                  0011: 8kHz
   * @n                  0100: 4kHz
   * @n                  0101: 2kHz
   * @n                  0110: 1kHz (default)
   * @n                  0111: 200Hz 
   * @n                  1000: 100Hz
   * @n                  1001: 50Hz
   * @n                  1010: 25Hz
   * @n                  1011: 12.5Hz
   * @n                  1100: Reserved
   * @n                  1101: Reserved
   * @n                  1110: Reserved
   * @n                  1111: 500Hz
   */
  typedef struct {
    uint8_t   gyroODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   gyroFsSel: 3; 
  } __attribute__ ((packed)) sGyroConfig0_t;
	
class Imu {
 private:
  // calibration
  bool cali_;
  bool  cali_flag[6];
  float accel_cali[6][3];
  uint8_t cali_step = 0;
  matrix::Vector3f scale_{1.0f, 1.0f, 1.0f};
  matrix::Vector3f offset_{0.0f, 0.0f, 0.0f};

  // algorithm
  Madgwick algorithm;

  // data
  matrix::Vector3f accel_read_v;
  matrix::Vector3f gyro_read_v;
  matrix::Vector3f accel_final_v;
  matrix::Vector3f euler_final_v;
  float temperature_{NAN};
  uint8_t gravity_{0};
  float scale_to_set = 1.0019;

  // avg
  float UIX[2];
  float UIY[2];
  float UIZ[2];
  float XA[500],YA[500],ZA[500];
  void ResetCalibration();
	void UpdateICM42688(void);
  void ProcessAccel();
  void UpdateEulerAngles(void);
  void DoAngleAvg();
  void ReadAccelAvg();
  void DoAccelCalibrationFull();
  void ParametersSave();
  void ParametersGet();
  void ParseSerialFromMaster();
  void ReadSerialData(unsigned char rc);
	float getTemperature(void);
	float getAccelDataX(void);
	float getAccelDataY(void);
	float getAccelDataZ(void);
	float getGyroDataX(void);
	float getGyroDataY(void);
	float getGyroDataZ(void);

 public:
  void Init();
  void Process();
  void RxFormMaster();
  void TxToMaster();
  bool  set_scale(const matrix::Vector3f scale);
  bool  set_offset(const matrix::Vector3f offset);
  matrix::Vector3f get_accel_read()  {return accel_read_v;};
  matrix::Vector3f get_accel_final() {return accel_final_v;};
  matrix::Vector3f get_euler_final() {return euler_final_v;};
  float get_roll_ui();
  float get_pitch_ui();
  float get_yaw_ui();
};

#endif