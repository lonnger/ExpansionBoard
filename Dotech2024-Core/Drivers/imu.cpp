#include "imu.h"

#if USE_IMU

#include <config.h>
#include <float.h>
#include <math.hpp>
#include "main.h"
#include "spi.h"


//	#include "stm32f4xx_hal.h"
// SPI_HandleTypeDef hspi1;
#define hspi_imu hspi1
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
sPWRMgmt0_t PWRMgmt0;
sGyroConfig0_t gyroConfig0;
sAccelConfig0_t accelConfig0;
float _gyroRange;
float _accelRange;
unsigned long print_timer_accel = 0;
unsigned long print_timer_euler = 0;
char rx_buffer[128];
const int rx_max_num = 128;
bool rx_data_flag = 0;

bool setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR);

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_SPI1_Init(void)
// {

//   /* USER CODE BEGIN SPI1_Init 0 */

//   /* USER CODE END SPI1_Init 0 */

//   /* USER CODE BEGIN SPI1_Init 1 */

//   /* USER CODE END SPI1_Init 1 */
//   /* SPI1 parameter configuration*/
//   hspi_imu.Instance = SPI1; // 指定要使用的 SPI 外设实例
//   hspi_imu.Init.Mode = SPI_MODE_MASTER;// 设置 SPI 模式为主模式
//   hspi_imu.Init.Direction = SPI_DIRECTION_2LINES;// 设置 SPI 数据传输方向为双向模式
//   hspi_imu.Init.DataSize = SPI_DATASIZE_8BIT;// 设置 SPI 数据帧的大小为 8 位
//   hspi_imu.Init.CLKPolarity = SPI_POLARITY_LOW;// 设置 SPI 时钟极性为低电平
//   hspi_imu.Init.CLKPhase = SPI_PHASE_1EDGE;// 设置 SPI 时钟相位为第二个边沿采样，下降沿
//   hspi_imu.Init.NSS = SPI_NSS_SOFT;// 设置 SPI 片选信号（NSS）为软件控制模式
//   hspi_imu.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;// 设置 SPI 波特率预分频器为 2
//   hspi_imu.Init.FirstBit = SPI_FIRSTBIT_MSB;// 设置 SPI 数据传输的第一个位为最高有效位（MSB）
//   hspi_imu.Init.TIMode = SPI_TIMODE_DISABLE;// 禁用 SPI 时钟同步模式
//   hspi_imu.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;// 禁用 SPI 的 CRC 计算功能
//   hspi_imu.Init.CRCPolynomial = 10;// 设置 SPI 的 CRC 多项式。在这个例子中，CRC 功能被禁用，所以该值没有实际意义。
//   if (HAL_SPI_Init(&hspi_imu) != HAL_OK)
//   {
// //    Error_Handler();
//   }
//   /* USER CODE BEGIN SPI1_Init 2 */

//   /* USER CODE END SPI1_Init 2 */

// }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
// void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
// /* USER CODE BEGIN MX_GPIO_Init_1 */
// /* USER CODE END MX_GPIO_Init_1 */

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOH_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

//   /*Configure GPIO pin : SPI1_CS_Pin */
//   GPIO_InitStruct.Pin = SPI1_CS_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

// /* USER CODE BEGIN MX_GPIO_Init_2 */
// /* USER CODE END MX_GPIO_Init_2 */
// }

uint8_t ICM42688_SPI_Transfer(uint8_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi_imu, &data, &rx_data, 1, 1000);
    return rx_data;
}
uint8_t readReg(uint8_t reg, void* pBuf, size_t size)
{
  if (pBuf == NULL) {
    // 处理 pBuf 为空指针的情况
    // DBG("pBuf ERROR!! : null pointer");
  }

  uint8_t* _pBuf = (uint8_t*)pBuf;
  size_t count = 0;

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // 低电平使能片选信号

  // 发送寄存器地址
  uint8_t txData = reg | 0x80;
  ICM42688_SPI_Transfer(txData);

  // 接收数据
  while (size--) {
    *_pBuf = ICM42688_SPI_Transfer(0);
    _pBuf++;
    count++;
  }
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 高电平禁用片选信号
  return count;
}

void writeReg(uint8_t reg, void* pBuf, size_t size)
{
  if (pBuf == NULL) {
//    DBG("pBuf ERROR!! : null pointer");
    return;
  }
  HAL_Delay(1);
  uint8_t* _pBuf = (uint8_t*)pBuf;

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // 低电平使能片选信号

  // 发送寄存器地址
  ICM42688_SPI_Transfer(reg);

  // 发送数据
  ICM42688_SPI_Transfer(*_pBuf);

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 高电平禁用片选信号
}
/***** variable start *****/

bool setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
      writeReg(ICM42688_GYRO_CONFIG0,&gyroConfig0,1);
      switch(FSR){
        case FSR_0:
          _gyroRange = 4000/65535.0;
          break;
        case FSR_1:
          _gyroRange = 2000/65535.0;
          break;
        case FSR_2:
          _gyroRange = 1000/65535.0;
          break;
        case FSR_3:
          _gyroRange = 500/65535.0;
          break;
        case FSR_4:
          _gyroRange = 250/65535.0;
          break;
        case FSR_5:
          _gyroRange = 125/65535.0;
          break;
        case FSR_6:
          _gyroRange = 62.5/65535.0;
          break;
        case FSR_7:
          _gyroRange = 31.25/65535.0;
          break;
      }
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
      writeReg(ICM42688_ACCEL_CONFIG0,&accelConfig0,1);
      switch(FSR){
        case FSR_0:
          _accelRange = 0.488f;// 其实是4000/8194
          break;
        case FSR_1:
          _accelRange = 0.244f;// 2000/8194
          break;
        case FSR_2:
          _accelRange = 0.122f;// 1000/8194
          break;
        case FSR_3:
          _accelRange = 0.061f;// 500/8194
          break;
      }
    }
  } 
  return ret;
}

int begin(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 初始化为高电平，即非选中，等待选中
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL, &bank, 1);
  
  uint8_t id = 0;
  if (readReg(ICM42688_WHO_AM_I, &id, 1) == 0) {
    // DBG("bus data access error");
    return ERR_DATA_BUS;
  }
//  DBG("real sensor id=");
//  DBG(id);
  if (id != ICM42688_ID) {
    return ERR_IC_VERSION;
  }
  
  uint8_t reset = 0;
  writeReg(ICM42688_DEVICE_CONFIG, &reset, 1);
  HAL_Delay(2);
  return ERR_OK;
}

void startTempMeasure()
{
  PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&PWRMgmt0,1);
  HAL_Delay(1);
}

void startGyroMeasure(uint8_t mode)
{
  PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&PWRMgmt0,1);
  HAL_Delay(1);
}

void startAccelMeasure(uint8_t mode)
{
  PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&PWRMgmt0,1);
  HAL_Delay(10);
}
 
void Imu::Init() {
	// MX_GPIO_Init();
	// MX_SPI1_Init();

	int status;
	while ((status = begin()) != 0) {
	if (status == -1) {
	//      Serial.println("bus data access error");
	} else{
	//      Serial.println("Chip versions do not match");
	HAL_Delay(10);
	}
	}
	
	setODRAndFSR(GYRO, ODR_200HZ, FSR_3);
	setODRAndFSR(ACCEL, ODR_200HZ, FSR_3);
  // 启动温度测量
  startTempMeasure();
  // 启动陀螺仪测量:使用低噪声模式进行测量
  startGyroMeasure(LN_MODE);
  // 启动加速度计测量:使用低噪声模式进行测量
  startAccelMeasure(LN_MODE);
	HAL_Delay(1000);
	UpdateICM42688();
}

void Imu::ProcessAccel() {
#ifdef SCALE_9_8
  accel_read_v /= 1670.601f;
#endif
  matrix::Vector3f accel_correct{(accel_read_v - offset_).emult(scale_)};
  accel_final_v = accel_correct;

//  if (PRINT_ACCEL_CORRECT) {
//    if (millis() - print_timer_accel > 1000) {
//      // 将 Vector3f 转换为字符串
//      String vectorStr = "a_read(";
//      vectorStr += String(accel_read_v(0), 4);
//      vectorStr += ", ";
//      vectorStr += String(accel_read_v(1), 4);
//      vectorStr += ", ";
//      vectorStr += String(accel_read_v(2), 4);
//      vectorStr += ") ";
//      vectorStr += "a_correct(";
//      vectorStr += String(accel_correct(0), 4);
//      vectorStr += ", ";
//      vectorStr += String(accel_correct(1), 4);
//      vectorStr += ", ";
//      vectorStr += String(accel_correct(2), 4);
//      vectorStr += ")";
//      Serial.println(vectorStr);
//      print_timer_accel = millis();
//    }
//  }
}

void Imu::UpdateEulerAngles() {
//  String vectorStr = "";

  algorithm.updateIMU(gyro_read_v(0), gyro_read_v(1), gyro_read_v(2),
                      accel_read_v(0), accel_read_v(1), accel_read_v(2));
  matrix::Vector3f euler_read_v{algorithm.getRoll(), algorithm.getPitch(),
                                algorithm.getYaw()};
//  if (PRINT_EULER_ANGLE) {
//    // 将 Vector3f 转换为字符串
//    vectorStr += "e_r(";
//    vectorStr += String(euler_read_v(0), 4);
//    vectorStr += ", ";
//    vectorStr += String(euler_read_v(1), 4);
//    vectorStr += ", ";
//    vectorStr += String(euler_read_v(2), 4);
//    vectorStr += ") ";
//  }

  algorithm.updateIMU(gyro_read_v(0), gyro_read_v(1), gyro_read_v(2),
                      accel_final_v(0), accel_final_v(1), accel_final_v(2));
  matrix::Vector3f euler_correct_v{algorithm.getRoll(), algorithm.getPitch(),
                                   algorithm.getYaw()};
//  if (PRINT_EULER_ANGLE) {
//    vectorStr += "e_c(";
//    vectorStr += String(euler_correct_v(0), 4);
//    vectorStr += ", ";
//    vectorStr += String(euler_correct_v(1), 4);
//    vectorStr += ", ";
//    vectorStr += String(euler_correct_v(2), 4);
//    vectorStr += ")\n";
//    if (millis() - print_timer_euler > 1000) {
//      // 将 Vector3f 转换为字符串
//      Serial.println(vectorStr);
//      print_timer_euler = millis();
//    }
//  }
}

void Imu::DoAngleAvg() {
  int num = 100;
  matrix::Vector3f angle_avg{algorithm.getRoll(), algorithm.getPitch(),
                             algorithm.getYaw()};
  float XSum = 0;
  float YSum = 0;
  float ZSum = 0;
  for (int i = 0; i < num - 1; i++) {
    XA[i] = XA[i + 1];
    YA[i] = YA[i + 1];
    ZA[i] = ZA[i + 1];
    XSum += XA[i];
    YSum += YA[i];
    ZSum += ZA[i];
  }
  XA[num - 1] = angle_avg(0);
  YA[num - 1] = angle_avg(1);
  ZA[num - 1] = angle_avg(2);
  XSum += angle_avg(0);
  YSum += angle_avg(1);
  ZSum += angle_avg(2);

  euler_final_v(0) = XSum / num;
  euler_final_v(1) = YSum / num;
  euler_final_v(2) = ZSum / num;
}

float Imu::get_roll_ui() {
  if (euler_final_v(0) > UIX[0]) {
    UIX[0] = euler_final_v(0);
  }
  if (euler_final_v(0) < UIX[1]) {
    UIX[1] = euler_final_v(0);
  }
  if ((abs(UIX[1] - UIX[0])) < UI_Threshold) {
    // Serial.print("[0]: "); Serial.print(UIPitch[0],4);  Serial.print("\t");
    // Serial.print("[1]: "); Serial.print(UIPitch[1],4);  Serial.print("\t");
    return ((UIX[0] + UIX[1]) / 2);
  }
  // UItimerX = millis();
  UIX[0] = euler_final_v(0);
  UIX[1] = euler_final_v(0);
  return euler_final_v(0);
}

float Imu::get_pitch_ui() {
  if (euler_final_v(1) > UIY[0]) {
    UIY[0] = euler_final_v(1);
  }
  if (euler_final_v(1) < UIY[1]) {
    UIY[1] = euler_final_v(1);
  }
  if ((abs(UIY[1] - UIY[0])) < UI_Threshold) {
    // Serial.print("[0]: "); Serial.print(UIPitch[0],4);  Serial.print("\t");
    // Serial.print("[1]: "); Serial.print(UIPitch[1],4);  Serial.print("\t");
    return ((UIY[0] + UIY[1]) / 2);
  }
  // UItimerX = millis();
  UIY[0] = euler_final_v(1);
  UIY[1] = euler_final_v(1);
  return euler_final_v(1);
}

float Imu::get_yaw_ui() {
  if (euler_final_v(2) > UIZ[0]) {
    UIZ[0] = euler_final_v(2);
  }
  if (euler_final_v(2) < UIZ[1]) {
    UIZ[1] = euler_final_v(2);
  }
  if ((abs(UIZ[1] - UIZ[0])) < UI_Threshold) {
    // Serial.print("[0]: "); Serial.print(UIPitch[0],4);  Serial.print("\t");
    // Serial.print("[1]: "); Serial.print(UIPitch[1],4);  Serial.print("\t");
    return ((UIZ[0] + UIZ[1]) / 2);
  }
  // UItimerX = millis();
  UIZ[0] = euler_final_v(2);
  UIZ[1] = euler_final_v(2);
  return euler_final_v(2);
}

bool Imu::set_offset(const matrix::Vector3f offset) {
  // if (matrix::Vector3f(offset_ - offset).longerThan(0.01f)) {
  //   if (offset.isAllFinite()) {
  offset_ = offset;
  // 将 Vector3f 转换为字符串
//  String vectorStr = "offset_: (";
//  vectorStr += String(offset_(0), 4);
//  vectorStr += ", ";
//  vectorStr += String(offset_(1), 4);
//  vectorStr += ", ";
//  vectorStr += String(offset_(2), 4);
//  vectorStr += ")";
//  Serial.println(vectorStr);
  //     // _calibration_count++;
  return true;
  //   }
  // }
  // return false;
}

bool Imu::set_scale(const matrix::Vector3f scale) {
  // if (matrix::Vector3f(scale_ - scale).longerThan(0.01f)) {
  //   if (scale.isAllFinite() && (scale(0) > 0.f) && (scale(1) > 0.f) &&
  //       (scale(2) > 0.f)) {
  scale_ = scale;
  // 将 Matrix3f 转换为字符串
//  String vectorStr = "scale_: (";
//  vectorStr += String(scale_(0), 4);
//  vectorStr += ", ";
//  vectorStr += String(scale_(1), 4);
//  vectorStr += ", ";
//  vectorStr += String(scale_(2), 4);
//  vectorStr += ")";
//  Serial.println(vectorStr);
  // _calibration_count++;
  return true;
  //   }
  // }

  // return false;
}

#ifdef SCALE_9_8
static constexpr float CONSTANTS_ONE_G = 9.80665f;
#elif
static constexpr float CONSTANTS_ONE_G = 16383.0f;
#endif
static constexpr unsigned detect_orientation_side_count = 6;
float accel_raw_ref[detect_orientation_side_count][3]{};

void Imu::ReadAccelAvg() {
  // check cali_step is right[1-6]
  if (cali_step < 1 || cali_step > 6) {
//    Serial.printf("[ReadAccelAvg]ERROR:cali_step = %d\n", cali_step);
    return;
  }

  // check if cali already
  if (cali_flag[cali_step - 1] == 1) {
    return;
  }

  int num = 750;
  int counts = 0;
  matrix::Vector3f raw;
  matrix::Vector3f sum;
  matrix::Vector3f start;

  while (counts < num) {
    UpdateICM42688();
#ifdef SCALE_9_8
    matrix::Vector3f accel_raw_v{accel_read_v/1670.601f};
#elif
    matrix::Vector3f accel_raw_v{accel_read_v}
#endif
    if (counts == 0) {
      start = accel_raw_v;
//      Serial.printf("[counts]%d\n", counts);
    }

    raw = accel_raw_v;
    if ((raw - start).norm() > IMU_CALI_TH) {
//      Serial.printf("[ReadAccelAvg] > norm:%f\n", (raw - start).norm());
      return;
    }
    sum += raw;
    counts++;
//    Serial.printf("[counts]%d\n", counts);
  }

  const matrix::Vector3f avg{sum / counts};
  avg.copyTo(accel_cali[cali_step - 1]);

//  Serial.print("[AccelResult]");
//  for (int j = 0; j < 3; ++j) {
//    Serial.print(String(accel_cali[cali_step - 1][j], 4));
//    Serial.print(",");
//  }

//  Serial.println("\n");  // 换行
//  // 复位，等待管理员调换姿势按下按键
//  Serial.printf("[ReadAccelAvg]cali_step:%d complete\n", cali_step);
  counts = 0;
  sum.setZero();
  cali_flag[cali_step - 1] = 1;
}

void Imu::DoAccelCalibrationFull() {
  /*=== 计算出 offset ===*/
  // X offset: average X from accel_top_down + accel_bottom_down
  matrix::Vector3f offset_v;
  const matrix::Vector3f accel_top_down{accel_cali[ORIENTATION_TOP_DOWN]};
  const matrix::Vector3f accel_bottom_down{accel_cali[ORIENTATION_BOTTOM_DOWN]};
  offset_v(0) = (accel_top_down(0) + accel_bottom_down(0)) * 0.5f;

  // Y offset: average Y from accel_left + accel_right
  const matrix::Vector3f accel_left_down{accel_cali[ORIENTATION_LEFT_DOWN]};
  const matrix::Vector3f accel_right_down{accel_cali[ORIENTATION_RIGHT_DOWN]};
  offset_v(1) = (accel_left_down(1) + accel_right_down(1)) * 0.5f;

  // Z offset: average Z from accel_back_down + accel_front_down
  const matrix::Vector3f accel_back_down{accel_cali[ORIENTATION_BACK_DOWN]};
  const matrix::Vector3f accel_front_down{accel_cali[ORIENTATION_FRONT_DOWN]};
  offset_v(2) = (accel_back_down(2) + accel_front_down(2)) * 0.5f;

  /*=== 计算出 accel_T ===*/
  matrix::Matrix3f mat_A;
  mat_A.row(0) = accel_top_down - offset_v;
  mat_A.row(1) = accel_left_down - offset_v;
  mat_A.row(2) = accel_back_down - offset_v;

  // calculate inverse matrix for A: simplify matrices mult because b has only
  // one non-zero element == g at index i
  const matrix::Matrix3f accel_T = mat_A.I() * CONSTANTS_ONE_G;

  /*=== 设置 offset 和 scale*/
  set_offset(offset_v);
  set_scale(accel_T.diag());
  ParametersSave();

  // 将 Matrix3f 转换为字符串
//  String matrixStr = "";
//  for (int i = 0; i < 3; i++) {
//    for (int j = 0; j < 3; j++) {
//      matrixStr += String(accel_T(i, j), 4);
//      matrixStr += "\t";
//    }
//    matrixStr += "\n";
//  }
//  Serial.println("[accel_T]");
//  Serial.println(matrixStr);

//  // 将 Vector3f 转换为字符串
//  String vectorStr = "offset_v: (";
//  vectorStr += String(offset_v(0), 4);
//  vectorStr += ", ";
//  vectorStr += String(offset_v(1), 4);
//  vectorStr += ", ";
//  vectorStr += String(offset_v(2), 4);
//  vectorStr += ")";
//  Serial.println(vectorStr);
}
void Imu::ParametersSave() {
//    while (!pref.begin("imu_cali", false)) {
//        Serial.println("[DoAccelCalibrationFull]imu_cali Put Fail");
//    }
//#ifdef SCALE_9_8
//    pref.putFloat("offset0_9_8", offset_(0));
//    pref.putFloat("offset1_9_8", offset_(1));
//    pref.putFloat("offset2_9_8", offset_(2));
//    pref.putFloat("scale0_9_8", scale_(0));
//    pref.putFloat("scale1_9_8", scale_(1));
//    pref.putFloat("scale2_9_8", scale_(2));
//    pref.end();
//#elif
//    pref.putFloat("offset0", offset_(0));
//    pref.putFloat("offset1", offset_(1));
//    pref.putFloat("offset2", offset_(2));
//    pref.putFloat("scale0", scale_(0));
//    pref.putFloat("scale1", scale_(1));
//    pref.putFloat("scale2", scale_(2));
//    pref.end();
//#endif
}

void Imu::ParametersGet() {
//  while (!pref.begin("imu_cali", false)) {
//    Serial.println("[ImuTask]imu_cali get Fail");
//  }
//#ifdef SCALE_9_8
//  offset_(0) = pref.getFloat("offset0_9_8", 0.0);
//  offset_(1) = pref.getFloat("offset1_9_8", 0.0);
//  offset_(2) = pref.getFloat("offset2_9_8", 0.0);
//  scale_(0)  = pref.getFloat("scale0_9_8", 1.0);
//  scale_(1)  = pref.getFloat("scale1_9_8", 1.0);
//  scale_(2)  = pref.getFloat("scale2_9_8", 1.0);
//#elif
//  offset_(0) = pref.getFloat("offset0", 0.0);
//  offset_(1) = pref.getFloat("offset1", 0.0);
//  offset_(2) = pref.getFloat("offset2", 0.0);
//  scale_(0)  = pref.getFloat("scale0", 1.0);
//  scale_(1)  = pref.getFloat("scale1", 1.0);
//  scale_(2)  = pref.getFloat("scale2", 1.0);
//#endif
//  pref.end();
}

void Imu::RxFormMaster() {
//    int StartTime = millis();
//    while (Serial1.available() && millis() - StartTime < 1000 && !rx_data_flag) {
//    ReadSerialData(Serial1.read());
//    }
//    ParseSerialFromMaster();
}

void Imu::TxToMaster() {
//  String dataString;
//  dataString = "";
//  dataString += "<";
//  dataString += String(euler_final_v(0), 4);
//  dataString += ",";
//  dataString += String(euler_final_v(1), 4);
//  dataString += ",";
//  dataString += String(euler_final_v(2), 4);
//  dataString += ",";
//  dataString += String(get_roll_ui(), 4);
//  dataString += ",";
//  dataString += String(get_pitch_ui(), 4);
//  dataString += ",";
//  dataString += String(get_yaw_ui(), 4);
//  dataString += ",";
//  dataString += String(accel_final_v(0), 4);
//  dataString += ",";
//  dataString += String(accel_final_v(1), 4);
//  dataString += ",";
//  dataString += String(accel_final_v(2), 4);
//  dataString += ",";
//  dataString += String(temperature_, 4);
//  dataString += ",";
//  dataString += String(gravity_, 4);
//  dataString += ">";
//  Serial1.print(dataString);
}

void Imu::ReadSerialData(unsigned char rc) {
  static bool rx_progress = false;  // 是否正在接收数据
  static uint8_t ndx = 0;
  char startMarker = '<';
  char endMarker = '>';

  // 已经开始接收数据，那么它会判断当前接收到的字符是否为结束标记
  if (rx_progress == true) {
    if (rc != endMarker) {
      rx_buffer[ndx] = rc;
      ndx++;
      if (ndx >= rx_max_num) {
        ndx = rx_max_num - 1;
      }
    } else {
      rx_buffer[ndx] = '\0';  // terminate the string
      rx_progress = false;
      ndx = 0;
      rx_data_flag = true;
    }
  } else if (rc == startMarker) {
    rx_progress = true;
  }
}

void Imu::ParseSerialFromMaster() {  // split the data into its parts
  if (rx_data_flag) {
    char* strtokIndx;  // this is used by strtok() as an index
    strtokIndx = strtok(rx_buffer, ",");  // get the first part - the string
    if (strtokIndx != NULL) cali_step = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    if (strtokIndx != NULL) scale_to_set = atof(strtokIndx);

    rx_data_flag = false;
  }
}

void Imu::ResetCalibration() {
  if (cali_ == 1) {
    cali_ = 0;
    for (int i = 0; i < 6; i++) {
      cali_flag[i] = 0;
      for (int j = 0; j < 3; ++j) {
        accel_cali[i][j] = 0.0;
      }
    }
  }
}
void Imu::Process() {
  if (cali_step == 0) {
    ResetCalibration();
    UpdateICM42688();
    ProcessAccel();
    UpdateEulerAngles();
    DoAngleAvg();
  } else if (cali_step == 7) {
    if (cali_ == 0) {
      DoAccelCalibrationFull();
      cali_ = 1;
    }
  } else {
    ReadAccelAvg();
  }
}


float Imu::getTemperature(void)
{
  float value;
	uint8_t data[2];
	int16_t value2;
	readReg(ICM42688_TEMP_DATA1, data, 2);
	value2 = ((uint16_t )data[0]<<8) | (uint16_t )data[1];
	value = value2/132.48 + 25;
  return value;
}

float Imu::getAccelDataX(void)
{
  float value;
	uint8_t data[2];
	readReg(ICM42688_ACCEL_DATA_X1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  // return value*_accelRange;
  return value;
}

float Imu::getAccelDataY(void)
{
  float value;
	uint8_t data[2];
	readReg(ICM42688_ACCEL_DATA_Y1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  return value;
}

float Imu::getAccelDataZ(void)
{
	float value;
  uint8_t data[2];
	readReg(ICM42688_ACCEL_DATA_Z1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  return value;
}

float Imu::getGyroDataX(void)
{
  float value;
	uint8_t data[2];
	readReg(ICM42688_GYRO_DATA_X1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  return value*_gyroRange;
}

float Imu::getGyroDataY(void)
{
  float value;
	uint8_t data[2];
	readReg(ICM42688_GYRO_DATA_Y1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  return value*_gyroRange;
}

float Imu::getGyroDataZ(void)
{
  float value;
	uint8_t data[2];
	readReg(ICM42688_GYRO_DATA_Z1, data, 2);
	int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
	value = value1;
  return value*_gyroRange;
}

void Imu::UpdateICM42688() {
  double ICM42688Temp;
  if ((ICM42688Temp = getTemperature()) != 0) {
    temperature_ = ICM42688Temp;
  }
  if ((ICM42688Temp = getAccelDataX()) != 0) {
    accel_read_v(0) = ICM42688Temp;
  }
  if ((ICM42688Temp = getAccelDataY()) != 0) {
    accel_read_v(1) = ICM42688Temp;
  }
  if ((ICM42688Temp = getAccelDataZ()) != 0) {
    accel_read_v(2) = ICM42688Temp;
  }
  if ((ICM42688Temp = getGyroDataX()) != 0) {
    gyro_read_v(0) = ICM42688Temp;
  }
  if ((ICM42688Temp = getGyroDataY()) != 0) {
    gyro_read_v(1) = ICM42688Temp;
  }
  if ((ICM42688Temp = getGyroDataZ()) != 0) {
    gyro_read_v(2) = ICM42688Temp;
  }
}

#endif