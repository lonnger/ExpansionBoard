#include "main.h"
#include "can.h"
#include "ADS1115.hpp"
#include "DYPA21_PWM.hpp"
#include "FreeRTOS.h"
#include "WS2812.hpp"
#include "task.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>

/*
 * 分线板的功能：
 * 1. pin口直接控制左右电机移动
 * 2. pin口左右电机的左右两个限位开关
 * 3. can接收Bmotor的cmd
 * 4. i2c: 超声波，激光测距仪（换轨/绑扎位置），读取电流值
 * 5. can发送 光耦，超声波，激光测距
 * 6. 接收光耦
 *
 */

enum BoardPosition
{
    Front,
    Back
};
static volatile BoardPosition boardPosition = BoardPosition::Front;
const uint8_t CANTxSize                     = 8;
CAN_TxHeaderTypeDef FrontTxHeader           = {0x87, 0, CAN_ID_STD, CAN_RTR_DATA, CANTxSize, DISABLE};
CAN_TxHeaderTypeDef BackTxHeader            = {0x88, 0, CAN_ID_STD, CAN_RTR_DATA, CANTxSize, DISABLE};
uint8_t TxData[CANTxSize]                   = {0x00};  // Data to be transmitted
// Start the transmission
uint32_t TxMailbox;
static volatile uint8_t RxBuffer[10];
static volatile int cnt         = 0;
static volatile bool needReboot = false;
enum B_MOTOR_CMD
{
    GoLeft,
    GoRight,
    BMotorStop
};
enum B_MOTOR_STATE
{
    Left,
    Right,
    BMotorMiddle
};
static volatile B_MOTOR_CMD BMotorCMD;
static volatile B_MOTOR_STATE BMotorState;

enum A_MOTOR_CMD
{
    GoUp,
    GoDown,
    AMotorStop
};
/*没有Amotor了，移到绑扎枪控制板*/
enum A_MOTOR_STATE
{
    Up,
    Down,
    AMotorMiddle
};
static volatile A_MOTOR_CMD AMotorCMD;
static volatile A_MOTOR_STATE AMotorState;
static volatile bool electromagnetOn = false;

static volatile bool needRedLightOn       = false;  // 红灯是否亮 故障报警
static volatile bool needRedLightFlashing = false;  // 红灯是闪烁 故障报警
void setLedStatus(bool on, bool flashing);

void commonLedStatus();
void CANcallback(CAN_HandleTypeDef *)
{
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))
    {
        uint32_t ulOriginalBASEPRI = taskENTER_CRITICAL_FROM_ISR();
        static volatile CAN_RxHeaderTypeDef canRxHeader;
        canRxHeader.StdId = 0;
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)&canRxHeader, (uint8_t *)&RxBuffer);
        /*****************************  前分线板  *****************************/
        if (canRxHeader.StdId == 0x666 && boardPosition == BoardPosition::Front)
        {
            if (((RxBuffer[0] >> 1) & 0x01) == 0)
            {
                if ((RxBuffer[0] & 0x01) == 0)
                    AMotorCMD = A_MOTOR_CMD::GoUp;
                else
                    AMotorCMD = A_MOTOR_CMD::GoDown;
            }
            else
            {
                AMotorCMD = A_MOTOR_CMD::AMotorStop;
            }
            if (((RxBuffer[0] >> 3) & 0x01) == 0)
            {
                if (((RxBuffer[0] >> 2) & 0x01) == 0)
                    BMotorCMD = B_MOTOR_CMD::GoLeft;
                else
                    BMotorCMD = B_MOTOR_CMD::GoRight;
            }
            else
            {
                BMotorCMD = B_MOTOR_CMD::BMotorStop;
            }
            if (((RxBuffer[0] >> 4) & 0x01) == 1)
            {
                NVIC_SystemReset();
            }
            // 解析红灯状态 故障报警
            commonLedStatus();
            cnt++;
        }
        /*****************************  后分线板  *****************************/
        else if (canRxHeader.StdId == 0x669 && boardPosition == BoardPosition::Back)
        {
            if (((RxBuffer[0] >> 1) & 0x01) == 0)
            {
                if ((RxBuffer[0] & 0x01) == 0)
                    AMotorCMD = A_MOTOR_CMD::GoUp;
                else
                    AMotorCMD = A_MOTOR_CMD::GoDown;
            }
            else
            {
                AMotorCMD = A_MOTOR_CMD::AMotorStop;
            }
            if (((RxBuffer[0] >> 3) & 0x01) == 0)
            {
                if (((RxBuffer[0] >> 2) & 0x01) == 0)
                    BMotorCMD = B_MOTOR_CMD::GoLeft;
                else
                    BMotorCMD = B_MOTOR_CMD::GoRight;
            }
            else
            {
                BMotorCMD = B_MOTOR_CMD::BMotorStop;
            }
            if (((RxBuffer[0] >> 4) & 0x01) == 1)
            {
                NVIC_SystemReset();
            }
            // 解析红灯状态 故障报警
            commonLedStatus();
            cnt++;
        }

        taskEXIT_CRITICAL_FROM_ISR(ulOriginalBASEPRI);
    }
}

void commonLedStatus()
{
    if (((RxBuffer[0] >> 5) & 0x01) == 1)
    {
        if (((RxBuffer[0] >> 6) & 0x01) == 1)
        {
            setLedStatus(true, true);
        }
        else
        {
            setLedStatus(true, false);
        }
    }
    else
    {
        setLedStatus(false, false);
    }
}

void CANinit()
{
    CAN_FilterTypeDef CANFilter = {
        0,                      // filterID HI
        0,                      // filterID LO
        0,                      // filterMask HI
        0,                      // filterMask LO
        CAN_FILTER_FIFO0,       // FIFO assignment
        0,                      // filter bank
        CAN_FILTERMODE_IDMASK,  // filter mode
        CAN_FILTERSCALE_16BIT,  // filter size
        CAN_FILTER_ENABLE,      // ENABLE or DISABLE
        0                       // Slave start bank
    };

    configASSERT(HAL_CAN_ConfigFilter(&hcan, &CANFilter) == HAL_OK);
    configASSERT(HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CANcallback) == HAL_OK);
    configASSERT(HAL_CAN_Start(&hcan) == HAL_OK);
    configASSERT(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
}

ADS1115::ADS1115_Config_t ads1115Config = {
    .channel         = ADS1115::MultiplexerConfig_t::CHANNEL_AIN0_GND,
    .pgaConfig       = ADS1115::PGA_Config_t::PGA_6_144,
    .operatingMode   = ADS1115::OperatingMode_t::MODE_CONTINOUS,
    .dataRate        = ADS1115::DataRate_t::DRATE_860,
    .compareMode     = ADS1115::CompareMode_t::COMP_HYSTERESIS,
    .polarityMode    = ADS1115::ComparePolarity_t::POLARITY_ACTIVE_LOW,
    .latchingMode    = ADS1115::LatchingMode_t::LATCHING_NONE,
    .queueComparator = ADS1115::QueueComparator_t::QUEUE_ONE,
};
ADS1115::ADS1115 ads1115(&hi2c2, 0x48, ads1115Config);
static volatile int16_t adcData[4] = {0};
static DYPA21_PWM::DYPA21_PWM dypa21(&htim4);
static volatile int16_t ultrasonicValue = 0;
static volatile uint8_t CurrentValue    = 0;
static volatile uint8_t photoCouplers   = 0;
static volatile bool bumperSensor       = false;
static volatile bool bFaultPin          = false;
static StackType_t i2cTaskStack[128];
static StaticTask_t i2cTaskTCB;
void i2cTask(void *param)
{
    //AIN0_GND电压差测电流，AIN3_GND电压差测激光测距仪（绑扎位置/换轨位置），AIN2_GND，AIN1_GND备用
    ads1115.ADS1115_setConversionReadyPin(); //设置电压报警阈值
    ads1115.ADS1115_startContinousMode();  //连续转换模式，持续更新转换结果寄存器的值
    ads1115Config.channel = ADS1115::MultiplexerConfig_t::CHANNEL_AIN2_GND;
    ads1115.ADS1115_updateConfig(ads1115Config);
    while (1)
    {
        ads1115Config.channel = ADS1115::MultiplexerConfig_t::CHANNEL_AIN0_GND;
        ads1115.ADS1115_updateConfig(ads1115Config);
        adcData[0] = ads1115.ADS1115_getData();  // CurrentValue

        ads1115Config.channel = ADS1115::MultiplexerConfig_t::CHANNEL_AIN3_GND;
        ads1115.ADS1115_updateConfig(ads1115Config);  // 前分线板：检测绑扎位置激光 ；  后分线板：检测换轨位置激光
        adcData[1] = ads1115.ADS1115_getData();       // 因为主控板要的是adcData[1]

        ads1115Config.channel = ADS1115::MultiplexerConfig_t::CHANNEL_AIN2_GND;
        ads1115.ADS1115_updateConfig(ads1115Config);
        adcData[2] = ads1115.ADS1115_getData();  // 不知道干嘛的，can也没发过去

        ads1115Config.channel = ADS1115::MultiplexerConfig_t::CHANNEL_AIN1_GND;
        ads1115.ADS1115_updateConfig(ads1115Config);  // 原本为CHANNEL_AIN3_GND修改channel_AIN1_GND
        adcData[3] = ads1115.ADS1115_getData();       // 也不知道干嘛的，can发过去了但是主控板没用

        CurrentValue = (adcData[0] * 0.011972 - 155.485) * 10;

        vTaskDelay(10);
    }
}

static StackType_t mainTaskStack[128];
static StaticTask_t mainTaskTCB;
// volatile uint8_t PA6;
// volatile uint8_t PB0;
// volatile uint8_t PB1;
// volatile uint8_t PB2;
// volatile uint8_t PB5;
// volatile uint8_t PB4;
// volatile uint8_t PB3;
// volatile uint8_t PA15;
//char *buf = "Hello UART!\r\n";
void mainTask(void *param)
{

    // PA13用于确定前后分线板 开发板上有拨动开关
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != GPIO_PIN_RESET)//硬件上 打到前 前是低电平 后是3.3高电平 和程序有点反 25-9-22
        boardPosition = BoardPosition::Front;//代码的front是变轨 其实物理上是后板对应变轨
    else
        boardPosition = BoardPosition::Back;

    CANinit();

    // 应对急停按下电源重新上电：发送紧急信号，主控板收到后机器停止
    uint8_t Emergency_Signal = 1; 
    TxData[2] = Emergency_Signal & 0xFF;
    for(uint8_t i = 0; i < 3; i++){
        
        HAL_CAN_AddTxMessage(&hcan, &FrontTxHeader, TxData, &TxMailbox);
        vTaskDelay(10);

    }
    Emergency_Signal = 0; 
    TxData[2] = Emergency_Signal & 0xFF;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    
    while (1)
    {
        //HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
        // PVT2光耦
        photoCouplers = 0;
        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 6;  // 升降电机磁感（RM2006电机） PA6 触发高电平，到底部触发
        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 7;  // 条形激光，PB0 遇到钢筋时触发高电平（距离近触发）
        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 2;  // 横移磁感 后（左）PB1 触发高电平  前（左）PB2 触发高电平
        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 3;  // 横移磁感 后（右）PB2 触发高电平  前（右）PB1 触发高电平
        //        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);   //
        //        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);   //
        //        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);   //
        //        photoCouplers = photoCouplers | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  //

        //        PA6 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        //        PB0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        //        PB1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        //        PB2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
        //        PB5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
        //        PB4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        //        PB3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
        //        PA15 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

        bumperSensor = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);                // 防撞条没用
        bFaultPin    = HAL_GPIO_ReadPin(B_nFAULT_GPIO_Port, B_nFAULT_Pin);  
        bFaultPin = 0;     // 横移电机故障引脚
       /* 
        if(adcData[1] < 1080){//600-1800
            adcData[1] = 13153;  // 机器在地面，节点传感器信号值会干扰自动模式机器前后方向
        }
        */
        TxData[0] = adcData[1] >> 8;  // 前分线板：检测绑扎位置激光 ；  后分线板：检测换轨位置激光
        TxData[1] = adcData[1] & 0xFF;


        printf("TxData[0]: %d, TxData[1]: %d\r\n", TxData[0], TxData[1]);

        TxData[2] = bFaultPin >> 8;
        TxData[3] = bFaultPin & 0xFF;

        TxData[4] = ultrasonicValue >> 8;
        TxData[5] = ultrasonicValue & 0xFF;

        TxData[6] = photoCouplers;
        TxData[7] = BMotorState << 2 | AMotorState;

        if (boardPosition == BoardPosition::Front)
        {
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
                HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
            if (HAL_CAN_AddTxMessage(&hcan, &FrontTxHeader, TxData, &TxMailbox) != HAL_OK)
            {
                Error_Handler();
            }
        }
        else if (boardPosition == BoardPosition::Back)
        {
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
                HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
            if (HAL_CAN_AddTxMessage(&hcan, &BackTxHeader, TxData, &TxMailbox) != HAL_OK)
            {
                Error_Handler();
            }
        }

        if (electromagnetOn)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        }

        vTaskDelay(10);
    }
}

void backMotorProcess(
    B_MOTOR_CMD cmd, B_MOTOR_STATE state, GPIO_TypeDef *leftGPIOType, uint16_t leftGPIOPin, GPIO_TypeDef *rightGPIOType, uint16_t rightGPIOPin);

static StackType_t BmotorTaskStack[128];
static StaticTask_t BmotorTaskTCB;
void BmotorTask(void *param)
{
    vTaskDelay(1000);
    /*初始状态时轮子着地，（正对机器时人的左右）前分线板电机往右走，相当于机身和支撑腿朝左；
     * 当触发换轨时，支撑腿着地，此时电机只能往左走，相当于机身和轮子朝右，（这里朝右等于以机身为正向的朝左）
     */
    if (boardPosition == BoardPosition::Front)
    {
        BMotorCMD   = B_MOTOR_CMD::GoRight;
        BMotorState = B_MOTOR_STATE::BMotorMiddle;
    }
    else if (boardPosition == BoardPosition::Back)
    {
        BMotorCMD   = B_MOTOR_CMD::GoLeft;
        BMotorState = B_MOTOR_STATE::BMotorMiddle;
    }
    while (1)
    {
        if (boardPosition == BoardPosition::Front)
        {
            backMotorProcess(BMotorCMD, BMotorState, GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_1);  // 前边横移电机控制
        }
        else if (boardPosition == BoardPosition::Back)
        {
            backMotorProcess(BMotorCMD, BMotorState, GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_1);  // 后边横移电机控制
        }
        vTaskDelay(1);
    }
}

/**
 * @param cmd           B_MOTOR_CMD::GoLeft;
 * @param state         B_MOTOR_STATE::BMotorMiddle;
 * @param leftGPIOType  GPIOB
 * @param leftGPIOPin   GPIO_PIN_2  左磁感
 * @param rightGPIOType GPIOB
 * @param rightGPIOPin  GPIO_PIN_1  右磁感
 *
 * PH和EN都是高电平时刹车 2025-03-18
 */
void backMotorProcess(
    B_MOTOR_CMD cmd, B_MOTOR_STATE state, GPIO_TypeDef *leftGPIOType, uint16_t leftGPIOPin, GPIO_TypeDef *rightGPIOType, uint16_t rightGPIOPin)
{
    // 停止
    if (cmd == B_MOTOR_CMD::BMotorStop && state != B_MOTOR_STATE::BMotorMiddle)
    {
        HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
        BMotorState = B_MOTOR_STATE::BMotorMiddle;
        return;
    }
    // 横移电机往左移动
    else if (cmd == B_MOTOR_CMD::GoLeft && state != B_MOTOR_STATE::Left)
    {
        /*往左走：PH高电平，EN低电平*/
        HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_RESET);
        /*注意限位开关的pin是可以通过接口更改的，千万不要随意更改接口*/
        while (HAL_GPIO_ReadPin(leftGPIOType, leftGPIOPin) == GPIO_PIN_RESET)
        {
            if (HAL_GPIO_ReadPin(B_nFAULT_GPIO_Port, B_nFAULT_Pin) == GPIO_PIN_SET || BMotorCMD == B_MOTOR_CMD::BMotorStop)
            {
                HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
                BMotorState = B_MOTOR_STATE::BMotorMiddle;
                return;
            }
            vTaskDelay(1);
        }
         HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
         vTaskDelay(10);
         HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
        
        BMotorState = B_MOTOR_STATE::Left;
        return;
    }
    // 横移电机往右移动
    else if (cmd == B_MOTOR_CMD::GoRight && state != B_MOTOR_STATE::Right)
    {
        /*往右走：PH低电平，EN高电平*/
        HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
        while (HAL_GPIO_ReadPin(rightGPIOType, rightGPIOPin) == GPIO_PIN_RESET)
        {
            if (HAL_GPIO_ReadPin(B_nFAULT_GPIO_Port, B_nFAULT_Pin) == GPIO_PIN_SET || BMotorCMD == B_MOTOR_CMD::BMotorStop)
            {
                /*停止：EN和PH都为高电平*/
                HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
                BMotorState = B_MOTOR_STATE::BMotorMiddle;
                return;
            }
            vTaskDelay(1);
        }
        // 走到右边触发限位开关后再往回走一点
        /*往左走：PH高电平，EN低电平*/
         HAL_GPIO_WritePin(B_PH_GPIO_Port, B_PH_Pin, GPIO_PIN_SET);
         HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
         vTaskDelay(10);
         HAL_GPIO_WritePin(B_EN_GPIO_Port, B_EN_Pin, GPIO_PIN_SET);
        BMotorState = B_MOTOR_STATE::Right;
        return;
    }
}

// 灯带LED个数
#define NUM_LEDS 50
void updateLEDStatus()
{
    if (needRedLightOn)
    {
        RGB_RED(NUM_LEDS);
        vTaskDelay(5000);
    }
    else
    {
        RGB_THEME(NUM_LEDS);
        vTaskDelay(5000);
    }
}

static StackType_t ledTaskStack[128];
static StaticTask_t ledTaskTCB;

void ledTask(void *param)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    while (1)
    {
        updateLEDStatus();
    }
}

// 函数来设置 needRedLightOn 和 needRedLightFlashing 并通知 ledTask
void setLedStatus(bool on, bool flashing)
{
    needRedLightOn       = on;
    needRedLightFlashing = flashing;
}

static StackType_t ultrasonicTaskStack[64];
static StaticTask_t ultrasonicTaskTCB;
void ultrasonicTask(void *param)
{
    while (1)
    {
        ultrasonicValue = dypa21.readData();
        vTaskDelay(1);
    }
}

void startUserTasks()
{
    xTaskCreateStatic(i2cTask, "i2c task", 128, NULL, 7, i2cTaskStack, &i2cTaskTCB);
    xTaskCreateStatic(mainTask, "main task", 128, NULL, 5, mainTaskStack, &mainTaskTCB);
    xTaskCreateStatic(BmotorTask, "B motor task", 128, NULL, 6, BmotorTaskStack, &BmotorTaskTCB);
    // xTaskCreateStatic(ledTask, "led task", 128, NULL, 0, ledTaskStack, &ledTaskTCB);
    xTaskCreateStatic(ultrasonicTask, "ultrasonicTaskStack", 64, NULL, 4, ultrasonicTaskStack, &ultrasonicTaskTCB);
}
