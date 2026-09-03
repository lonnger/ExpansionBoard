#include "DYPA21_PWM.hpp"
#if USE_DYPA21_PWM
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

namespace DYPA21_PWM
{         //DYPA21_PWM::DYPA21_PWM将传入的定时器句柄保存到类的成员变量中，以便后续使用该定时器进行测距操作
DYPA21_PWM::DYPA21_PWM(TIM_HandleTypeDef *tim_handle) { this->tim_handle = tim_handle; }

static volatile uint8_t tim4Ch2CaptureState = 0;
static volatile uint16_t tim4Ch2CaptureValue = 0;
static uint32_t distanceMm = 0;

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
    {
        return;
    }

    if ((tim4Ch2CaptureState & 0x80) == 0)
    {   
        if (tim4Ch2CaptureState & 0X40)  // 逻辑判断：如果现在 Bit 6 为 1，说明之前已经抓到了上升沿，那么现在这次中断必然是【下降沿】
        {
            // --- 阶段 2：检测到下降沿（脉冲结束） ---
            // 读取此刻定时器的值（这就存下了高电平持续了多少个 Tick）
            tim4Ch2CaptureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            // 捕获值写入完成后再发布完成标志
            tim4Ch2CaptureState |= 0X80;
            
            // 清除之前的边沿设置，重新配置为【上升沿】捕获，为下一次测距做准备
            TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
            TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
        }
        else
        {
        // --- 阶段 1：检测到上升沿（脉冲开始） ---
            tim4Ch2CaptureState = 0;      // 清空所有状态标志
            tim4Ch2CaptureValue = 0;      // 清空上一次的捕获值
            tim4Ch2CaptureState |= 0X40;  // 标记 Bit 6 = 1，表示：我已经抓到起点了，正在计时中...

            __HAL_TIM_DISABLE(htim);  // 临时关闭定时器，确保重置过程不受干扰
            __HAL_TIM_SET_COUNTER(htim, 0); // 将定时器计数器（CNT）清零，从 0 开始重新数数
            
            // 关键动作：修改捕获极性，改为捕获【下降沿】，这样下次跳变才能触发上面的“阶段 2”
            TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
            TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
            
            __HAL_TIM_ENABLE(htim);   // 重新开启定时器，开始计时
        }
    }
}

extern "C" void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }

/**
 * @brief  主动触发一次超声波测距并读取结果
 * @return int16_t 返回计算后的距离值（单位通常为 mm）
 */
int16_t DYPA21_PWM::readData()
{
    // --- 阶段 1：发送触发脉冲 (Trigger) ---
    // 先拉低引脚，确保处于干净的低电平状态
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(5); // 等待 5ms，确保电平稳定
    
    // 拉高引脚，向超声波模块发送一个“开始测量”的指令信号
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    
    // --- 阶段 2：等待回波 (Waiting) ---
    // 释放 CPU 权限 160ms（FreeRTOS 任务延时），给声波往返和中断函数留出足够的“跑数”时间
    vTaskDelay(160); 

    // --- 阶段 3：处理捕获到的原始数据 ---
    // 检查状态位的最高位 (Bit 7 / 0x80) 是否为 1，即确认中断函数是否已经抓到了完整的下降沿
    if (tim4Ch2CaptureState & 0X80)
    {
        // TIM4 以 1 MHz 计数；直接使用单周期内捕获的脉宽。
        // 本驱动不再读取未实现的“溢出次数”状态位。
        const uint32_t captureTicks = tim4Ch2CaptureValue;

        // 将“时间（Tick）”转换为“距离（mm）”
        // 5.75 是一个转换系数，它结合了声速(340m/s)和定时器的频率
        distanceMm = captureTicks / 5.75;
        
        // --- 阶段 4：复位并收工 ---
        // 将状态标志清零，允许下一次测距任务能够重新开始计时
        tim4Ch2CaptureState = 0;
    }

    // 返回最后计算出的距离（如果本次没抓到新数据，则返回上一次的旧值）
    return distanceMm;
}

}  // namespace DYPA21_PWM
#endif
