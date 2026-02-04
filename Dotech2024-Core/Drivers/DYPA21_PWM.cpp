#include "DYPA21_PWM.hpp"
#if USE_DYPA21_PWM
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

namespace DYPA21_PWM
{
DYPA21_PWM::DYPA21_PWM(TIM_HandleTypeDef *tim_handle) { this->tim_handle = tim_handle; }

uint8_t TiM2CH1_CAP_STATE;
uint16_t TiM2CH1_CAP_VALUE;
volatile uint32_t temp     = 0;
volatile uint32_t distance = 0;

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((TiM2CH1_CAP_STATE & 0x80) == 0)
    {
        if (TiM2CH1_CAP_STATE & 0X40)  // ����һ���½���
        {
            TiM2CH1_CAP_STATE |= 0X80;                                              // ��ǳɹ�����һ�θߵ�ƽ����
            TiM2CH1_CAP_VALUE = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);   // ��ȡ��ǰ�ļ�����ֵ
            TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2);                       // ���ԭ��������
            TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);  // ���������ز���
        }
        else
        {
            TiM2CH1_CAP_STATE = 0;                                                   // ����Զ����״̬�Ĵ���
            TiM2CH1_CAP_VALUE = 0;                                                   // ��ղ���ֵ
            TiM2CH1_CAP_STATE |= 0X40;                                               // ��ǲ���������
            __HAL_TIM_DISABLE(&htim4);                                               // �رն�ʱ��
            __HAL_TIM_SET_COUNTER(&htim4, 0);                                        // ������ֵ����
            TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2);                        // һ��Ҫ�����ԭ�������ã���
            TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);  // �����½��ز���
            __HAL_TIM_ENABLE(&htim4);                                                // ʹ�ܶ�ʱ��
        }
    }
}

extern "C" void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }

int16_t DYPA21_PWM::readData()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    vTaskDelay(160);
    if (TiM2CH1_CAP_STATE & 0X80)  // ���һ�θߵ�ƽ����
    {
        temp = TiM2CH1_CAP_STATE & 0X3F;
        temp *= 65536;              // �����ʱ��
        temp += TiM2CH1_CAP_VALUE;  // �ܵĸߵ�ƽʱ��
        distance          = temp / 5.75;
        TiM2CH1_CAP_STATE = 0;  // ׼����һ�β���
    }
    return distance;
}

}  // namespace DYPA21_PWM
#endif