#include "i6x_receive.h"
#include "i6x.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

static uint8_t sbus_rx_buf[I6X_FRAME_LENGTH];

void i6x_remote_init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart3, sbus_rx_buf, I6X_FRAME_LENGTH);

    // 关闭半传输中断，避免不必要的 DMA 中断
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void i6x_remote_uart_idle_handler(void)
{
    uint16_t rx_len;

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        rx_len = I6X_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        HAL_UART_DMAStop(&huart3);

        if (rx_len == I6X_FRAME_LENGTH)
        {
            sbus_to_i6x(get_i6x_point(), sbus_rx_buf);
        }

        HAL_UART_Receive_DMA(&huart3, sbus_rx_buf, I6X_FRAME_LENGTH);
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}