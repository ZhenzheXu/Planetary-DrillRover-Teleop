#include "i6x.h"

#define TO_STICK(v) (((v) < 0) - ((v) > 0))
#define MAPPING_ENABLE 1

i6x_ctrl_t i6x_ctrl;

/**
 * @brief 将原始通道值 -784~783 映射到 -660~660
 */
static int16_t map_to_660(int16_t val)
{
    int32_t temp;

    if (val >= 0)
    {
        temp = ((int32_t)val * 660 + 391) / 783;
    }
    else
    {
        temp = ((int32_t)val * 660 - 392) / 784;
    }

    return (int16_t)temp;
}

/**
 * @brief SBUS数据解包为i6x遥控器数据
 * @param i6x_ctrl 解包后的遥控器数据结构体
 * @param sbus_data 25字节SBUS原始数据
 */
void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data)
{
    uint8_t i;
    uint8_t flag;

    if (i6x_ctrl == 0 || sbus_data == 0)
    {
        return;
    }

    // SBUS帧头帧尾检查
    if (sbus_data[0] != 0x0F || sbus_data[24] != 0x00)
    {
        return;
    }

    i6x_ctrl->ch[0] = (int16_t)(((sbus_data[1] | (sbus_data[2] << 8)) & 0x07FF) - 1024);

    i6x_ctrl->ch[1] = (int16_t)((((sbus_data[2] >> 3) | (sbus_data[3] << 5)) & 0x07FF) - 1024);

    i6x_ctrl->ch[2] = (int16_t)((((sbus_data[3] >> 6) | (sbus_data[4] << 2) |
                                  (sbus_data[5] << 10)) & 0x07FF) - 1024);

    i6x_ctrl->ch[3] = (int16_t)((((sbus_data[5] >> 1) | (sbus_data[6] << 7)) & 0x07FF) - 1024);

    i6x_ctrl->ch[4] = (int16_t)((((sbus_data[6] >> 4) | (sbus_data[7] << 4)) & 0x07FF) - 1024);

    i6x_ctrl->ch[5] = (int16_t)((((sbus_data[7] >> 7) | (sbus_data[8] << 1) |
                                  (sbus_data[9] << 9)) & 0x07FF) - 1024);

    i6x_ctrl->s[0] = (int8_t)TO_STICK(((((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF) - 1024));

    i6x_ctrl->s[1] = (int8_t)TO_STICK(((((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF) - 1024));

    i6x_ctrl->s[2] = (int8_t)TO_STICK((((sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF) - 1024));

    i6x_ctrl->s[3] = (int8_t)TO_STICK(((((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF) - 1024));

#if MAPPING_ENABLE
    for (i = 0; i < 6; i++)
    {
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);
    }
#endif

    flag = sbus_data[23];
    i6x_ctrl->frame_lost = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe   = (flag >> 3) & 0x01;
}

/**
 * @brief 获取遥控器数据结构体指针
 */
i6x_ctrl_t *get_i6x_point(void)
{
    return &i6x_ctrl;
}