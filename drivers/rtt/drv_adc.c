/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2025-10-15     LZerro            first version
 */

#include "drv_adc.h"
#include "adc_config.h"

#define DBG_TAG "drv.adc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_ADC

#if defined(BSP_USING_ADC0)
struct rt_adc_device adc0;
#endif

#if defined(BSP_USING_ADC1)
struct rt_adc_device adc1;
#endif

#if defined(BSP_USING_ADC2)
struct rt_adc_device adc2;
#endif

static const struct gd32_adc adc_obj[] = {
#ifdef BSP_USING_ADC0
    ADC0_CONFIG,
#endif

#ifdef BSP_USING_ADC1
    ADC1_CONFIG,
#endif

#ifdef BSP_USING_ADC2
    ADC2_CONFIG,
#endif
};

/**
 * @brief ADC MSP Initialization
 *        This function configures the hardware resources.
 * @param adc_clk, pin
 * @retval None
 */
static void gd32_adc_gpio_init(rcu_periph_enum adc_clk, rt_base_t pin)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(adc_clk);

#if defined SOC_SERIES_GD32F4xx || defined SOC_SERIES_GD32E23x || defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
    /* configure adc pin */
    gpio_mode_set(PIN_GDPORT(pin), GPIO_MODE_ANALOG, GPIO_PUPD_NONE, PIN_GDPIN(pin));
#else
    /* configure adc pin */
    gpio_init(PIN_GDPORT(pin), GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, PIN_GDPIN(pin));
#endif
}

/**
 * @brief ADC enable
 *        This function enable adc.
 * @param device, channel, enabled
 * @retval None
 */
static rt_err_t gd32_adc_enabled(struct rt_adc_device *device, rt_int8_t channel, rt_bool_t enabled)
{
    uint32_t         adc_periph;
    struct gd32_adc *adc = (struct gd32_adc *)device->parent.user_data;

    if (channel >= MAX_EXTERN_ADC_CHANNEL)
    {
        LOG_E("invalid channel");
        return -RT_EINVAL;
    }

    adc_periph = (uint32_t)(adc->adc_periph);

    if (enabled == ENABLE)
    {
#if defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        // 先关闭定时器
        adc_disable(adc_periph);
#endif

        // 初始化引脚
        gd32_adc_gpio_init(adc->adc_clk, adc->adc_pins[channel]);

#if defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        // 配置时钟, ADC0/1最高时钟配置位72M，ADC2为80M
        adc_clock_config(adc_periph, ADC_CLK_SYNC_HCLK_DIV6);

        // 设置ADC采样率
        adc_resolution_config(adc_periph, ADC_RESOLUTION_12B);
#endif

        // 设置ADC数据对齐模式
#if defined SOC_SERIES_GD32E23x
        adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
#else
        adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
#endif

        // 设置为规则通道
#if defined SOC_SERIES_GD32F4xx
        adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, 1);
        adc_external_trigger_source_config(adc_periph, ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_EXTI_11);
        adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, ENABLE);
#elif defined SOC_SERIES_GD32E23x
        adc_channel_length_config(ADC_REGULAR_CHANNEL, 1);
        adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
        adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
#elif defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1);
#else
        adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1);
        adc_external_trigger_source_config(adc_periph, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
        adc_external_trigger_config(adc_periph, ADC_REGULAR_CHANNEL, ENABLE);
#endif

        // 使能ADC
#if defined SOC_SERIES_GD32E23x
        adc_enable();
#else
        adc_enable(adc_periph);
#endif

        // 延时等待ADC启动
        rt_thread_mdelay(1);

        // 校准ADC
#if defined SOC_SERIES_GD32E23x
        adc_calibration_enable();
#elif defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        adc_calibration_mode_config(adc_periph, ADC_CALIBRATION_OFFSET);
        adc_calibration_number(adc_periph, ADC_CALIBRATION_NUM1);
        adc_calibration_enable(adc_periph);
#else
        adc_calibration_enable(adc_periph);
#endif
    }
    else
    {
#if defined SOC_SERIES_GD32E23x
        adc_disable();
#else
        adc_disable(adc_periph);
#endif
    }
    return RT_EOK;
}

/**
 * @brief convert adc.
 *        This function get adc value.
 * @param device, channel, value
 * @retval None
 */
static rt_err_t gd32_adc_convert(struct rt_adc_device *device, rt_int8_t channel, rt_uint32_t *value)
{
    uint32_t         adc_periph;
    uint32_t         timeout = 0;
    struct gd32_adc *adc     = (struct gd32_adc *)(device->parent.user_data);

    if (!value)
    {
        LOG_E("invalid param");
        return -RT_EINVAL;
    }

    adc_periph = (uint32_t)(adc->adc_periph);

    // 清除标志位
#if defined SOC_SERIES_GD32E23x
    adc_flag_clear(ADC_FLAG_EOC | ADC_FLAG_STRC);
#else
    adc_flag_clear(adc_periph, ADC_FLAG_EOC | ADC_FLAG_STRC);
#endif

    // 配置通道
#if defined SOC_SERIES_GD32F4xx
    adc_routine_channel_config(adc_periph, 0, channel, ADC_SAMPLETIME_480);
    adc_software_trigger_enable(adc_periph, ADC_ROUTINE_CHANNEL);
#elif defined SOC_SERIES_GD32E23x
    adc_regular_channel_config(0, channel, ADC_SAMPLETIME_13POINT5);
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#elif defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
    adc_regular_channel_config(adc_periph, 0, channel, 480);
    adc_software_trigger_enable(adc_periph, ADC_REGULAR_CHANNEL);
#else
    adc_regular_channel_config(adc_periph, 0, channel, ADC_SAMPLETIME_13POINT5);
    adc_software_trigger_enable(adc_periph, ADC_REGULAR_CHANNEL);
#endif

    // 等待转化完成
#if defined SOC_SERIES_GD32E23x
    while (!adc_flag_get(ADC_FLAG_EOC))
#else
    while (!adc_flag_get(adc_periph, ADC_FLAG_EOC))
#endif
    {
        if (timeout >= 100)
        {
#if defined SOC_SERIES_GD32E23x
            adc_flag_clear(ADC_FLAG_EOC | ADC_FLAG_STRC);
#else
            adc_flag_clear(adc_periph, ADC_FLAG_EOC | ADC_FLAG_STRC);
#endif
            LOG_E("Convert Timeout");
            return -RT_ETIMEOUT;
        }

        timeout++;
        rt_thread_delay(1);
    }

    // 读取数值
#if defined SOC_SERIES_GD32F4xx
    *value = adc_routine_data_read(adc_periph);
    adc_flag_clear(adc_periph, ADC_FLAG_EOC | ADC_FLAG_STRC);
#elif defined SOC_SERIES_GD32E23x
    *value = adc_regular_data_read();
    adc_flag_clear(ADC_FLAG_EOC | ADC_FLAG_STRC);
#else
    *value = adc_regular_data_read(adc_periph);
    adc_flag_clear(adc_periph, ADC_FLAG_EOC | ADC_FLAG_STRC);
#endif

    return RT_EOK;
}

static struct rt_adc_ops gd32_adc_ops = {
    .enabled = gd32_adc_enabled,
    .convert = gd32_adc_convert,
};

static int rt_hw_adc_init(void)
{
    int ret, i = 0;

    syscfg_analog_switch_enable(SYSCFG_PA0_ANALOG_SWITCH);
    syscfg_analog_switch_enable(SYSCFG_PA1_ANALOG_SWITCH);
    syscfg_analog_switch_enable(SYSCFG_PC2_ANALOG_SWITCH);
    syscfg_analog_switch_enable(SYSCFG_PC3_ANALOG_SWITCH);

    for (; i < sizeof(adc_obj) / sizeof(adc_obj[0]); i++)
    {
        ret = rt_hw_adc_register(adc_obj[i].adc,
                                 (const char *)adc_obj[i].device_name,
                                 &gd32_adc_ops, &adc_obj[i]);
        if (ret != RT_EOK)
        {
            /* TODO err handler */
            LOG_E("failed register %s, err=%d", adc_obj[i].device_name, ret);
        }
    }

    return ret;
}
INIT_BOARD_EXPORT(rt_hw_adc_init);
#endif
