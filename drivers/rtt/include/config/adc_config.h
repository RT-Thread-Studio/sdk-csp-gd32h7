/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-10-15     LZerro     compatible with RT-Studio
 */

#ifndef __ADC_CONFIG_H__
#define __ADC_CONFIG_H__

#include <rtthread.h>
#include <board.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "drv_adc.h"

#ifdef RT_USING_ADC
#define MAX_EXTERN_ADC_CHANNEL 20
#endif

#ifdef BSP_USING_ADC0
#ifndef ADC0_CONFIG
#define ADC0_CONFIG         \
    {                       \
        ADC0,               \
        RCU_ADC0,           \
        {                   \
            GET_PIN(A, 0),  \
            GET_PIN(A, 1),  \
            GET_PIN(F, 11), \
            GET_PIN(A, 6),  \
            GET_PIN(C, 4),  \
            GET_PIN(B, 1),  \
            GET_PIN(F, 12), \
            GET_PIN(A, 7),  \
            GET_PIN(C, 5),  \
            GET_PIN(B, 0),  \
            GET_PIN(C, 0),  \
            GET_PIN(C, 1),  \
            GET_PIN(C, 2),  \
            GET_PIN(C, 13), \
            GET_PIN(A, 2),  \
            GET_PIN(A, 3),  \
            GET_PIN(A, 0),  \
            GET_PIN(A, 1),  \
            GET_PIN(A, 18), \
            GET_PIN(A, 19), \
        },                  \
        &adc0,              \
        "adc0",             \
    }
#endif /* ADC0_CONFIG */
#endif /* BSP_USING_ADC0 */

#ifdef BSP_USING_ADC1
#ifndef ADC1_CONFIG
#define ADC1_CONFIG         \
    {                       \
        ADC1,               \
        RCU_ADC1,           \
        {                   \
            GET_PIN(A, 0),  \
            GET_PIN(A, 1),  \
            GET_PIN(F, 13), \
            GET_PIN(A, 6),  \
            GET_PIN(C, 4),  \
            GET_PIN(B, 1),  \
            GET_PIN(F, 14), \
            GET_PIN(A, 7),  \
            GET_PIN(C, 5),  \
            GET_PIN(B, 0),  \
            GET_PIN(C, 0),  \
            GET_PIN(C, 1),  \
            GET_PIN(C, 2),  \
            GET_PIN(C, 13), \
            GET_PIN(A, 2),  \
            GET_PIN(A, 3),  \
            -1,             \
            -1,             \
            GET_PIN(A, 18), \
            GET_PIN(A, 19), \
        },                  \
        &adc1,              \
        "adc1",             \
    }
#endif /* ADC1_CONFIG */
#endif /* BSP_USING_ADC1 */

#ifdef BSP_USING_ADC2
#ifndef ADC2_CONFIG
#define ADC2_CONFIG         \
    {                       \
        ADC2,               \
        RCU_ADC2,           \
        {                   \
            GET_PIN(C, 2),  \
            GET_PIN(C, 3),  \
            GET_PIN(F, 9),  \
            GET_PIN(F, 7),  \
            GET_PIN(F, 5),  \
            GET_PIN(F, 3),  \
            GET_PIN(F, 10), \
            GET_PIN(F, 8),  \
            GET_PIN(F, 6),  \
            GET_PIN(F, 4),  \
            GET_PIN(C, 0),  \
            GET_PIN(C, 1),  \
            GET_PIN(C, 2),  \
            GET_PIN(H, 2),  \
            GET_PIN(H, 3),  \
            GET_PIN(H, 4),  \
            GET_PIN(H, 5),  \
            -1,             \
            -1,             \
            -1,             \
        },                  \
        &adc2,              \
        "adc2",             \
    }
#endif /* ADC2_CONFIG */
#endif /* BSP_USING_ADC2 */

#ifdef __cplusplus
}
#endif

#endif // __ADC_CONFIG_H__
