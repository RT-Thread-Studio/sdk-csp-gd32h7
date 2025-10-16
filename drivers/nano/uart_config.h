/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-09-10     WangShun     first version
 */

#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include <rtthread.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UART0 configuration */
#if defined(BSP_USING_UART0)
#ifndef UART0_CONFIG
#define UART0_CONFIG                                                \
    {                                                               \
        .name = "uart0",                                            \
        .usart_periph = USART0,                                     \
        .irq_type = USART0_IRQn,                                    \
        .tx_pin_name = BSP_UART0_TX_PIN,                            \
        .rx_pin_name = BSP_UART0_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART1 */

/* UART1 configuration */
#if defined(BSP_USING_UART1)
#ifndef UART1_CONFIG
#define UART1_CONFIG                                                \
    {                                                               \
        .name = "uart1",                                            \
        .usart_periph = USART1,                                     \
        .irq_type = USART1_IRQn,                                    \
        .tx_pin_name = BSP_UART1_TX_PIN,                            \
        .rx_pin_name = BSP_UART1_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART1 */

/* UART2 configuration */
#if defined(BSP_USING_UART2)
#ifndef UART2_CONFIG
#define UART2_CONFIG                                                \
    {                                                               \
        .name = "uart2",                                            \
        .usart_periph = USART2,                                     \
        .irq_type = USART2_IRQn,                                    \
        .tx_pin_name = BSP_UART2_TX_PIN,                            \
        .rx_pin_name = BSP_UART2_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART2 */

/* UART3 configuration */
#if defined(BSP_USING_UART3)
#ifndef UART3_CONFIG
#define UART3_CONFIG                                                \
    {                                                               \
        .name = "uart3",                                            \
        .usart_periph = USART3,                                     \
        .irq_type = USART3_IRQn,                                    \
        .tx_pin_name = BSP_UART3_TX_PIN,                            \
        .rx_pin_name = BSP_UART3_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART3 */

/* UART4 configuration */
#if defined(BSP_USING_UART4)
#ifndef UART4_CONFIG
#define UART4_CONFIG                                                \
    {                                                               \
        .name = "uart4",                                            \
        .usart_periph = UART4,                                      \
        .irq_type = UART4_IRQn,                                     \
        .tx_pin_name = BSP_UART4_TX_PIN,                            \
        .rx_pin_name = BSP_UART4_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART4 */

/* UART5 configuration */
#if defined(BSP_USING_UART5)
#ifndef UART5_CONFIG
#define UART5_CONFIG                                                \
    {                                                               \
        .name = "uart5",                                            \
        .usart_periph = UART5,                                      \
        .irq_type = UART5_IRQn,                                     \
        .tx_pin_name = BSP_UART5_TX_PIN,                            \
        .rx_pin_name = BSP_UART5_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART5 */

/* UART6 configuration */
#if defined(BSP_USING_UART6)
#ifndef UART6_CONFIG
#define UART6_CONFIG                                                \
    {                                                               \
        .name = "uart6",                                            \
        .usart_periph = USART6,                                     \
        .irq_type = USART6_IRQn,                                    \
        .tx_pin_name = BSP_UART6_TX_PIN,                            \
        .rx_pin_name = BSP_UART6_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART6 */

/* UART7 configuration */
#if defined(BSP_USING_UART7)
#ifndef UART7_CONFIG
#define UART7_CONFIG                                                \
    {                                                               \
        .name = "uart7",                                            \
        .usart_periph = UART7,                                      \
        .irq_type = UART7_IRQn,                                     \
        .tx_pin_name = BSP_UART7_TX_PIN,                            \
        .rx_pin_name = BSP_UART7_RX_PIN,                            \
    }
#endif
#endif /* BSP_USING_UART7 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_CONFIG_H__ */

