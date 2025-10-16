/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-11-09     xiangxistu   first version
 * 2020-05-18     chenyaxing   modify uart_config struct
 * 2025-09-11     WangShun     fully adapted for GD32H7xx with SPL
 * 2025-10-16     kurisaw      general-purpose configuration
 */

#include <string.h>
#include <stdlib.h>
#include "uart_config.h"

#ifdef SOC_SERIES_GD32H75E
#include <gd32h75e.h>
#else
#include <gd32h7xx.h>
#endif

/* GD32 UART config structure */
struct gd32_uart_config
{
    const char *name;
    uint32_t instance; /* USART peripheral address */
    IRQn_Type irq_type;
    const char *tx_pin_name;
    const char *rx_pin_name;
};

static struct gd32_uart_config *_uart_config = NULL;

/* UART configuration array */
struct gd32_uart_config uart_config[] =
{
#ifdef BSP_USING_UART0
    UART0_CONFIG,
#endif
#ifdef BSP_USING_UART1
    UART1_CONFIG,
#endif
#ifdef BSP_USING_UART2
    UART2_CONFIG,
#endif
#ifdef BSP_USING_UART3
    UART3_CONFIG,
#endif
#ifdef BSP_USING_UART4
    UART4_CONFIG,
#endif
#ifdef BSP_USING_UART5
    UART5_CONFIG,
#endif
#ifdef BSP_USING_UART6
    UART6_CONFIG,
#endif
#ifdef BSP_USING_UART7
    UART7_CONFIG,
#endif
};

static long gd32_uart_clk_enable(struct gd32_uart_config *config)
{
    /* Enable UART peripheral clock */
    switch (config->instance)
    {
    case USART0:
        rcu_periph_clock_enable(RCU_USART0);
        break;
    case USART1:
        rcu_periph_clock_enable(RCU_USART1);
        break;
    case USART2:
        rcu_periph_clock_enable(RCU_USART2);
        break;
    case UART3:
        rcu_periph_clock_enable(RCU_UART3);
        break;
    case UART4:
        rcu_periph_clock_enable(RCU_UART4);
        break;
    case USART5:
        rcu_periph_clock_enable(RCU_USART5);
        break;
    case UART6:
        rcu_periph_clock_enable(RCU_UART6);
        break;
    case UART7:
        rcu_periph_clock_enable(RCU_UART7);
        break;
    default:
        return -1;
    }

    return 0;
}

static long gd32_gpio_clk_enable(uint32_t gpiox)
{
    /* Enable GPIO peripheral clock */
    switch (gpiox)
    {
    case GPIOA:
        rcu_periph_clock_enable(RCU_GPIOA);
        break;
    case GPIOB:
        rcu_periph_clock_enable(RCU_GPIOB);
        break;
    case GPIOC:
        rcu_periph_clock_enable(RCU_GPIOC);
        break;
    case GPIOD:
        rcu_periph_clock_enable(RCU_GPIOD);
        break;
    case GPIOE:
        rcu_periph_clock_enable(RCU_GPIOE);
        break;
    case GPIOF:
        rcu_periph_clock_enable(RCU_GPIOF);
        break;
    case GPIOG:
        rcu_periph_clock_enable(RCU_GPIOG);
        break;
    case GPIOH:
        rcu_periph_clock_enable(RCU_GPIOH);
        break;
    default:
        return -1;
    }

    return 0;
}

static int up_char(char *c)
{
    if ((*c >= 'a') && (*c <= 'z'))
    {
        *c = *c - 32;
    }
    return 0;
}

static void get_pin_by_name(const char *pin_name, uint32_t *port, uint32_t *pin)
{
    int pin_num = atoi((char *)&pin_name[2]);
    char port_name = pin_name[1];
    up_char(&port_name);
    *port = (GPIOA + (uint32_t)(port_name - 'A') * (GPIOB - GPIOA));
    *pin = (1U << pin_num); /* GD32 uses 1 << n for pin */
}

static long gd32_gpio_configure(struct gd32_uart_config *config)
{
    uint32_t tx_port, rx_port;
    uint32_t tx_pin, rx_pin;
    uint8_t af;

    get_pin_by_name(config->tx_pin_name, &tx_port, &tx_pin);
    get_pin_by_name(config->rx_pin_name, &rx_port, &rx_pin);

    /* Enable GPIO clocks */
    gd32_gpio_clk_enable(tx_port);
    if (tx_port != rx_port)
    {
        gd32_gpio_clk_enable(rx_port);
    }

    /* Determine alternate function based on UART instance */
    switch (config->instance)
    {
    case USART0:
#if defined SOC_SERIES_GD32F4xx || defined SOC_SERIES_GD32F5xx || defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        af = GPIO_AF_4;
        break;
#elif defined SOC_SERIES_GD32E23x
        af = GPIO_AF_1;
        break;
#endif
    case USART1:
#if defined SOC_SERIES_GD32F4xx || defined SOC_SERIES_GD32F5xx || defined SOC_SERIES_GD32H7xx || defined SOC_SERIES_GD32H75E
        af = GPIO_AF_7;
        break;
#elif defined SOC_SERIES_GD32E23x
        af = GPIO_AF_1;
        break;
#endif
    case USART2:
        af = GPIO_AF_7;
        break;
    case UART3:
        af = GPIO_AF_8;
        break;
    case UART4:
        af = GPIO_AF_8;
        break;
    case USART5:
#if defined SOC_SERIES_GD32F4xx || defined SOC_SERIES_GD32F5xx
        af = GPIO_AF_8;
        break;
#elif defined (SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
        af = GPIO_AF_7;
        break;
#endif
    case UART6:
#if defined SOC_SERIES_GD32F4xx || defined SOC_SERIES_GD32F5xx
        af = GPIO_AF_8;
        break;
#elif defined (SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
        af = GPIO_AF_7;
        break;
#endif
    case UART7:
        af = GPIO_AF_8;
        break;
    default:
        return -1;
    }

    /* Configure TX pin */
    gpio_af_set(tx_port, af, tx_pin);
    gpio_mode_set(tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, tx_pin);
    gpio_output_options_set(tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, tx_pin);

    /* Configure RX pin */
    gpio_af_set(rx_port, af, rx_pin);
    gpio_mode_set(rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, rx_pin);
    gpio_output_options_set(rx_port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, rx_pin);

    return 0;
}

static long gd32_configure(struct gd32_uart_config *config)
{
    /* Enable UART clock */
    if (gd32_uart_clk_enable(config) != 0)
        return -1;

    /* Configure GPIO first */
    if (gd32_gpio_configure(config) != 0)
        return -1;

    /* Initialize UART */
    usart_deinit(config->instance);
    usart_baudrate_set(config->instance, 115200U);
    usart_word_length_set(config->instance, USART_WL_8BIT);
    usart_stop_bit_set(config->instance, USART_STB_1BIT);
    usart_parity_config(config->instance, USART_PM_NONE);
    usart_hardware_flow_rts_config(config->instance, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(config->instance, USART_CTS_DISABLE);
    usart_receive_config(config->instance, USART_RECEIVE_ENABLE);
    usart_transmit_config(config->instance, USART_TRANSMIT_ENABLE);

    /* Enable UART */
    usart_enable(config->instance);

    return 0;
}

int rt_hw_usart_init(void)
{
    _uart_config = &uart_config[0];
    if (_uart_config == NULL)
        return -1;

    return gd32_configure(_uart_config);
}

void print_char(char c)
{
    /* Wait until transmit buffer is empty */
    while (RESET == usart_flag_get(_uart_config->instance, USART_FLAG_TBE));

    /* Handle newline character */
    if (c == '\n')
    {
        usart_data_transmit(_uart_config->instance, '\r');
        while (RESET == usart_flag_get(_uart_config->instance, USART_FLAG_TBE));
    }

    /* Transmit character */
    usart_data_transmit(_uart_config->instance, (uint8_t)c);
}

/* Print string function */
void print_string(const char *str)
{
    while (*str)
    {
        print_char(*str++);
    }
}

/* Check if data is available */
int uart_data_available(void)
{
    return (usart_flag_get(_uart_config->instance, USART_FLAG_RBNE) != RESET);
}

/* Read character from UART */
char uart_get_char(void)
{
    if (uart_data_available())
    {
        return (char)usart_data_receive(_uart_config->instance);
    }
    return -1;
}
