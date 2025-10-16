/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-20     BruceOu      first implementation
 * 2025-09-29     WangShun     add drv_config.h
 */
#include "board.h"
#include <stdlib.h>
#include "drv_usart.h"
#include "uart_config.h"

#ifdef RT_USING_SERIAL

#if !defined(BSP_USING_UART0) && !defined(BSP_USING_UART1) && \
    !defined(BSP_USING_UART2) && !defined(BSP_USING_UART3) && \
    !defined(BSP_USING_UART4) && !defined(BSP_USING_UART5) && \
    !defined(BSP_USING_UART6) && !defined(BSP_USING_UART7)
#error "Please define at least one UARTx"

#endif

#include <rtdevice.h>

#ifdef RT_SERIAL_USING_DMA
static void gd32_dma_config(struct rt_serial_device *serial, rt_ubase_t flag);
static void gd32_dma_tx_config(struct rt_serial_device *serial, rt_ubase_t flag);
static void dma_rx_done_isr(struct rt_serial_device *serial);

extern void Error_Handler(void);
#endif

static void GD32_UART_IRQHandler(struct rt_serial_device *serial);

#if defined(BSP_USING_UART0)
struct rt_serial_device serial0;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart0_rxdma = {
    DMA0,
    DMA_CH0,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART0_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel0_IRQn,
    0,
};
gd32_uart_dma uart0_txdma = {
    DMA1,
    DMA_CH0,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART0_TX,
#endif
    DMA_INTF_FTFIF,
    DMA1_Channel0_IRQn,
    0,
};

void DMA0_Channel0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial0);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_FTF);
    dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_HTF);
    dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_FEE);
    dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_TAE);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void USART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial0);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART0 */

#if defined(BSP_USING_UART1)
struct rt_serial_device serial1;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart1_rxdma = {
    DMA0,
    DMA_CH1,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART1_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel1_IRQn,
    0,
};

void DMA0_Channel1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART1 */

#if defined(BSP_USING_UART2)
struct rt_serial_device serial2;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart2_rxdma = {
    DMA0,
    DMA_CH2,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART2_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel2_IRQn,
    0,
};
gd32_uart_dma uart2_txdma = {
    DMA1,
    DMA_CH2,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART2_TX,
#endif
    DMA_INTF_FTFIF,
    DMA1_Channel2_IRQn,
    0,
};

void DMA0_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_FTF);
    dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_HTF);
    dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_FEE);
    dma_flag_clear(DMA1, DMA_CH2, DMA_FLAG_TAE);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART2 */

#if defined(BSP_USING_UART3)
struct rt_serial_device serial3;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart3_rxdma = {
    DMA0,
    DMA_CH3,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_UART3_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel3_IRQn,
    0,
};

void DMA0_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void UART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART3 */

#if defined(BSP_USING_UART4)
struct rt_serial_device serial4;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart4_rxdma = {
    DMA0,
    DMA_CH4,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_UART4_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel4_IRQn,
    0,
};

void DMA0_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_UART4 */

#if defined(BSP_USING_UART5)
struct rt_serial_device serial5;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart5_rxdma = {
    DMA0,
    DMA_CH5,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_USART5_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel5_IRQn,
    0,
};

void DMA0_Channel5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void USART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART5 */

#if defined(BSP_USING_UART6)
struct rt_serial_device serial6;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart6_rxdma = {
    DMA0,
    DMA_CH6,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_UART6_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel6_IRQn,
    0,
};

void DMA0_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial6);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void UART6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial6);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART6 */

#if defined(BSP_USING_UART7)
struct rt_serial_device serial7;

#if defined(RT_SERIAL_USING_DMA)
gd32_uart_dma uart7_rxdma = {
    DMA0,
    DMA_CH7,
#if defined(SOC_SERIES_GD32H7xx) || defined(SOC_SERIES_GD32H75E)
    DMA_REQUEST_UART7_RX,
#endif
    DMA_INTF_FTFIF,
    DMA0_Channel7_IRQn,
    0,
};

void DMA0_Channel7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial7);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_SERIAL_USING_DMA */

void UART7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    GD32_UART_IRQHandler(&serial7);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* BSP_USING_UART7 */

static const struct gd32_uart uart_obj[] =
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

int get_uart_pin_config(const char *pin_name, uint32_t *port, uint32_t *pin, rcu_periph_enum *clk)
{
    if (pin_name == NULL || port == NULL || pin == NULL || clk == NULL)
    {
        return -1;
    }

    if (strlen(pin_name) < 3 || pin_name[0] != 'P')
    {
        return -1;
    }

    char port_letter = pin_name[1];
    switch (port_letter)
    {
#ifdef GPIOA
        case 'A':
            *port = GPIOA;
            *clk = RCU_GPIOA;
            break;
#endif /* GPIOA */
#ifdef GPIOB
        case 'B':
            *port = GPIOB;
            *clk = RCU_GPIOB;
            break;
#endif /* GPIOB */
#ifdef GPIOC
        case 'C':
            *port = GPIOC;
            *clk = RCU_GPIOC;
            break;
#endif /* GPIOC */
#ifdef GPIOD
        case 'D':
            *port = GPIOD;
            *clk = RCU_GPIOD;
            break;
#endif /* GPIOD */
#ifdef GPIOE
        case 'E':
            *port = GPIOE;
            *clk = RCU_GPIOE;
            break;
#endif /* GPIOE */
#ifdef GPIOF
        case 'F':
            *port = GPIOF;
            *clk = RCU_GPIOF;
            break;
#endif /* GPIOF */
#ifdef GPIOG
        case 'G':
            *port = GPIOG;
            *clk = RCU_GPIOG;
            break;
#endif /* GPIOG */
#ifdef GPIOH
        case 'H':
            *port = GPIOH;
            *clk = RCU_GPIOH;
            break;
#endif /* GPIOH */
#ifdef GPIOI
        case 'I':
            *port = GPIOI;
            *clk = RCU_GPIOI;
            break;
#endif /* GPIOI */
#ifdef GPIOJ
        case 'J':
            *port = GPIOJ;
            *clk = RCU_GPIOJ;
            break;
#endif /* GPIOJ */
#ifdef GPIOK
        case 'K':
            *port = GPIOK;
            *clk = RCU_GPIOK;
            break;
#endif /* GPIOK */
        default:
            return -1;
    }

    int pin_num = atoi(pin_name + 2);
    if (pin_num < 0 || pin_num > 15)
    {
        return -1;
    }
    *pin = GPIO_PIN_0 << pin_num;

    return 0;
}

int get_uart_alternate_config(const char *alternate, uint32_t *af)
{
    if (alternate == NULL || af == NULL)
    {
        return -1;
    }

    if (strlen(alternate) != 3 || alternate[0] != 'A' || alternate[1] != 'F')
    {
        return -1;
    }

    int af_num = atoi(alternate + 2);
    if (af_num < 0 || af_num > 15)
    {
        return -1;
    }

    switch (af_num)
    {
        case 0: *af = GPIO_AF_0; break;
        case 1: *af = GPIO_AF_1; break;
        case 2: *af = GPIO_AF_2; break;
        case 3: *af = GPIO_AF_3; break;
        case 4: *af = GPIO_AF_4; break;
        case 5: *af = GPIO_AF_5; break;
        case 6: *af = GPIO_AF_6; break;
        case 7: *af = GPIO_AF_7; break;
        case 8: *af = GPIO_AF_8; break;
        case 9: *af = GPIO_AF_9; break;
        case 10: *af = GPIO_AF_10; break;
        case 11: *af = GPIO_AF_11; break;
        case 12: *af = GPIO_AF_12; break;
        case 13: *af = GPIO_AF_13; break;
        case 14: *af = GPIO_AF_14; break;
        case 15: *af = GPIO_AF_15; break;
        default: return -1;
    }

    return 0;
}

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void gd32_uart_gpio_init(struct gd32_uart *uart)
{
    rt_uint32_t tx_port, rx_port;
    rt_uint32_t tx_pin, rx_pin;
    rt_uint32_t pin_af;
    rcu_periph_enum tx_periph, rx_periph;

    if (get_uart_pin_config(uart->tx_pin_name, &tx_port, &tx_pin, &tx_periph) != RT_EOK)
    {
        return;
    }

    if (get_uart_pin_config(uart->rx_pin_name, &rx_port, &rx_pin, &rx_periph) != RT_EOK)
    {
        return;
    }

    get_uart_alternate_config(uart->alternate, &pin_af);

    /* enable USART clock */
    rcu_periph_clock_enable(tx_periph);
    rcu_periph_clock_enable(rx_periph);
    rcu_periph_clock_enable(uart->per_clk);

    /* connect port to USARTx_Tx */
    gpio_af_set(tx_port, pin_af, tx_pin);

    /* connect port to USARTx_Rx */
    gpio_af_set(rx_port, pin_af, rx_pin);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, tx_pin);
    gpio_output_options_set(tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, tx_pin);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, rx_pin);
    gpio_output_options_set(rx_port, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, rx_pin);

    NVIC_SetPriority(uart->irqn, 0);
    NVIC_EnableIRQ(uart->irqn);
}

/**
  * @brief  uart configure
  * @param  serial, cfg
  * @retval None
  */
static rt_err_t gd32_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct gd32_uart *)serial->parent.user_data;

    gd32_uart_gpio_init(uart);

    usart_baudrate_set(uart->uart_periph, cfg->baud_rate);

    switch (cfg->data_bits)
    {
    case DATA_BITS_9:
        usart_word_length_set(uart->uart_periph, USART_WL_9BIT);
        break;

    default:
        usart_word_length_set(uart->uart_periph, USART_WL_8BIT);
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_2:
        usart_stop_bit_set(uart->uart_periph, USART_STB_2BIT);
        break;
    default:
        usart_stop_bit_set(uart->uart_periph, USART_STB_1BIT);
        break;
    }

    switch (cfg->parity)
    {
    case PARITY_ODD:
        usart_parity_config(uart->uart_periph, USART_PM_ODD);
        break;
    case PARITY_EVEN:
        usart_parity_config(uart->uart_periph, USART_PM_EVEN);
        break;
    default:
        usart_parity_config(uart->uart_periph, USART_PM_NONE);
        break;
    }

    usart_receive_config(uart->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(uart->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(uart->uart_periph);

    return RT_EOK;
}

/**
  * @brief  uart control
  * @param  serial, arg
  * @retval None
  */
static rt_err_t gd32_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct gd32_uart *uart;

#ifdef RT_SERIAL_USING_DMA
    rt_ubase_t ctrl_arg = (rt_ubase_t)arg;
#endif

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->irqn);
        /* disable interrupt */
        usart_interrupt_disable(uart->uart_periph, USART_INT_RBNE);

#ifdef RT_SERIAL_USING_DMA
        /* disable DMA */
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            nvic_irq_disable(uart->uart_dma->rx_irq_ch);

            /* disable interrupt */
            usart_interrupt_disable(uart->uart_periph, USART_INT_IDLE);

            dma_channel_disable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);
            usart_dma_receive_config(uart->uart_periph, USART_RECEIVE_DMA_DISABLE);
            dma_deinit(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

            uart->uart_dma->last_recv_index = 0;
        }
#ifdef RT_SERIAL_USING_TX_DMA
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX)
        {
            nvic_irq_disable(uart->uart_tx_dma->rx_irq_ch);

            dma_channel_disable(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch);
            usart_dma_transmit_config(uart->uart_periph, USART_TRANSMIT_DMA_DISABLE);
            dma_deinit(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch);
        }
#endif
#endif
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_EnableIRQ(uart->irqn);
        usart_flag_clear(uart->uart_periph, USART_FLAG_RBNE);
        /* enable interrupt */
        usart_interrupt_enable(uart->uart_periph, USART_INT_RBNE);
        break;

#ifdef RT_SERIAL_USING_DMA
    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            gd32_dma_config(serial, ctrl_arg);
        }
#ifdef RT_SERIAL_USING_TX_DMA
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX)
        {
            gd32_dma_tx_config(serial, ctrl_arg);
        }
#endif
        break;
#endif

    case RT_DEVICE_CTRL_CLOSE:
        usart_disable(uart->uart_periph);
        break;
    }

    return RT_EOK;
}

/**
  * @brief  uart put char
  * @param  serial, ch
  * @retval None
  */
static int gd32_uart_putc(struct rt_serial_device *serial, char ch)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    usart_data_transmit(uart->uart_periph, ch);
    while((usart_flag_get(uart->uart_periph, USART_FLAG_TBE) == RESET));

    return RT_EOK;
}

/**
  * @brief  uart get char
  * @param  serial
  * @retval None
  */
static int gd32_uart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    ch = -1;
    if (usart_flag_get(uart->uart_periph, USART_FLAG_RBNE) != RESET)
        ch = usart_data_receive(uart->uart_periph);
    return ch;
}

#ifdef RT_SERIAL_USING_DMA
static void dma_uart_config(struct rt_serial_device *serial, uint32_t setting_recv_len,
                            void *mem_base_addr)
{
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    dma_single_data_parameter_struct dma_init_struct;

    /* rx dma config */
    uart->uart_dma->setting_recv_len = setting_recv_len;
    dma_deinit(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.request      = uart->uart_dma->dma_mux_req_rx;
    dma_init_struct.direction    = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.memory0_addr = (uint32_t)mem_base_addr;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.number       = uart->uart_dma->setting_recv_len;
    dma_init_struct.periph_addr  = (uint32_t) &(USART_RDATA(uart->uart_periph));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

    dma_flag_clear(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch, uart->uart_dma->rx_flag);
    dma_interrupt_enable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch, DMA_CHXCTL_FTFIE);
}

static void gd32_dma_config(struct rt_serial_device *serial, rt_ubase_t flag)
{
    dma_single_data_parameter_struct dma_init_struct;

    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    /* wait IDLEF set and clear it */
    while(RESET == usart_flag_get(uart->uart_periph, USART_FLAG_IDLE))
    {
        rt_thread_mdelay(10);
    }

    usart_flag_clear(uart->uart_periph, USART_FLAG_IDLE);
    /* enable transmit idle interrupt */
    usart_interrupt_enable(uart->uart_periph, USART_INT_IDLE);
    /* DMA clock enable */
    if(DMA0 == uart->uart_dma->dma_periph)
    {
        rcu_periph_clock_enable(RCU_DMA0);
    } else if(DMA1 == uart->uart_dma->dma_periph)
    {
        rcu_periph_clock_enable(RCU_DMA1);
    } else {
        Error_Handler();
    }

    /* enable DMAMUX clock */
    rcu_periph_clock_enable(RCU_DMAMUX);

#ifdef RT_USING_CACHE
    /* clean d-cache */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, rx_fifo->buffer, serial->config.bufsz);
#endif

    /* rx dma config */
    dma_uart_config(serial, serial->config.bufsz, rx_fifo->buffer);

    usart_dma_receive_config(uart->uart_periph, USART_RECEIVE_DMA_ENABLE);
    dma_channel_enable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);
    /* rx dma interrupt config */
    nvic_irq_enable(uart->uart_dma->rx_irq_ch, 1, 0);
}

#ifdef RT_SERIAL_USING_TX_DMA
static void gd32_dma_tx_config(struct rt_serial_device *serial, rt_ubase_t flag)
{
    dma_single_data_parameter_struct dma_init_struct;

    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    /* DMA clock enable */
    if(DMA0 == uart->uart_tx_dma->dma_periph)
    {
        rcu_periph_clock_enable(RCU_DMA0);
    } else if(DMA1 == uart->uart_tx_dma->dma_periph)
    {
        rcu_periph_clock_enable(RCU_DMA1);
    } else {
        Error_Handler();
    }

    /* enable DMAMUX clock */
    rcu_periph_clock_enable(RCU_DMAMUX);

#ifdef RT_USING_CACHE
    /* clean d-cache */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, rx_fifo->buffer, serial->config.bufsz);
#endif

    /* tx dma config */
    uart->uart_tx_dma->setting_recv_len = 0;
    dma_deinit(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.request      = uart->uart_tx_dma->dma_mux_req_rx;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.periph_addr  = (uint32_t) &(USART_TDATA(uart->uart_periph));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch);
}

#ifdef RT_USING_CACHE
#ifdef BSP_SCB_ENABLE_D_CACHE
static uint8_t dma_buf_cache_pre_bk[32] = {0};
static uint8_t dma_buf_cache_post_bk[32] = {0};
#endif
#endif
static rt_ssize_t gd32_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    struct gd32_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    uart = (struct gd32_uart *) serial->parent.user_data;

    if (size == 0)
    {
        return 0;
    }

    if (RT_SERIAL_DMA_TX == direction)
    {
#ifdef RT_USING_CACHE
#ifdef BSP_SCB_ENABLE_D_CACHE
        uint32_t *dma_cache_pre_ptr = NULL, *dma_cache_post_ptr = NULL;
        uint32_t pre_size = (rt_uint32_t)buf & (rt_uint32_t)0x1F;
        dma_cache_pre_ptr = (uint32_t *)((uint32_t)buf - pre_size);
        rt_memcpy(dma_buf_cache_pre_bk, dma_cache_pre_ptr, pre_size);
        dma_cache_post_ptr = (uint32_t *)((uint32_t)buf + size);
        rt_memcpy(dma_buf_cache_post_bk, dma_cache_post_ptr, sizeof(dma_buf_cache_post_bk));
        /* invalidate d-cache */
        rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, buf, size);

        rt_memcpy(dma_cache_pre_ptr, dma_buf_cache_pre_bk, pre_size);
        rt_memcpy(dma_cache_post_ptr, dma_buf_cache_post_bk, sizeof(dma_buf_cache_post_bk));
#endif
#endif

        dma_memory_address_config(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch, DMA_MEMORY_0, (uint32_t)buf);
        dma_transfer_number_config(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch, size);

        dma_flag_clear(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch, uart->uart_tx_dma->rx_flag);
        dma_interrupt_enable(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch, DMA_CHXCTL_FTFIE);

        usart_flag_clear(uart->uart_periph, USART_FLAG_TBE);
        usart_flag_clear(uart->uart_periph, USART_FLAG_TC);
        /* enable transmit idle interrupt */
        usart_interrupt_enable(uart->uart_periph, USART_INT_TC);

        usart_dma_transmit_config(uart->uart_periph, USART_TRANSMIT_DMA_ENABLE);
        /* rx dma interrupt config */
        nvic_irq_enable(uart->uart_tx_dma->rx_irq_ch, 1, 0);

        dma_channel_enable(uart->uart_tx_dma->dma_periph, uart->uart_tx_dma->dma_ch);

    }
    return 0;
}
#endif

#ifdef RT_USING_CACHE
static struct rt_serial_rx_fifo rx_fifo_cahce_bk = {0};
static uint8_t rx_fifo_buf_cache_bk[32] = {0};
#endif

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial)
{
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;

    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

#ifdef RT_USING_CACHE
    uint32_t *rx_fifo_end_ptr = NULL;
    rx_fifo_end_ptr = (uint32_t *)((uint32_t)rx_fifo->buffer + serial->config.bufsz);
    rt_memcpy(&rx_fifo_cahce_bk, rx_fifo, sizeof(rx_fifo_cahce_bk));
    rt_memcpy(rx_fifo_buf_cache_bk, rx_fifo_end_ptr, sizeof(rx_fifo_buf_cache_bk));
    /* invalidate d-cache */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, rx_fifo->buffer, serial->config.bufsz);

    rt_memcpy(rx_fifo, &rx_fifo_cahce_bk, sizeof(rx_fifo_cahce_bk));
    rt_memcpy(rx_fifo_end_ptr, rx_fifo_buf_cache_bk, sizeof(rx_fifo_buf_cache_bk));
#endif

    recv_total_index = uart->uart_dma->setting_recv_len -
                       dma_transfer_number_get(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

    if (recv_total_index >= uart->uart_dma->last_recv_index)
    {
        recv_len = recv_total_index - uart->uart_dma->last_recv_index;
    } else {
        recv_len = uart->uart_dma->setting_recv_len - uart->uart_dma->last_recv_index + recv_total_index;
    }
    uart->uart_dma->last_recv_index = recv_total_index;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    usart_data_receive(uart->uart_periph);
    usart_flag_clear(uart->uart_periph, USART_FLAG_IDLE);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct rt_serial_device *serial)
{
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;

    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

#ifdef RT_USING_CACHE
    uint32_t *rx_fifo_end_ptr = NULL;
    rx_fifo_end_ptr = (uint32_t *)((uint32_t)rx_fifo->buffer+serial->config.bufsz);
    rt_memcpy(&rx_fifo_cahce_bk, rx_fifo, sizeof(rx_fifo_cahce_bk));
    rt_memcpy(rx_fifo_buf_cache_bk, rx_fifo_end_ptr, sizeof(rx_fifo_buf_cache_bk));
    /* invalidate d-cache */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, rx_fifo->buffer, serial->config.bufsz);

    rt_memcpy(rx_fifo, &rx_fifo_cahce_bk, sizeof(rx_fifo_cahce_bk));
    rt_memcpy(rx_fifo_end_ptr, rx_fifo_buf_cache_bk, sizeof(rx_fifo_buf_cache_bk));
#endif

    if (dma_flag_get(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch, uart->uart_dma->rx_flag) != RESET)
    {
        /* disable dma, stop receive data */
        dma_channel_disable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

        recv_total_index = uart->uart_dma->setting_recv_len -
                           dma_transfer_number_get(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);

        if (recv_total_index >= uart->uart_dma->last_recv_index)
        {
            recv_len = recv_total_index - uart->uart_dma->last_recv_index;
        } else {
            recv_len = uart->uart_dma->setting_recv_len - uart->uart_dma->last_recv_index + recv_total_index;
            uart->uart_dma->last_recv_index = recv_total_index;
        }
        uart->uart_dma->last_recv_index = recv_total_index;

        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

        /* start receive data */
        dma_flag_clear(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch, uart->uart_dma->rx_flag);
        dma_channel_enable(uart->uart_dma->dma_periph, uart->uart_dma->dma_ch);
    }
}

#endif /* RT_SERIAL_USING_DMA */


/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void GD32_UART_IRQHandler(struct rt_serial_device *serial)
{
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    /* UART in mode Receiver -------------------------------------------------*/
    if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_RBNE) != RESET) &&
        (usart_flag_get(uart->uart_periph, USART_FLAG_RBNE) != RESET))
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        usart_flag_clear(uart->uart_periph, USART_FLAG_RBNE);
    }
#ifdef RT_SERIAL_USING_DMA
    if(usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_IDLE) != RESET)
    {
        dma_uart_rx_idle_isr(serial);
    }
#endif
    if (usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_TC) != RESET)
    {
        /* clear interrupt */
        usart_flag_clear(uart->uart_periph, USART_FLAG_TC);
        usart_interrupt_disable(uart->uart_periph, USART_INT_TC);
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);
    }

    if (usart_flag_get(uart->uart_periph, USART_FLAG_ORERR) == SET)
    {
        usart_flag_clear(uart->uart_periph, USART_FLAG_ORERR);
    }
}

static const struct rt_uart_ops gd32_uart_ops =
{
    .configure = gd32_uart_configure,
    .control = gd32_uart_control,
    .putc = gd32_uart_putc,
    .getc = gd32_uart_getc,
#ifndef RT_SERIAL_USING_TX_DMA
    NULL,
#else
    .dma_transmit = gd32_dma_transmit,
#endif
};

/**
  * @brief  uart init
  * @param  None
  * @retval None
  */
int rt_hw_usart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    int i;

    int result;
    rt_uint32_t flag = 0;

    for (i = 0; i < sizeof(uart_obj) / sizeof(uart_obj[0]); i++)
    {
        uart_obj[i].serial->ops    = &gd32_uart_ops;
        uart_obj[i].serial->config = config;

        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX;
#if defined(RT_SERIAL_USING_DMA)
        flag |= RT_DEVICE_FLAG_DMA_RX;
#if defined(RT_SERIAL_USING_TX_DMA)
        flag |= RT_DEVICE_FLAG_DMA_TX;
#endif
#endif
        /* register device */
        result = rt_hw_serial_register(uart_obj[i].serial,
                              uart_obj[i].device_name,
                              flag,
                              (void *)&uart_obj[i]);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

#endif

