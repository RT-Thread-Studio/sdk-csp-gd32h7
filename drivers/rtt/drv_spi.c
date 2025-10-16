/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-20     BruceOu      first implementation
 * 2025-10-10     WangShun     compatible with RT-Studio
 */
#include "drv_spi.h"
#include "spi_config.h"

#ifdef RT_USING_SPI

#if defined(BSP_USING_SPI0) || defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2) || defined(BSP_USING_SPI3) || defined(BSP_USING_SPI4) || defined(BSP_USING_SPI5)

#define DBG_TAG "drv.spi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_SPI0
static struct rt_spi_bus spi_bus0;
#endif
#ifdef BSP_USING_SPI1
static struct rt_spi_bus spi_bus1;
#endif
#ifdef BSP_USING_SPI2
static struct rt_spi_bus spi_bus2;
#endif
#ifdef BSP_USING_SPI3
static struct rt_spi_bus spi_bus3;
#endif
#ifdef BSP_USING_SPI4
static struct rt_spi_bus spi_bus4;
#endif
#ifdef BSP_USING_SPI5
static struct rt_spi_bus spi_bus5;
#endif

#ifdef RT_SPI_USING_DMA
gd32_spi_dma spi_dma[] = {

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI0_TX,
        DMA_REQUEST_SPI0_RX,
        DMA_INTF_FTFIF,

        0
    },

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI1_TX,
        DMA_REQUEST_SPI1_RX,
        DMA_INTF_FTFIF,

        0
    },

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI2_TX,
        DMA_REQUEST_SPI2_RX,
        DMA_INTF_FTFIF,

        0
    },

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI3_TX,
        DMA_REQUEST_SPI3_RX,
        DMA_INTF_FTFIF,

        0
    },

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI4_TX,
        DMA_REQUEST_SPI4_RX,
        DMA_INTF_FTFIF,

        0
    },

    {
        DMA0,
        DMA_CH2,
        DMA_CH3,
        DMA_REQUEST_SPI5_TX,
        DMA_REQUEST_SPI5_RX,
        DMA_INTF_FTFIF,

        0
    },
};
#endif

static const struct gd32_spi spi_bus_obj[] =
{
#ifdef BSP_USING_SPI0
    SPI0_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI1
    SPI1_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI2
    SPI2_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI3
    SPI3_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI4
    SPI4_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI5
    SPI5_BUS_CONFIG,
#endif
};

/* private rt-thread spi ops function */
static rt_err_t spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_ssize_t spixfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops gd32_spi_ops =
{
    .configure = spi_configure,
    .xfer = spixfer,
};

/**
* @brief SPI Initialization
* @param gd32_spi: SPI BUS
* @retval None
*/
static void gd32_spi_init(struct gd32_spi *gd32_spi)
{
    rt_uint32_t sck_port, miso_port, mosi_port;
    rt_uint32_t sck_pin, miso_pin, mosi_pin;
    rcu_periph_enum sck_periph, miso_periph, mosi_periph;
    rt_uint32_t pin_af;

    if (get_pin_config(gd32_spi->sck_pin_name, &sck_port, &sck_pin, &sck_periph) != RT_EOK)
    {
        return;
    }

    if (get_pin_config(gd32_spi->miso_pin_name, &miso_port, &miso_pin, &miso_periph) != RT_EOK)
    {
        return;
    }

    if (get_pin_config(gd32_spi->mosi_pin_name, &mosi_port, &mosi_pin, &mosi_periph) != RT_EOK)
    {
        return;
    }

    pin_alternate_config(gd32_spi->alternate, &pin_af);

    /* enable SPI clock */
    rcu_periph_clock_enable(gd32_spi->spi_clk);
    rcu_periph_clock_enable(sck_periph);
    rcu_periph_clock_enable(miso_periph);
    rcu_periph_clock_enable(mosi_periph);

    /*GPIO pin configuration*/
    gpio_af_set(sck_port, pin_af, sck_pin);
    gpio_af_set(miso_port, pin_af, miso_pin);
    gpio_af_set(mosi_port, pin_af, mosi_pin);
    gpio_mode_set(sck_port, GPIO_MODE_AF, GPIO_PUPD_NONE, sck_pin);
    gpio_mode_set(miso_port, GPIO_MODE_AF, GPIO_PUPD_NONE, miso_pin);
    gpio_mode_set(mosi_port, GPIO_MODE_AF, GPIO_PUPD_NONE, mosi_pin);
    gpio_output_options_set(sck_port, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, sck_pin);
    gpio_output_options_set(miso_port, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, miso_pin);
    gpio_output_options_set(mosi_port, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, mosi_pin);
}

static rt_err_t spi_configure(struct rt_spi_device* device,
                          struct rt_spi_configuration* configuration)
{
    struct rt_spi_bus * spi_bus = (struct rt_spi_bus *)device->bus;
    struct gd32_spi *spi_device = (struct gd32_spi *)spi_bus->parent.user_data;
    spi_parameter_struct spi_init_struct;
    uint32_t spi_periph = spi_device->spi_periph;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    /* Init SPI */
    gd32_spi_init(spi_device);

#if defined SOC_SERIES_GD32H7xx || defined (SOC_SERIES_GD32H75E)
    /* data_width */
    if(configuration->data_width >=4 && configuration->data_width <= 32)
    {
        spi_init_struct.data_size = CFG0_DZ(configuration->data_width - 1);
    }
    else
    {
        return -RT_EIO;
    }
#else
    /* data_width */
    if(configuration->data_width <= 8)
    {
        spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    }
    else if(configuration->data_width <= 16)
    {
        spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
    }
    else
    {
        return -RT_EIO;
    }
#endif

    /* baudrate */
    {
        rcu_clock_freq_enum spi_src;
        uint32_t spi_apb_clock;
        uint32_t max_hz;

        max_hz = configuration->max_hz;

        LOG_D("sys   freq: %d\n", rcu_clock_freq_get(CK_SYS));
        LOG_D("CK_APB2 freq: %d\n", rcu_clock_freq_get(CK_APB2));
        LOG_D("max   freq: %d\n", max_hz);

        #if defined SOC_SERIES_GD32E23x
        spi_src = CK_APB2;
        #else
        if (spi_periph == SPI1 || spi_periph == SPI2)
        {
            spi_src = CK_APB1;
        }
        else
        {
            spi_src = CK_APB2;
        }
        #endif
        spi_apb_clock = rcu_clock_freq_get(spi_src);

        if(max_hz >= spi_apb_clock/2)
        {
            spi_init_struct.prescale = SPI_PSC_2;
        }
        else if (max_hz >= spi_apb_clock/4)
        {
            spi_init_struct.prescale = SPI_PSC_4;
        }
        else if (max_hz >= spi_apb_clock/8)
        {
            spi_init_struct.prescale = SPI_PSC_8;
        }
        else if (max_hz >= spi_apb_clock/16)
        {
            spi_init_struct.prescale = SPI_PSC_16;
        }
        else if (max_hz >= spi_apb_clock/32)
        {
            spi_init_struct.prescale = SPI_PSC_32;
        }
        else if (max_hz >= spi_apb_clock/64)
        {
            spi_init_struct.prescale = SPI_PSC_64;
        }
        else if (max_hz >= spi_apb_clock/128)
        {
            spi_init_struct.prescale = SPI_PSC_128;
        }
        else
        {
            /*  min prescaler 256 */
            spi_init_struct.prescale = SPI_PSC_256;
        }
    } /* baudrate */

    switch(configuration->mode & RT_SPI_MODE_3)
    {
    case RT_SPI_MODE_0:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        break;
    case RT_SPI_MODE_1:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
        break;
    case RT_SPI_MODE_2:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
        break;
    case RT_SPI_MODE_3:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
        break;
    }

    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        spi_init_struct.endian = SPI_ENDIAN_MSB;
    }
    else
    {
        spi_init_struct.endian = SPI_ENDIAN_LSB;
    }

    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.nss = SPI_NSS_SOFT;

    spi_crc_off(spi_periph);

#if defined (SOC_SERIES_GD32H7xx) || defined (SOC_SERIES_GD32H75E)
    /* enable SPI byte access */
    spi_byte_access_enable(spi_periph);
    /* enable SPI NSS output */
    spi_nss_output_enable(spi_periph);
#endif

    /* init SPI */
    spi_init(spi_periph, &spi_init_struct);
    /* Enable SPI_MASTER */
    spi_enable(spi_periph);

    return RT_EOK;
}

#ifdef RT_SPI_USING_DMA
static rt_err_t dma_spi_config(struct rt_spi_device *device, struct rt_spi_message *message)
{
    struct rt_spi_bus *spi_bus = (struct rt_spi_bus *)device->bus;
    struct gd32_spi *spi_device = (struct gd32_spi *)spi_bus->parent.user_data;
    rt_uint8_t *recv_ptr_dma = RT_NULL;
    rt_uint8_t *send_ptr_dma = RT_NULL;

    dma_single_data_parameter_struct dma_init_struct;
    /* deinitialize DMA registers of a channel */
    dma_deinit(spi_device->spi_dma->dma_periph, spi_device->spi_dma->txdma_ch);
    dma_deinit(spi_device->spi_dma->dma_periph, spi_device->spi_dma->rxdma_ch);
    dma_single_data_para_struct_init(&dma_init_struct);

    /* SPI transmit DMA config */
    dma_init_struct.request = spi_device->spi_dma->dma_mux_req_tx;
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
    if(message->send_buf  == RT_NULL)
    {
        dma_init_struct.memory0_addr = (uint32_t)send_ptr_dma;
    } else {
        dma_init_struct.memory0_addr = (uint32_t)message->send_buf;
    }
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.number = message->length;
    dma_init_struct.periph_addr = (uint32_t)&SPI_TDATA(spi_device->spi_periph);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
    dma_single_data_mode_init(spi_device->spi_dma->dma_periph, spi_device->spi_dma->txdma_ch, &dma_init_struct);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->txdma_ch, DMA_FLAG_FTF);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->txdma_ch, DMA_FLAG_HTF);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->txdma_ch, DMA_FLAG_FEE);

    /* SPI receive DMA config */
    dma_init_struct.request = spi_device->spi_dma->dma_mux_req_rx;
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    if(message->recv_buf  == RT_NULL)
    {
        dma_init_struct.memory0_addr = (uint32_t)recv_ptr_dma;
    } else {
        dma_init_struct.memory0_addr = (uint32_t)message->recv_buf;
    }
    dma_init_struct.periph_addr = (uint32_t)&SPI_RDATA(spi_device->spi_periph);
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(spi_device->spi_dma->dma_periph, spi_device->spi_dma->rxdma_ch, &dma_init_struct);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->rxdma_ch, DMA_FLAG_FTF);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->rxdma_ch, DMA_FLAG_HTF);
    dma_flag_clear(spi_device->spi_dma->dma_periph, spi_device->spi_dma->rxdma_ch, DMA_FLAG_FEE);

    return RT_EOK;
}
#endif

static rt_ssize_t spixfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct rt_spi_bus * gd32_spi_bus = (struct rt_spi_bus *)device->bus;
    struct gd32_spi *spi_device = (struct gd32_spi *)gd32_spi_bus->parent.user_data;
    struct rt_spi_configuration * config = &device->config;
    uint32_t spi_periph = spi_device->spi_periph;

    RT_ASSERT(device != NULL);
    RT_ASSERT(message != NULL);

 #ifdef RT_SPI_USING_DMA
    dma_spi_config(device, message);
#endif

    /* take CS */
    if (message->cs_take && !(device->config.mode & RT_SPI_NO_CS) && (device->cs_pin != PIN_NONE))
    {
        if (device->config.mode & RT_SPI_CS_HIGH)
        {
            rt_pin_write(device->cs_pin, PIN_HIGH);
        }
        else
        {
            rt_pin_write(device->cs_pin, PIN_LOW);
        }
    }

    LOG_D("%s transfer prepare and start", spi_drv->config->bus_name);
    LOG_D("%s sendbuf: %X, recvbuf: %X, length: %d",
          spi_drv->config->bus_name,
          (uint32_t)message->send_buf,
          (uint32_t)message->recv_buf, message->length);

    {
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;
            rt_uint32_t size = message->length;

            LOG_D("spi poll transfer start: %d\n", size);

            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                /* Todo: replace register read/write by gd32f4 lib */
                /* Wait until the transmit buffer is empty */
                #if defined (SOC_SERIES_GD32H7xx) || defined (SOC_SERIES_GD32H75E)
                spi_master_transfer_start(spi_periph, SPI_TRANS_START);
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TP));
                #else
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));
                #endif
                /* Send the byte */
                spi_i2s_data_transmit(spi_periph, data);

                /* Wait until a data is received */
                #if defined (SOC_SERIES_GD32H7xx) || defined (SOC_SERIES_GD32H75E)
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RP));
                #else
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));
                #endif
                /* Get the received data */
                data = spi_i2s_data_receive(spi_periph);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
            LOG_D("spi poll transfer finsh\n");
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;
            rt_uint32_t size = message->length;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                /* Wait until the transmit buffer is empty */
                #if defined (SOC_SERIES_GD32H7xx) || defined (SOC_SERIES_GD32H75E)
                spi_master_transfer_start(spi_periph, SPI_TRANS_START);
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TP));
                #else
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));
                #endif
                /* Send the byte */
                spi_i2s_data_transmit(spi_periph, data);

                /* Wait until a data is received */
                #if defined (SOC_SERIES_GD32H7xx) || defined (SOC_SERIES_GD32H75E)
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RP));
                #else
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));
                #endif
                /* Get the received data */
                data = spi_i2s_data_receive(spi_periph);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        #if defined SOC_SERIES_GD32H7xx || defined (SOC_SERIES_GD32H75E)
        else if(config->data_width <= 32)
        {
            const rt_uint32_t * send_ptr = message->send_buf;
            rt_uint32_t * recv_ptr = message->recv_buf;
            rt_uint32_t size = message->length;
            /* SPI master start transfer */
            spi_master_transfer_start(spi_periph, SPI_TRANS_START);
            while(size--)
            {
                rt_uint32_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                /* Wait until the transmit buffer is empty */
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TP));
                /* Send the byte */
                spi_i2s_data_transmit(spi_periph, data);

                /* Wait until a data is received */
                while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RP));
                /* Get the received data */
                data = spi_i2s_data_receive(spi_periph);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        #endif
        else
        {
            return -RT_EIO;
        }
    }

    /* release CS */
    if (message->cs_release && !(device->config.mode & RT_SPI_NO_CS) && (device->cs_pin != PIN_NONE))
    {
        if (device->config.mode & RT_SPI_CS_HIGH)
            rt_pin_write(device->cs_pin, PIN_LOW);
        else
            rt_pin_write(device->cs_pin, PIN_HIGH);
    }

    return message->length;
};

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);

    if(cs_pin != PIN_NONE)
    {
        /* initialize the cs pin && select the slave*/
        rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);
        rt_pin_write(cs_pin, PIN_HIGH);
    }

    result = rt_spi_bus_attach_device_cspin(spi_device, device_name, bus_name, cs_pin, RT_NULL);

    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    RT_ASSERT(result == RT_EOK);

    LOG_D("%s attach to %s done", device_name, bus_name);

    return result;
}

int rt_hw_spi_init(void)
{
    int result = 0;
    int i;

    for (i = 0; i < sizeof(spi_bus_obj) / sizeof(spi_bus_obj[0]); i++)
    {
        spi_bus_obj[i].spi_bus->parent.user_data = (void *)&spi_bus_obj[i];

        result = rt_spi_bus_register(spi_bus_obj[i].spi_bus, spi_bus_obj[i].bus_name, &gd32_spi_ops);

        RT_ASSERT(result == RT_EOK);

        LOG_D("%s bus init done", spi_bus_obj[i].bus_name);
    }

    return result;
}

INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif /* BSP_USING_SPI0 || BSP_USING_SPI1 || BSP_USING_SPI2 || BSP_USING_SPI3 || BSP_USING_SPI4 || BSP_USING_SPI5 */
#endif /* RT_USING_SPI */

