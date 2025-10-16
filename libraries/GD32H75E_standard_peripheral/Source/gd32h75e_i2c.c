/*!
    \file    gd32h75e_i2c.c
    \brief   I2C driver

    \version 2025-08-07, V1.2.0, firmware for GD32H75E
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "gd32h75e_i2c.h"

/* I2C register bit mask */
#define I2C_ADDRESS_MASK              ((uint32_t)0x000003FFU)               /*!< i2c address mask */
#define I2C_ADDRESS2_MASK             ((uint32_t)0x000000FEU)               /*!< the second i2c address mask */

/* I2C register bit offset */
#define CTL0_DNF_OFFSET               ((uint32_t)0x00000008U)               /*!< bit offset of DNF in I2C_CTL0 */
#define CTL1_BYTENUM_OFFSET           ((uint32_t)0x00000010U)               /*!< bit offset of BYTENUM in I2C_CTL1 */
#define STAT_READDR_OFFSET            ((uint32_t)0x00000011U)               /*!< bit offset of READDR in I2C_STAT */
#define TIMING_SCLL_OFFSET            ((uint32_t)0x00000000U)               /*!< bit offset of SCLL in I2C_TIMING */
#define TIMING_SCLH_OFFSET            ((uint32_t)0x00000008U)               /*!< bit offset of SCLH in I2C_TIMING */
#define TIMING_SDADELY_OFFSET         ((uint32_t)0x00000010U)               /*!< bit offset of SDADELY in I2C_TIMING */
#define TIMING_SCLDELY_OFFSET         ((uint32_t)0x00000014U)               /*!< bit offset of SCLDELY in I2C_TIMING */
#define TIMING_PSC_OFFSET             ((uint32_t)0x0000001CU)               /*!< bit offset of PSC in I2C_TIMING */
#define SADDR1_ADDMSK_OFFSET          ((uint32_t)0x00000008U)               /*!< bit offset of ADDMSK in I2C_SADDR1 */
#define TIMEOUT_BUSTOB_OFFSET         ((uint32_t)0x00000010U)               /*!< bit offset of BUSTOB in I2C_TIMEOUT */

/*!
    \brief      reset I2C (API_ID:0x0001U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_deinit(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0001U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        switch(i2c_periph) {
        /* reset I2C0 */
        case I2C0:
            rcu_periph_reset_enable(RCU_I2C0RST);
            rcu_periph_reset_disable(RCU_I2C0RST);
            break;
        /* reset I2C1 */
        case I2C1:
            rcu_periph_reset_enable(RCU_I2C1RST);
            rcu_periph_reset_disable(RCU_I2C1RST);
            break;
        /* reset I2C2 */
        case I2C2:
            rcu_periph_reset_enable(RCU_I2C2RST);
            rcu_periph_reset_disable(RCU_I2C2RST);
            break;
        /* reset I2C3 */
        case I2C3:
            rcu_periph_reset_enable(RCU_I2C3RST);
            rcu_periph_reset_disable(RCU_I2C3RST);
            break;
        default:
            break;
        }
    }
}

/*!
    \brief      configure the timing parameters (API_ID:0x0002U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  psc: 0-0x0000000F, timing prescaler
    \param[in]  scl_dely: 0-0x0000000F, data setup time
    \param[in]  sda_dely: 0-0x0000000F, data hold time
    \param[out] none
    \retval     none
*/
void i2c_timing_config(uint32_t i2c_periph, uint32_t psc, uint32_t scl_dely, uint32_t sda_dely)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0002U), ERR_PERIPH);
    } else if(NOT_I2C_PSC(psc)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0002U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_SCL_DELY(scl_dely)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0002U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_SDA_DELY(sda_dely)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0002U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* clear PSC, SCLDELY, SDADELY bits in I2C_TIMING register */
        I2C_TIMING(i2c_periph) &= ~I2C_TIMING_PSC;
        I2C_TIMING(i2c_periph) &= ~I2C_TIMING_SCLDELY;
        I2C_TIMING(i2c_periph) &= ~I2C_TIMING_SDADELY;
        /* mask PSC, SCLDELY, SDADELY bits in I2C_TIMING register */
        psc = (uint32_t)(psc << TIMING_PSC_OFFSET) & I2C_TIMING_PSC;
        scl_dely = (uint32_t)(scl_dely << TIMING_SCLDELY_OFFSET) & I2C_TIMING_SCLDELY;
        sda_dely = (uint32_t)(sda_dely << TIMING_SDADELY_OFFSET) & I2C_TIMING_SDADELY;
        /* write PSC, SCLDELY, SDADELY bits in I2C_TIMING register */
        I2C_TIMING(i2c_periph) |= (psc | scl_dely | sda_dely);
    }
}

/*!
    \brief      configure digital noise filter (API_ID:0x0003U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  filter_length: the length of filter spikes
                only one parameter can be selected which is shown as below:
      \arg        FILTER_DISABLE: digital filter is disabled
      \arg        FILTER_LENGTH_1: digital filter is enabled and filter spikes with a length of up to 1 tI2CCLK
      \arg        FILTER_LENGTH_2: digital filter is enabled and filter spikes with a length of up to 2 tI2CCLK
      \arg        FILTER_LENGTH_3: digital filter is enabled and filter spikes with a length of up to 3 tI2CCLK
      \arg        FILTER_LENGTH_4: digital filter is enabled and filter spikes with a length of up to 4 tI2CCLK
      \arg        FILTER_LENGTH_5: digital filter is enabled and filter spikes with a length of up to 5 tI2CCLK
      \arg        FILTER_LENGTH_6: digital filter is enabled and filter spikes with a length of up to 6 tI2CCLK
      \arg        FILTER_LENGTH_7: digital filter is enabled and filter spikes with a length of up to 7 tI2CCLK
      \arg        FILTER_LENGTH_8: digital filter is enabled and filter spikes with a length of up to 8 tI2CCLK
      \arg        FILTER_LENGTH_9: digital filter is enabled and filter spikes with a length of up to 9 tI2CCLK
      \arg        FILTER_LENGTH_10: digital filter is enabled and filter spikes with a length of up to 10 tI2CCLK
      \arg        FILTER_LENGTH_11: digital filter is enabled and filter spikes with a length of up to 11 tI2CCLK
      \arg        FILTER_LENGTH_12: digital filter is enabled and filter spikes with a length of up to 12 tI2CCLK
      \arg        FILTER_LENGTH_13: digital filter is enabled and filter spikes with a length of up to 13 tI2CCLK
      \arg        FILTER_LENGTH_14: digital filter is enabled and filter spikes with a length of up to 14 tI2CCLK
      \arg        FILTER_LENGTH_15: digital filter is enabled and filter spikes with a length of up to 15 tI2CCLK
    \param[out] none
    \retval     none
*/
void i2c_digital_noise_filter_config(uint32_t i2c_periph, uint32_t filter_length)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0003U), ERR_PERIPH);
    } else if(NOT_I2C_FILTER_LENGTH(filter_length)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0003U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= (uint32_t)(~I2C_CTL0_DNF);
        I2C_CTL0(i2c_periph) |= (uint32_t)(filter_length << CTL0_DNF_OFFSET);
    }
}

/*!
    \brief      enable analog noise filter (API_ID:0x0004U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_analog_noise_filter_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0004U), ERR_PERIPH);
    } else
#endif
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_ANOFF;
    }
}

/*!
    \brief      disable analog noise filter (API_ID:0x0005U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_analog_noise_filter_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0005U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_ANOFF;
    }
}

/*!
    \brief      configure the SCL high and low period of clock in master mode (API_ID:0x0006U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  sclh: 0-0x000000FF, SCL high period
    \param[in]  scll: 0-0x000000FF, SCL low period
    \param[out] none
    \retval     none
*/
void i2c_master_clock_config(uint32_t i2c_periph, uint32_t sclh, uint32_t scll)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0006U), ERR_PERIPH);
    } else if(NOT_I2C_SCLH(sclh)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0006U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_SCLL(scll)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0006U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* clear SCLH, SCLL bits in I2C_TIMING register */
        I2C_TIMING(i2c_periph) &= ~I2C_TIMING_SCLH;
        I2C_TIMING(i2c_periph) &= ~I2C_TIMING_SCLL;
        /* mask SCLH, SCLL bits in I2C_TIMING register */
        sclh = (uint32_t)(sclh << TIMING_SCLH_OFFSET) & I2C_TIMING_SCLH;
        scll = (uint32_t)(scll << TIMING_SCLL_OFFSET) & I2C_TIMING_SCLL;
        /* write SCLH, SCLL bits in I2C_TIMING register */
        I2C_TIMING(i2c_periph) |= (sclh | scll);
    }
}

/*!
    \brief      configure I2C slave address and transfer direction in master mode (API_ID:0x0007U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  address: 0-0x3FF except reserved address, I2C slave address to be sent
    \param[in]  trans_direction: I2C transfer direction in master mode
                only one parameter can be selected which is shown as below:
      \arg        I2C_MASTER_TRANSMIT: master transmit
      \arg        I2C_MASTER_RECEIVE: master receive
    \param[out] none
    \retval     none
*/
void i2c_master_addressing(uint32_t i2c_periph, uint32_t address, uint32_t trans_direction)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0007U), ERR_PERIPH);
    } else if(NOT_I2C_ADDRESS(address)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0007U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_TRANS_DIRECTION(trans_direction)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0007U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure slave address */
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_SADDRESS;
        I2C_CTL1(i2c_periph) |= address;
        /* configure transfer direction */
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_TRDIR;
        I2C_CTL1(i2c_periph) |= trans_direction;
    }
}

/*!
    \brief      10-bit address header executes read direction only in master receive mode (API_ID:0x0008U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_address10_header_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0008U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_HEAD10R;
    }
}

/*!
    \brief      10-bit address header executes complete sequence in master receive mode (API_ID:0x0009U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_address10_header_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0009U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_HEAD10R;
    }
}

/*!
    \brief      enable 10-bit addressing mode in master mode (API_ID:0x000AU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_address10_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000AU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_ADD10EN;
    }
}

/*!
    \brief      disable 10-bit addressing mode in master mode (API_ID:0x000BU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_address10_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000BU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_ADD10EN;
    }
}

/*!
    \brief      enable I2C automatic end mode in master mode (API_ID:0x000CU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_automatic_end_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000CU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_AUTOEND;
    }
}

/*!
    \brief      disable I2C automatic end mode in master mode (API_ID:0x000DU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_automatic_end_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000DU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_AUTOEND;
    }
}

/*!
    \brief      enable the response to a general call (API_ID:0x000EU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_slave_response_to_gcall_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000EU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_GCEN;
    }
}

/*!
    \brief      disable the response to a general call (API_ID:0x000FU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_slave_response_to_gcall_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x000FU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_GCEN;
    }
}

/*!
    \brief      enable to stretch SCL low when data is not ready in slave mode (API_ID:0x0010U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_stretch_scl_low_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0010U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_SS;
    }
}

/*!
    \brief      disable to stretch SCL low when data is not ready in slave mode (API_ID:0x0011U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_stretch_scl_low_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0011U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_SS;
    }
}

/*!
    \brief      configure I2C slave address (API_ID:0x0012U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  address: I2C address
    \param[in]  addr_format: 7 bits or 10 bits
                only one parameter can be selected which is shown as below:
      \arg        I2C_ADDFORMAT_7BITS: address format is 7 bits
      \arg        I2C_ADDFORMAT_10BITS: address format is 10 bits
    \param[out] none
    \retval     none
*/
void i2c_address_config(uint32_t i2c_periph, uint32_t address, uint32_t addr_format)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0012U), ERR_PERIPH);
    } else if(NOT_I2C_ADDRESS(address)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0012U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_ADDR_FORMAT(addr_format)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0012U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure ADDRESS[7:1] and address format */
        address = address & I2C_ADDRESS_MASK;
        I2C_SADDR0(i2c_periph) = (addr_format | address);
        /* enable i2c address in slave mode */
        I2C_SADDR0(i2c_periph) |= I2C_SADDR0_ADDRESSEN;
    }
}

/*!
    \brief      define which bits of ADDRESS[7:1] need to compare with the incoming address byte (API_ID:0x0013U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  compare_bits: the bits need to compare
                only one parameter can be selected which is shown as below:
      \arg        ADDRESS_BIT1_COMPARE: address bit1 needs compare
      \arg        ADDRESS_BIT2_COMPARE: address bit2 needs compare
      \arg        ADDRESS_BIT3_COMPARE: address bit3 needs compare
      \arg        ADDRESS_BIT4_COMPARE: address bit4 needs compare
      \arg        ADDRESS_BIT5_COMPARE: address bit5 needs compare
      \arg        ADDRESS_BIT6_COMPARE: address bit6 needs compare
      \arg        ADDRESS_BIT7_COMPARE: address bit7 needs compare
    \param[out] none
    \retval     none
*/
void i2c_address_bit_compare_config(uint32_t i2c_periph, uint32_t compare_bits)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0013U), ERR_PERIPH);
    } else if(NOT_I2C_COMPARE_BITS(compare_bits)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0013U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL2(i2c_periph) &= ~I2C_CTL2_ADDM;
        I2C_CTL2(i2c_periph) |= compare_bits;
    }
}

/*!
    \brief      disable I2C address in slave mode (API_ID:0x0014U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_address_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0014U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_SADDR0(i2c_periph) &= ~I2C_SADDR0_ADDRESSEN;
    }
}

/*!
    \brief      configure I2C second slave address (API_ID:0x0015U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  address: I2C address
    \param[in]  addr_mask: the bits not need to compare
                only one parameter can be selected which is shown as below:
      \arg        ADDRESS2_NO_MASK: no mask, all the bits must be compared
      \arg        ADDRESS2_MASK_BIT1: ADDRESS2[1] is masked, only ADDRESS2[7:2] are compared
      \arg        ADDRESS2_MASK_BIT1_2: ADDRESS2[2:1] is masked, only ADDRESS2[7:3] are compared
      \arg        ADDRESS2_MASK_BIT1_3: ADDRESS2[3:1] is masked, only ADDRESS2[7:4] are compared
      \arg        ADDRESS2_MASK_BIT1_4: ADDRESS2[4:1] is masked, only ADDRESS2[7:5] are compared
      \arg        ADDRESS2_MASK_BIT1_5: ADDRESS2[5:1] is masked, only ADDRESS2[7:6] are compared
      \arg        ADDRESS2_MASK_BIT1_6: ADDRESS2[6:1] is masked, only ADDRESS2[7] are compared
      \arg        ADDRESS2_MASK_ALL: all the ADDRESS2[7:1] bits are masked
    \param[out] none
    \retval     none
*/
void i2c_second_address_config(uint32_t i2c_periph, uint32_t address, uint32_t addr_mask)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0015U), ERR_PERIPH);
    } else if(NOT_I2C_SECOND_ADDRESS(address)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0015U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_I2C_ADDR_MASK(addr_mask)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0015U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure ADDRESS2[7:1] */
        address = address & I2C_ADDRESS2_MASK;
        I2C_SADDR1(i2c_periph) |= address;
        /* configure ADDRESS2[7:1] mask */
        I2C_SADDR1(i2c_periph) &= ~I2C_SADDR1_ADDMSK2;
        I2C_SADDR1(i2c_periph) |= (uint32_t)(addr_mask << SADDR1_ADDMSK_OFFSET);
        /* enable i2c second address in slave mode */
        I2C_SADDR1(i2c_periph) |= I2C_SADDR1_ADDRESS2EN;
    }
}

/*!
    \brief      disable I2C second address in slave mode (API_ID:0x0016U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_second_address_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0016U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_SADDR1(i2c_periph) &= ~I2C_SADDR1_ADDRESS2EN;
    }
}

/*!
    \brief      get received match address in slave mode (API_ID:0x0017U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     uint32_t: received match address
*/
uint32_t i2c_recevied_address_get(uint32_t i2c_periph)
{
    uint32_t reval = 0U;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0017U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reval = (uint32_t)((I2C_STAT(i2c_periph) & I2C_STAT_READDR) >> STAT_READDR_OFFSET);
    }
    return reval;
}

/*!
    \brief      enable slave byte control (API_ID:0x0018U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_slave_byte_control_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0018U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_SBCTL;
    }
}

/*!
    \brief      disable slave byte control (API_ID:0x0019U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_slave_byte_control_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0019U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_SBCTL;
    }
}

/*!
    \brief      generate a NACK in slave mode (API_ID:0x001AU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_nack_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x001AU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_NACKEN;
    }
}

/*!
    \brief      enable wakeup from deep-sleep mode (API_ID:0x001CU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_wakeup_from_deepsleep_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x001CU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
    I2C_CTL0(i2c_periph) |= I2C_CTL0_WUEN;
    }
}

/*!
    \brief      disable wakeup from deep-sleep mode (API_ID:0x001DU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_wakeup_from_deepsleep_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x001DU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
    I2C_CTL0(i2c_periph) &= ~I2C_CTL0_WUEN;
    }
}

/*!
    \brief      enable I2C (API_ID:0x001EU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x001EU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_I2CEN;
    }
}

/*!
    \brief      disable I2C (API_ID:0x001FU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x001FU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_I2CEN;
    }
}

/*!
    \brief      generate a START condition on I2C bus (API_ID:0x0020U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_start_on_bus(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0020U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_START;
    }
}

/*!
    \brief      generate a STOP condition on I2C bus (API_ID:0x0021U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_stop_on_bus(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0021U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_STOP;
    }
}

/*!
    \brief      I2C transmit data (API_ID:0x0022U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  data: data to be transmitted
    \param[out] none
    \retval     none
*/
void i2c_data_transmit(uint32_t i2c_periph, uint32_t data)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0022U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TDATA(i2c_periph) = (I2C_TDATA_TDATA & data);
    }
}

/*!
    \brief      I2C receive data (API_ID:0x0023U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     uint32_t: 0x0-0xFF
*/
uint32_t i2c_data_receive(uint32_t i2c_periph)
{
    uint32_t reval = 0U;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0023U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reval = (I2C_RDATA(i2c_periph) & I2C_RDATA_RDATA);
    }
    return reval;
}

/*!
    \brief      enable I2C reload mode (API_ID:0x0024U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_reload_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0024U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_RELOAD;
    }
}

/*!
    \brief      disable I2C reload mode (API_ID:0x0025U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_reload_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0025U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) &= ~I2C_CTL1_RELOAD;
    }
}

/*!
    \brief      configure number of bytes to be transferred (API_ID:0x0026U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  byte_number: 0x0-0xFF, number of bytes to be transferred
    \param[out] none
    \retval     none
*/
void i2c_transfer_byte_number_config(uint32_t i2c_periph, uint8_t byte_number)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0026U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) &= (uint32_t)(~I2C_CTL1_BYTENUM);
        I2C_CTL1(i2c_periph) |= (uint32_t)(((uint32_t)byte_number) << CTL1_BYTENUM_OFFSET);
    }
}

/*!
    \brief      enable I2C DMA for transmission or reception (API_ID:0x0027U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  dma: I2C DMA
                only one parameter can be selected which is shown as below:
      \arg        I2C_DMA_TRANSMIT: transmit data using DMA
      \arg        I2C_DMA_RECEIVE: receive data using DMA
    \param[out] none
    \retval     none
*/
void i2c_dma_enable(uint32_t i2c_periph, uint8_t dma)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0027U), ERR_PERIPH);
    } else if(NOT_I2C_DMA(dma)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0027U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(I2C_DMA_TRANSMIT == dma) {
            I2C_CTL0(i2c_periph) |= I2C_CTL0_DENT;
        } else {
            I2C_CTL0(i2c_periph) |= I2C_CTL0_DENR;
        }
    }
}

/*!
    \brief      disable I2C DMA for transmission or reception (API_ID:0x0028U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  dma: I2C DMA
                only one parameter can be selected which is shown as below:
      \arg        I2C_DMA_TRANSMIT: transmit data using DMA
      \arg        I2C_DMA_RECEIVE: receive data using DMA
    \param[out] none
    \retval     none
*/
void i2c_dma_disable(uint32_t i2c_periph, uint8_t dma)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0028U), ERR_PERIPH);
    } else if(NOT_I2C_DMA(dma)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0028U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(I2C_DMA_TRANSMIT == dma) {
            I2C_CTL0(i2c_periph) &= ~I2C_CTL0_DENT;
        } else {
            I2C_CTL0(i2c_periph) &= ~I2C_CTL0_DENR;
        }
    }
}

/*!
    \brief      I2C transfers PEC value (API_ID:0x0029U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_pec_transfer(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0029U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL1(i2c_periph) |= I2C_CTL1_PECTRANS;
    }
}

/*!
    \brief      enable I2C PEC calculation (API_ID:0x002AU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_pec_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002AU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_PECEN;
    }
}

/*!
    \brief      disable I2C PEC calculation (API_ID:0x002BU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_pec_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002BU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_PECEN;
    }
}

/*!
    \brief      get packet error checking value (API_ID:0x002CU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     uint32_t: 0x0-0xFF
*/
uint32_t i2c_pec_value_get(uint32_t i2c_periph)
{
    uint32_t reval = 0U;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002CU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reval = (I2C_PEC(i2c_periph) & I2C_PEC_PECV);
    }
    return reval;
}

/*!
    \brief      enable SMBus alert (API_ID:0x002DU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_alert_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002DU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_SMBALTEN;
    }
}

/*!
    \brief      disable SMBus alert (API_ID:0x002EU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_alert_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002EU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_SMBALTEN;
    }
}

/*!
    \brief      enable SMBus device default address (API_ID:0x002FU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_default_addr_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x002FU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_SMBDAEN;
    }
}

/*!
    \brief      disable SMBus device default address (API_ID:0x0030U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_default_addr_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0030U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_SMBDAEN;
    }
}

/*!
    \brief      enable SMBus host address (API_ID:0x0031U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_host_addr_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0031U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= I2C_CTL0_SMBHAEN;
    }
}

/*!
    \brief      disable SMBus host address (API_ID:0x0032U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_smbus_host_addr_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0032U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~I2C_CTL0_SMBHAEN;
    }
}

/*!
    \brief      enable extended clock timeout detection (API_ID:0x0033U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_extented_clock_timeout_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0033U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) |= I2C_TIMEOUT_EXTOEN;
    }
}

/*!
    \brief      disable extended clock timeout detection (API_ID:0x0034U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_extented_clock_timeout_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0034U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) &= ~I2C_TIMEOUT_EXTOEN;
    }
}

/*!
    \brief      enable clock timeout detection (API_ID:0x0035U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_clock_timeout_enable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0035U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) |= I2C_TIMEOUT_TOEN;
    }
}

/*!
    \brief      disable clock timeout detection (API_ID:0x0036U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void i2c_clock_timeout_disable(uint32_t i2c_periph)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0036U), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) &= ~I2C_TIMEOUT_TOEN;
    }
}

/*!
    \brief      configure bus timeout B (API_ID:0x0037U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  timeout: bus timeout B
    \param[out] none
    \retval     none
*/
void i2c_bus_timeout_b_config(uint32_t i2c_periph, uint32_t timeout)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0037U), ERR_PERIPH);
    } else if(NOT_I2C_TIMEOUT(timeout)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0037U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) &= ~I2C_TIMEOUT_BUSTOB;
        I2C_TIMEOUT(i2c_periph) |= (uint32_t)(timeout << TIMEOUT_BUSTOB_OFFSET);
    }
}

/*!
    \brief      configure bus timeout A (API_ID:0x0038U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  timeout: bus timeout A
    \param[out] none
    \retval     none
*/
void i2c_bus_timeout_a_config(uint32_t i2c_periph, uint32_t timeout)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0038U), ERR_PERIPH);
    } else if(NOT_I2C_TIMEOUT(timeout)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0038U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) &= ~I2C_TIMEOUT_BUSTOA;
        I2C_TIMEOUT(i2c_periph) |= timeout;
    }
}

/*!
    \brief      configure idle clock timeout detection (API_ID:0x0039U)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  timeout: bus timeout A
      \arg        BUSTOA_DETECT_SCL_LOW: BUSTOA is used to detect SCL low timeout
      \arg        BUSTOA_DETECT_IDLE: BUSTOA is used to detect both SCL and SDA high timeout when the bus is idle
    \param[out] none
    \retval     none
*/
void i2c_idle_clock_timeout_config(uint32_t i2c_periph, uint32_t timeout)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0039U), ERR_PERIPH);
    } else if(NOT_I2C_TIMEOUT_DET(timeout)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x0039U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_TIMEOUT(i2c_periph) &= ~I2C_TIMEOUT_TOIDLE;
        I2C_TIMEOUT(i2c_periph) |= timeout;
    }
}

/*!
    \brief      get I2C flag status (API_ID:0x003AU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  flag: I2C flags
                only one parameter can be selected which is shown as below:
      \arg        I2C_FLAG_TBE: I2C_TDATA is empty during transmitting
      \arg        I2C_FLAG_TI: transmit interrupt
      \arg        I2C_FLAG_RBNE: I2C_RDATA is not empty during receiving
      \arg        I2C_FLAG_ADDSEND: address received matches in slave mode
      \arg        I2C_FLAG_NACK: not acknowledge flag
      \arg        I2C_FLAG_STPDET: STOP condition detected in slave mode
      \arg        I2C_FLAG_TC: transfer complete in master mode
      \arg        I2C_FLAG_TCR: transfer complete reload
      \arg        I2C_FLAG_BERR: bus error
      \arg        I2C_FLAG_LOSTARB: arbitration Lost
      \arg        I2C_FLAG_OUERR: overrun/underrun error in slave mode
      \arg        I2C_FLAG_PECERR: PEC error
      \arg        I2C_FLAG_TIMEOUT: timeout flag
      \arg        I2C_FLAG_SMBALT: SMBus alert
      \arg        I2C_FLAG_I2CBSY: busy flag
      \arg        I2C_FLAG_TR: whether the I2C is a transmitter or a receiver in slave mode
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus i2c_flag_get(uint32_t i2c_periph, uint32_t flag)
{
    FlagStatus reval = RESET;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003AU), ERR_PERIPH);
    } else if(NOT_I2C_STAT_FALG(flag)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003AU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(RESET != (I2C_STAT(i2c_periph) & flag)) {
            reval = SET;
        } else {
            reval = RESET;
        }
    }
    return reval;
}

/*!
    \brief      clear I2C flag status (API_ID:0x003BU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  flag: I2C flags
                only one parameter can be selected which is shown as below:
      \arg        I2C_FLAG_ADDSEND: address received matches in slave mode
      \arg        I2C_FLAG_NACK: not acknowledge flag
      \arg        I2C_FLAG_STPDET: STOP condition detected in slave mode
      \arg        I2C_FLAG_BERR: bus error
      \arg        I2C_FLAG_LOSTARB: arbitration Lost
      \arg        I2C_FLAG_OUERR: overrun/underrun error in slave mode
      \arg        I2C_FLAG_PECERR: PEC error
      \arg        I2C_FLAG_TIMEOUT: timeout flag
      \arg        I2C_FLAG_SMBALT: SMBus Alert
    \param[out] none
    \retval     none
*/
void i2c_flag_clear(uint32_t i2c_periph, uint32_t flag)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003BU), ERR_PERIPH);
    } else if(NOT_I2C_STAT_CLEAR_FALG(flag)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003BU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_STATC(i2c_periph) |= flag;
    }
}

/*!
    \brief      enable I2C interrupt (API_ID:0x003CU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  interrupt: I2C interrupts
                only one parameter can be selected which is shown as below:
      \arg        I2C_INT_ERR: error interrupt
      \arg        I2C_INT_TC: transfer complete interrupt
      \arg        I2C_INT_STPDET: stop detection interrupt
      \arg        I2C_INT_NACK: not acknowledge received interrupt
      \arg        I2C_INT_ADDM: address match interrupt
      \arg        I2C_INT_RBNE: receive interrupt
      \arg        I2C_INT_TI: transmit interrupt
    \param[out] none
    \retval     none
*/
void i2c_interrupt_enable(uint32_t i2c_periph, uint32_t interrupt)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003CU), ERR_PERIPH);
    } else if(NOT_I2C_INT(interrupt)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003CU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) |= interrupt;
    }
}

/*!
    \brief      disable I2C interrupt (API_ID:0x003DU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  interrupt: I2C interrupts
                only one parameter can be selected which is shown as below:
      \arg        I2C_INT_ERR: error interrupt
      \arg        I2C_INT_TC: transfer complete interrupt
      \arg        I2C_INT_STPDET: stop detection interrupt
      \arg        I2C_INT_NACK: not acknowledge received interrupt
      \arg        I2C_INT_ADDM: address match interrupt
      \arg        I2C_INT_RBNE: receive interrupt
      \arg        I2C_INT_TI: transmit interrupt
    \param[out] none
    \retval     none
*/
void i2c_interrupt_disable(uint32_t i2c_periph, uint32_t interrupt)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003DU), ERR_PERIPH);
    } else if(NOT_I2C_INT(interrupt)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003DU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_CTL0(i2c_periph) &= ~interrupt;
    }
}

/*!
    \brief      get I2C interrupt flag status (API_ID:0x003EU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  int_flag: I2C interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        I2C_INT_FLAG_TI: transmit interrupt flag
      \arg        I2C_INT_FLAG_RBNE: I2C_RDATA is not empty during receiving interrupt flag
      \arg        I2C_INT_FLAG_ADDSEND: address received matches in slave mode interrupt flag
      \arg        I2C_INT_FLAG_NACK: not acknowledge interrupt flag
      \arg        I2C_INT_FLAG_STPDET: stop condition detected in slave mode interrupt flag
      \arg        I2C_INT_FLAG_TC: transfer complete in master mode interrupt flag
      \arg        I2C_INT_FLAG_TCR: transfer complete reload interrupt flag
      \arg        I2C_INT_FLAG_BERR: bus error interrupt flag
      \arg        I2C_INT_FLAG_LOSTARB: arbitration lost interrupt flag
      \arg        I2C_INT_FLAG_OUERR: overrun/underrun error in slave mode interrupt flag
      \arg        I2C_INT_FLAG_PECERR: PEC error interrupt flag
      \arg        I2C_INT_FLAG_TIMEOUT: timeout interrupt flag
      \arg        I2C_INT_FLAG_SMBALT: SMBus Alert interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus i2c_interrupt_flag_get(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag)
{
    FlagStatus reval = RESET;
    uint32_t ret1;
    uint32_t ret2;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003EU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* get the status of interrupt enable bit */
        ret1 = (I2C_REG_VAL(i2c_periph, int_flag) & BIT(I2C_BIT_POS(int_flag)));
        /* get the status of interrupt flag */
        ret2 = (I2C_REG_VAL2(i2c_periph, int_flag) & BIT(I2C_BIT_POS2(int_flag)));
        if(0 != (ret1 && ret2)) {
            reval = SET;
        } else {
            /* this code is written to avoid MISRA 15.7 (no 'else' at end of 'if ... else if') chain */
        }
    }
    return reval;
}

/*!
    \brief      clear I2C interrupt flag status (API_ID:0x003FU)
    \param[in]  i2c_periph: I2Cx(x=0,1,2,3)
    \param[in]  int_flag: I2C interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        I2C_INT_FLAG_ADDSEND: address received matches in slave mode interrupt flag
      \arg        I2C_INT_FLAG_NACK: not acknowledge interrupt flag
      \arg        I2C_INT_FLAG_STPDET: stop condition detected in slave mode interrupt flag
      \arg        I2C_INT_FLAG_BERR: bus error interrupt flag
      \arg        I2C_INT_FLAG_LOSTARB: arbitration lost interrupt flag
      \arg        I2C_INT_FLAG_OUERR: overrun/underrun error in slave mode interrupt flag
      \arg        I2C_INT_FLAG_PECERR: PEC error interrupt flag
      \arg        I2C_INT_FLAG_TIMEOUT: timeout interrupt flag
      \arg        I2C_INT_FLAG_SMBALT: SMBus Alert interrupt flag
    \param[out] none
    \retval     none
*/
void i2c_interrupt_flag_clear(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_I2C_PERIPH(i2c_periph)) {
        fw_debug_report_err(I2C_MODULE_ID, API_ID(0x003FU), ERR_PERIPH);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        I2C_STATC(i2c_periph) |= BIT(I2C_BIT_POS2(int_flag));
    }
}
