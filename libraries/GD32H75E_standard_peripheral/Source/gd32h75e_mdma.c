/*!
    \file    gd32h75e_mdma.c
    \brief   MDMA driver

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

#include "gd32h75e_mdma.h"

#define MDMA_ADDRESS_MASK           ((uint32_t)0x0000FFFFU)                 /*!< MDMA multi-block update address mask */
#define STAT1_FLAG_MASK             ((uint32_t)0x00000F00U)                 /*!< MDMA_STAT1 flag mask */
#define CHXMBADDRU_DADDRUV_OFFSET   (16U)                                   /*!< destination update address offset */
#define CHXCFG_BTLEN_OFFSET         (18U)                                   /*!< bit offset of BTLEN in MDMA_CHxCFG */
#define CHXBTCFG_BRNUM_OFFSET       (20U)                                   /*!< bit offset of BRNUM in MDMA_CHxBTCFG */

/*!
    \brief      deinitialize MDMA (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mdma_deinit(void)
{
    /* reset MDMA */
    rcu_periph_reset_enable(RCU_MDMARST);
    rcu_periph_reset_disable(RCU_MDMARST);
}

/*!
    \brief      deinitialize MDMA registers of a channel (API_ID(0x0002U))
    \param[in]  channelx: specify which MDMA channel is deinitialized
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_deinit(mdma_channel_enum channelx)
{
    MDMA_CHXSTATC(channelx)   = 0x0000001FU;
    MDMA_CHXCTL0(channelx)    = 0U;
    MDMA_CHXCFG(channelx)     = 0U;
    MDMA_CHXBTCFG(channelx)   = 0U;
    MDMA_CHXSADDR(channelx)   = 0U;
    MDMA_CHXDADDR(channelx)   = 0U;
    MDMA_CHXMBADDRU(channelx) = 0U;
    MDMA_CHXLADDR(channelx)   = 0U;
    MDMA_CHXCTL1(channelx)    = 0U;
    MDMA_CHXMADDR(channelx)   = 0U;
    MDMA_CHXMDATA(channelx)   = 0U;
}

/*!
    \brief      initialize the MDMA single data mode parameters structure with the default values (API_ID(0x0003U))
    \param[in]  none
    \param[out] init_struct: the initialization data needed to initialize MDMA channel
    \retval     none
*/
void mdma_para_struct_init(mdma_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* set the MDMA struct with the default values */
        init_struct->request                  = 0U;
        init_struct->trans_trig_mode          = MDMA_BUFFER_TRANSFER;
        init_struct->priority                 = MDMA_PRIORITY_LOW;
        init_struct->endianness               = MDMA_LITTLE_ENDIANNESS;
        init_struct->source_inc               = MDMA_SOURCE_INCREASE_DISABLE;
        init_struct->dest_inc                 = MDMA_DESTINATION_INCREASE_DISABLE;
        init_struct->source_data_size         = MDMA_SOURCE_DATASIZE_8BIT;
        init_struct->dest_data_size           = MDMA_DESTINATION_DATASIZE_8BIT;
        init_struct->data_alignment           = MDMA_DATAALIGN_RIGHT;
        init_struct->buff_trans_len           = 0U;
        init_struct->source_burst             = MDMA_SOURCE_BURST_SINGLE;
        init_struct->dest_burst               = MDMA_DESTINATION_BURST_SINGLE;
        init_struct->mask_addr                = 0x00000000U;
        init_struct->mask_data                = 0x00000000U;
        init_struct->source_addr              = 0x00000000U;
        init_struct->dest_addr                = 0x00000000U;
        init_struct->tbytes_num_in_block      = 0U;
        init_struct->source_bus               = MDMA_SOURCE_AXI;
        init_struct->dest_bus                 = MDMA_DESTINATION_AXI;
        init_struct->bufferable_write_mode    = MDMA_BUFFERABLE_WRITE_DISABLE;
    }
}

/*!
    \brief      initialize the MDMA multi block transfer mode parameters structure with the default values (API_ID(0x0004U))
    \param[in]  none
    \param[out] block_init_struct: the initialization data needed to initialize MDMA multi block transfer
    \retval     none
*/
void mdma_multi_block_para_struct_init(mdma_multi_block_parameter_struct *block_init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(block_init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0004U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        block_init_struct->block_num          = 0U;
        block_init_struct->saddr_update_val   = 0U;
        block_init_struct->dstaddr_update_val = 0U;
        block_init_struct->saddr_update_dir   = UPDATE_DIR_INCREASE;
        block_init_struct->dstaddr_update_dir = UPDATE_DIR_INCREASE;
    }
}

/*!
    \brief      initialize the MDMA link node configuration structure with the default values (API_ID(0x0005U))
    \param[in]  none
    \param[out] node: the initialization data needed to initialize link node
    \retval     none
*/
void mdma_link_node_para_struct_init(mdma_link_node_parameter_struct *node)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0005U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        node->chxcfg_reg     = 0U;
        node->chxbtcfg_reg   = 0U;
        node->chxsaddr_reg   = 0U;
        node->chxdaddr_reg   = 0U;
        node->chxmbaddru_reg = 0U;
        node->chxladdr_reg   = 0U;
        node->chxctl1_reg    = 0U;
        node->reserved       = 0U;
        node->chxmaddr_reg   = 0U;
        node->chxmdata_reg   = 0U;
    }
}

/*!
    \brief      initialize MDMA channel with MDMA parameter structure (API_ID(0x0006U))
    \param[in]  channelx: specify which MDMA channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  init_struct: the data needed to initialize MDMA single data mode
                             and the member values are shown as below:
                  request: MDMA_REQUEST_DMA0_CH0_FTFIF, MDMA_REQUEST_DMA0_CH1_FTFIF, MDMA_REQUEST_DMA0_CH2_FTFIF, MDMA_REQUEST_DMA0_CH3_FTFIF, MDMA_REQUEST_DMA0_CH4_FTFIF,
                           MDMA_REQUEST_DMA0_CH5_FTFIF, MDMA_REQUEST_DMA0_CH6_FTFIF, MDMA_REQUEST_DMA0_CH7_FTFIF, MDMA_REQUEST_DMA1_CH0_FTFIF, MDMA_REQUEST_DMA1_CH1_FTFIF,
                           MDMA_REQUEST_DMA1_CH2_FTFIF, MDMA_REQUEST_DMA1_CH3_FTFIF, MDMA_REQUEST_DMA1_CH4_FTFIF, MDMA_REQUEST_DMA1_CH5_FTFIF, MDMA_REQUEST_DMA1_CH6_FTFIF,
                           MDMA_REQUEST_DMA1_CH7_FTFIF, MDMA_REQUEST_OSPI0_FT, MDMA_REQUEST_OSPI0_TC, MDMA_REQUEST_OSPI1_FT, MDMA_REQUEST_OSPI1_TC, MDMA_REQUEST_SW
                  trans_trig_mode: MDMA_BUFFER_TRANSFER, MDMA_BLOCK_TRANSFER, MDMA_MULTI_BLOCK_TRANSFER, MDMA_COMPLETE_TRANSFER
                  priority: MDMA_PRIORITY_LOW, MDMA_PRIORITY_MEDIUM, MDMA_PRIORITY_HIGH, MDMA_PRIORITY_ULTRA_HIGH
                  endianness: MDMA_LITTLE_ENDIANNESS, MDMA_BYTE_ENDIANNESS_EXCHANGE, MDMA_HALFWORD_ENDIANNESS_EXCHANGE, MDMA_WORD_ENDIANNESS_EXCHANGE
                  source_inc: MDMA_SOURCE_INCREASE_DISABLE, MDMA_SOURCE_INCREASE_8BIT, MDMA_SOURCE_INCREASE_16BIT, MDMA_SOURCE_INCREASE_32BIT, MDMA_SOURCE_INCREASE_64BIT,
                              MDMA_SOURCE_DECREASE_8BIT, MDMA_SOURCE_DECREASE_16BIT, MDMA_SOURCE_DECREASE_32BIT, MDMA_SOURCE_DECREASE_64BIT
                  dest_inc: MDMA_DESTINATION_INCREASE_DISABLE, MDMA_DESTINATION_INCREASE_8BIT, MDMA_DESTINATION_INCREASE_16BIT, MDMA_DESTINATION_INCREASE_32BIT, MDMA_DESTINATION_INCREASE_64BIT,
                            MDMA_DESTINATION_DECREASE_8BIT, MDMA_DESTINATION_DECREASE_16BIT, MDMA_DESTINATION_DECREASE_32BIT, MDMA_DESTINATION_DECREASE_64BIT
                  source_data_size: MDMA_SOURCE_DATASIZE_8BIT, MDMA_SOURCE_DATASIZE_16BIT, MDMA_SOURCE_DATASIZE_32BIT, MDMA_SOURCE_DATASIZE_64BIT
                  dest_data_size: MDMA_DESTINATION_DATASIZE_8BIT, MDMA_DESTINATION_DATASIZE_16BIT, MDMA_DESTINATION_DATASIZE_32BIT, MDMA_DESTINATION_DATASIZE_64BIT
                  data_alignment: MDMA_DATAALIGN_PKEN, MDMA_DATAALIGN_RIGHT, MDMA_DATAALIGN_RIGHT_SIGNED, MDMA_DATAALIGN_LEFT
                  buff_trans_len: the number of bytes to be transferred is (buff_trans_len+1), buff_trans_len ranges from 0 to 127
                  source_burst: MDMA_SOURCE_BURST_SINGLE, MDMA_SOURCE_BURST_2BEATS, MDMA_SOURCE_BURST_4BEATS, MDMA_SOURCE_BURST_8BEATS, MDMA_SOURCE_BURST_16BEATS,
                                MDMA_SOURCE_BURST_32BEATS, MDMA_SOURCE_BURST_64BEATS, MDMA_SOURCE_BURST_128BEATS
                  dest_burst: MDMA_DESTINATION_BURST_SINGLE, MDMA_DESTINATION_BURST_2BEATS, MDMA_DESTINATION_BURST_4BEATS, MDMA_DESTINATION_BURST_8BEATS, MDMA_DESTINATION_BURST_16BEATS,
                              MDMA_DESTINATION_BURST_32BEATS, MDMA_DESTINATION_BURST_64BEATS, MDMA_DESTINATION_BURST_128BEATS
                  mask_addr: mask address, ranges from 0x00000000 to 0xFFFFFFFF
                  mask_data: mask data, ranges from 0x00000000 to 0xFFFFFFFF
                  source_addr: source address, ranges from 0x00000000 to 0xFFFFFFFF
                  destination_addr: destination address, ranges from 0x00000000 to 0xFFFFFFFF
                  tbytes_num_in_block: transfer byte number of a block transfer, ranges from 0x00000000 to 0x0001FFFF
                  source_bus: MDMA_SOURCE_AXI, MDMA_SOURCE_AHB_TCM
                  dest_bus: MDMA_DESTINATION_AXI, MDMA_DESTINATION_AHB_TCM
                  bufferable_write_mode: MDMA_BUFFERABLE_WRITE_ENABLE, MDMA_BUFFERABLE_WRITE_DISABLE
    \param[out] none
    \retval     none
*/
void mdma_init(mdma_channel_enum channelx, mdma_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_POINTER);
    } else if(NOT_MDMA_REQUEST(init_struct->request)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_TRANS_TRIG(init_struct->trans_trig_mode)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_PRIORITY(init_struct->priority)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_ENDIANNESS(init_struct->endianness)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_INC(init_struct->source_inc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_INC(init_struct->dest_inc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_DATA_SIZE(init_struct->source_data_size)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_DATA_SIZE(init_struct->dest_data_size)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DATA_ALIGN(init_struct->data_alignment)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_BUFF_TRANS_LEN(init_struct->buff_trans_len)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_MDMA_SOURCE_BURST(init_struct->source_burst)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_BURST(init_struct->dest_burst)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID); 
    } else if(NOT_MDMA_TBYTES_NUM(init_struct->tbytes_num_in_block)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_BUS(init_struct->source_bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_BUS(init_struct->dest_bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_BUFFERABLE_WRITE(init_struct->bufferable_write_mode)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0006U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        mdma_channel_disable(channelx);

        /* configure endianness and priority */
        MDMA_CHXCTL0(channelx) &= ~(MDMA_CHXCTL0_BES | MDMA_CHXCTL0_HWES | MDMA_CHXCTL0_WES | MDMA_CHXCTL0_PRIO);
        MDMA_CHXCTL0(channelx) |= (init_struct->endianness | init_struct->priority);

        /* configure MDMA transfer width, memory transfer width, channel priority */
        MDMA_CHXCFG(channelx) = (init_struct->data_alignment | init_struct->dest_burst | init_struct->dest_data_size | init_struct->dest_inc | \
                init_struct->source_burst | init_struct->source_data_size | init_struct->source_inc | init_struct->trans_trig_mode | \
                ((init_struct->buff_trans_len << CHXCFG_BTLEN_OFFSET) & MDMA_CHXCFG_BTLEN) | init_struct->bufferable_write_mode);
        /* configure mask address, mask data */
        MDMA_CHXMADDR(channelx) = init_struct->mask_addr;
        MDMA_CHXMDATA(channelx) = init_struct->mask_data;

        /* configure source address */
        MDMA_CHXSADDR(channelx) = init_struct->source_addr;
        /* configure destination address */
        MDMA_CHXDADDR(channelx) = init_struct->dest_addr;

        /* configure block transfer byte number */
        if(MDMA_BUFFER_TRANSFER == init_struct->trans_trig_mode) {
            MDMA_CHXBTCFG(channelx) = (init_struct->tbytes_num_in_block & MDMA_CHXBTCFG_TBNUM);
        } else if(MDMA_BLOCK_TRANSFER == init_struct->trans_trig_mode) {
            MDMA_CHXBTCFG(channelx) = (init_struct->tbytes_num_in_block & MDMA_CHXBTCFG_TBNUM);
        } else if(MDMA_MULTI_BLOCK_TRANSFER == init_struct->trans_trig_mode) {
            MDMA_CHXBTCFG(channelx) &= ~MDMA_CHXBTCFG_TBNUM;
            MDMA_CHXBTCFG(channelx) |= (init_struct->tbytes_num_in_block & MDMA_CHXBTCFG_TBNUM);
        } else if(MDMA_COMPLETE_TRANSFER == init_struct->trans_trig_mode) {
            MDMA_CHXBTCFG(channelx) &= ~MDMA_CHXBTCFG_TBNUM;
            MDMA_CHXBTCFG(channelx) |= (init_struct->tbytes_num_in_block & MDMA_CHXBTCFG_TBNUM);
        } else {
            /* illegal parameters */
        }

        /* configure request source */
        MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_SWREQMOD;
        if(MDMA_REQUEST_SW == init_struct->request) {
            MDMA_CHXCFG(channelx) |= MDMA_CHXCFG_SWREQMOD;
        } else {
            MDMA_CHXCTL1(channelx) &= ~MDMA_CHXCTL1_TRIGSEL;
            MDMA_CHXCTL1(channelx) |= init_struct->request;
        }
        /* configure bus type for source and destination */
        MDMA_CHXCTL1(channelx) &= ~(MDMA_CHXCTL1_SBSEL | MDMA_CHXCTL1_DBSEL);
        MDMA_CHXCTL1(channelx) |= (init_struct->source_bus | init_struct->dest_bus);
    }
}

/*!
    \brief      configure MDMA buffer/block transfer mode (API_ID(0x0007U))
    \param[in]  channelx: specify which MDMA channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  saddr: source address, ranges from 0x00000000 to 0xFFFFFFFF
    \param[in]  daddr: destination address, ranges from 0x00000000 to 0xFFFFFFFF
    \param[in]  tbnum: number of bytes to transfer, ranges from 0x00000000 to 0x00010000
    \param[out] none
    \retval     none
*/
void mdma_buffer_block_mode_config(mdma_channel_enum channelx, uint32_t saddr, uint32_t daddr, uint32_t tbnum)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_TBNUM_OF_BLOCK_TRANS_MODE(tbnum)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0007U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXSADDR(channelx) = saddr;
        MDMA_CHXDADDR(channelx) = daddr;
        MDMA_CHXBTCFG(channelx) &= ~MDMA_CHXBTCFG_TBNUM;
        MDMA_CHXBTCFG(channelx) |= (tbnum & MDMA_CHXBTCFG_TBNUM);
    }
}

/*!
    \brief      configure MDMA multi block transfer mode (API_ID(0x0008U))
    \param[in]  channelx: specify which MDMA channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  tbnum: number of bytes to transfer in block, range from 0x00000000 to 0x00000FFF
    \param[in]  block_init_struct: the data needed to initialize MDMA multi block mode,
                the member values are shown as below:
                  block_num: multi block number, ranges from 0x00000000 to 0x00000FFF
                  saddr_update_val: source address update value, ranges from 0x0000 to 0xFFFF
                  dstaddr_update_val: destination address update value, ranges from 0x0000 to 0xFFFF
                  saddr_update_dir: source address update direction, UPDATE_DIR_INCREASE, UPDATE_DIR_DECREASE
                  dstaddr_update_dir: destination address update direction, UPDATE_DIR_INCREASE, UPDATE_DIR_DECREASE
    \param[out] none
    \retval     none
*/
void mdma_multi_block_mode_config(mdma_channel_enum channelx, uint32_t tbnum, mdma_multi_block_parameter_struct *block_init_struct)
{
    uint32_t blockoffset;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_TBNUM_OF_MULTI_BLOCK_TRANS_MODE(tbnum)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0008U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_VALID_POINTER(block_init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0008U), ERR_PARAM_POINTER);
    } else if(NOT_MDMA_MULTI_BLOCK_NUM(block_init_struct->block_num)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0008U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXBTCFG(channelx) = (((block_init_struct->block_num << CHXBTCFG_BRNUM_OFFSET) & MDMA_CHXBTCFG_BRNUM) | (tbnum & MDMA_CHXBTCFG_TBNUM));

        MDMA_CHXMBADDRU(channelx) &= ~(MDMA_CHXMBADDRU_SADDRUV | MDMA_CHXMBADDRU_DADDRUV);
        /* if block source address offset is negative, set the block repeat source address update mode to decrement */
        if(UPDATE_DIR_DECREASE == block_init_struct->saddr_update_dir) {
            MDMA_CHXBTCFG(channelx) |= MDMA_CHXBTCFG_SADDRUM;
            /* write new chxmbaddru register value: source repeat block offset */
            blockoffset = (uint32_t)block_init_struct->saddr_update_val;
            MDMA_CHXMBADDRU(channelx) |= (blockoffset & MDMA_ADDRESS_MASK);
        } else {
            MDMA_CHXBTCFG(channelx) &= ~MDMA_CHXBTCFG_SADDRUM;
            /* write new chxmbaddru register value: source repeat block offset */
            MDMA_CHXMBADDRU(channelx) |= (((uint32_t)block_init_struct->saddr_update_val) & MDMA_ADDRESS_MASK);
        }

        if(UPDATE_DIR_DECREASE == block_init_struct->dstaddr_update_dir) {
            MDMA_CHXBTCFG(channelx) |= MDMA_CHXBTCFG_DADDRUM;
            /* write new chxmbaddru register value: destination repeat block offset */
            blockoffset = (uint32_t)block_init_struct->dstaddr_update_val;
            MDMA_CHXMBADDRU(channelx) |= ((uint32_t)(blockoffset & MDMA_ADDRESS_MASK) << CHXMBADDRU_DADDRUV_OFFSET);
        } else {
            MDMA_CHXBTCFG(channelx) &= ~MDMA_CHXBTCFG_DADDRUM;
            /* write new chxmbaddru register value: destination repeat block offset */
            MDMA_CHXMBADDRU(channelx) |= ((((uint32_t)block_init_struct->dstaddr_update_val) & MDMA_ADDRESS_MASK) << CHXMBADDRU_DADDRUV_OFFSET);
        }
    }
}

/*!
    \brief      create MDMA link list node (API_ID(0x0009U))
    \param[in]  channelx: specify which MDMA channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] node: link node structure to create, members of the structure are shown as below:
                  chxcfg_reg: channel configure register
                  chxbtcfg_reg: channel block transfer configure register
                  chxsaddr_reg: channel source address register
                  chxdaddr_reg: channel destination address register
                  chxmbaddru_reg: channel multi-block address update register
                  chxladdr_reg: channel link address register
                  chxctl1_reg: channel control register 1
                  chxmaddr_reg: channel mask address register
                  chxmdata_reg: channel mask data register
    \param[in]  block_init_struct: mdma muti block structure, members of the structure and the member values are shown as below:
                  block_num: multi-block number, ranges from 0x00000000 to 0x00000FFF
                  saddr_update_val: source address update value, ranges from 0x0000 to 0xFFFF
                  dstaddr_update_val: destination address update value, ranges from 0x0000 to 0xFFFF
                  saddr_update_dir: source address update direction, UPDATE_DIR_INCREASE, UPDATE_DIR_DECREASE
                  dstaddr_update_dir: destination address update direction, UPDATE_DIR_INCREASE, UPDATE_DIR_DECREASE
    \param[in]  init_struct: the data needed to initialize MDMA single data mode,
                members of the structure and the member values are shown as below:
                  request: MDMA_REQUEST_x, x is the type of request
                  trans_trig_mode: MDMA_BUFFER_TRANSFER, MDMA_BLOCK_TRANSFER, MDMA_MULTI_BLOCK_TRANSFER, MDMA_COMPLETE_TRANSFER
                  priority: MDMA_PRIORITY_LOW, MDMA_PRIORITY_MEDIUM, MDMA_PRIORITY_HIGH, MDMA_PRIORITY_ULTRA_HIGH
                  endianness: MDMA_LITTLE_ENDIANNESS, MDMA_BYTE_ENDIANNESS_EXCHANGE, MDMA_HALFWORD_ENDIANNESS_EXCHANGE, MDMA_WORD_ENDIANNESS_EXCHANGE
                  source_inc: MDMA_SOURCE_INCREASE_DISABLE, MDMA_SOURCE_INCREASE_8BIT, MDMA_SOURCE_INCREASE_16BIT, MDMA_SOURCE_INCREASE_32BIT, MDMA_SOURCE_INCREASE_64BIT,
                              MDMA_SOURCE_DECREASE_8BIT, MDMA_SOURCE_DECREASE_16BIT, MDMA_SOURCE_DECREASE_32BIT, MDMA_SOURCE_DECREASE_64BIT
                  dest_inc: MDMA_DESTINATION_INCREASE_DISABLE, MDMA_DESTINATION_INCREASE_8BIT, MDMA_DESTINATION_INCREASE__16BIT, MDMA_DESTINATION_INCREASE_32BIT, MDMA_DESTINATION_INCREASE_64BIT,
                            MDMA_DESTINATION_DECREASE_8BIT, MDMA_DESTINATION_DECREASE_16BIT, MDMA_DESTINATION_DECREASE_32BIT, MDMA_DESTINATION_DECREASE_64BIT
                  source_data_size: MDMA_SOURCE_DATASIZE_8BIT, MDMA_SOURCE_DATASIZE_16BIT, MDMA_SOURCE_DATASIZE_32BIT, MDMA_SOURCE_DATASIZE_64BIT
                  dest_data_size: MDMA_DESTINATION_DATASIZE_8BIT, MDMA_DESTINATION_DATASIZE_16BIT, MDMA_DESTINATION_DATASIZE_32BIT, MDMA_DESTINATION_DATASIZE_64BIT
                  data_alignment: MDMA_DATAALIGN_PKEN, MDMA_DATAALIGN_RIGHT, MDMA_DATAALIGN_RIGHT_SIGNED, MDMA_DATAALIGN_LEFT
                  buff_trans_len: the number of bytes to be transferred is (buff_trans_len+1), buff_trans_len ranges from 0 to 127
                  source_burst: MDMA_SOURCE_BURST_SINGLE, MDMA_SOURCE_BURST_2BEATS, MDMA_SOURCE_BURST_4BEATS, MDMA_SOURCE_BURST_8BEATS, MDMA_SOURCE_BURST_16BEATS,
                                MDMA_SOURCE_BURST_32BEATS, MDMA_SOURCE_BURST_64BEATS, MDMA_SOURCE_BURST_128BEATS
                  dest_burst: MDMA_DESTINATION_BURST_SINGLE, MDMA_DESTINATION_BURST_2BEATS, MDMA_DESTINATION_BURST_4BEATS, MDMA_DESTINATION_BURST_8BEATS, MDMA_DESTINATION_BURST_16BEATS,
                              MDMA_DESTINATION_BURST_32BEATS, MDMA_DESTINATION_BURST_64BEATS, MDMA_DESTINATION_BURST_128BEATS
                  mask_addr: mask address, ranges from 0x00000000 to 0xFFFFFFFF
                  mask_data: mask data, ranges from 0x00000000 to 0xFFFFFFFF
                  source_addr: source address, ranges from 0x00000000 to 0xFFFFFFFF
                  dest_addr: destination address, ranges from 0x00000000 to 0xFFFFFFFF
                  tbytes_num_in_block: transfer byte number of a block transfer, ranges from 0x00000000 to 0x0001FFFF
                  source_bus: MDMA_SOURCE_AXI, MDMA_SOURCE_AHB_TCM
                  dest_bus: MDMA_DESTINATION_AXI, MDMA_DESTINATION_AHB_TCM
                  bufferable_write_mode: MDMA_BUFFERABLE_WRITE_ENABLE, MDMA_BUFFERABLE_WRITE_DISABLE
    \retval     none
*/
void mdma_node_create(mdma_link_node_parameter_struct *node, mdma_multi_block_parameter_struct *block_init_struct, mdma_parameter_struct *init_struct)
{
    uint32_t cfg, blockoffset;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(block_init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_POINTER);
    } else if(NOT_MDMA_MULTI_BLOCK_NUM(block_init_struct->block_num)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_POINTER);
    } else if(NOT_MDMA_REQUEST(init_struct->request)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_TRANS_TRIG(init_struct->trans_trig_mode)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_PRIORITY(init_struct->priority)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_ENDIANNESS(init_struct->endianness)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_INC(init_struct->source_inc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_INC(init_struct->dest_inc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_DATA_SIZE(init_struct->source_data_size)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_DATA_SIZE(init_struct->dest_data_size)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DATA_ALIGN(init_struct->data_alignment)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_BUFF_TRANS_LEN(init_struct->buff_trans_len)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_MDMA_SOURCE_BURST(init_struct->source_burst)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_BURST(init_struct->dest_burst)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID); 
    } else if(NOT_MDMA_TBYTES_NUM(init_struct->tbytes_num_in_block)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_SOURCE_BUS(init_struct->source_bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_DEST_BUS(init_struct->dest_bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else if(NOT_MDMA_BUFFERABLE_WRITE(init_struct->bufferable_write_mode)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0009U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure channel configure register */
        cfg = (init_struct->data_alignment | init_struct->dest_burst | init_struct->dest_data_size | init_struct->dest_inc | \
               init_struct->source_burst | init_struct->source_data_size | init_struct->source_inc | init_struct->trans_trig_mode | \
               ((init_struct->buff_trans_len << CHXCFG_BTLEN_OFFSET) & MDMA_CHXCFG_BTLEN) | init_struct->bufferable_write_mode);
        node->chxcfg_reg = cfg;

        /* configure channel request source */
        if(MDMA_REQUEST_SW == init_struct->request) {
            node->chxcfg_reg |= MDMA_CHXCFG_SWREQMOD;
        } else {
            node->chxctl1_reg &= ~MDMA_CHXCTL1_TRIGSEL;
            node->chxctl1_reg |= init_struct->request;
        }
        /* configure bus type for source and destination */
        node->chxctl1_reg &= ~(MDMA_CHXCTL1_SBSEL | MDMA_CHXCTL1_DBSEL);
        node->chxctl1_reg |= (init_struct->source_bus | init_struct->dest_bus);
        
        /* configure channel block transfer configure register */
        cfg = (((block_init_struct->block_num << CHXBTCFG_BRNUM_OFFSET) & MDMA_CHXBTCFG_BRNUM)| (init_struct->tbytes_num_in_block & MDMA_CHXBTCFG_TBNUM));
        node->chxbtcfg_reg = cfg;

        /* configure source address, destination address, mask address and mask data */
        node->chxsaddr_reg = init_struct->source_addr;
        node->chxdaddr_reg = init_struct->dest_addr;
        node->chxmaddr_reg = init_struct->mask_addr;
        node->chxmdata_reg = init_struct->mask_data;

        node->chxmbaddru_reg &= ~(MDMA_CHXMBADDRU_SADDRUV | MDMA_CHXMBADDRU_DADDRUV);
        /* if block source address offset is negative, set the block repeat source address update mode to decrement */
        if(UPDATE_DIR_DECREASE == block_init_struct->saddr_update_dir) {
            node->chxbtcfg_reg |= MDMA_CHXBTCFG_SADDRUM;
            /* write new chxmbaddru register value: source repeat block offset */
            blockoffset = (uint32_t)block_init_struct->saddr_update_val;
            node->chxmbaddru_reg |= (blockoffset & MDMA_ADDRESS_MASK);
        } else {
            node->chxbtcfg_reg &= ~MDMA_CHXBTCFG_SADDRUM;
            /* write new chxmbaddru register value: source repeat block offset */
            node->chxmbaddru_reg |= (((uint32_t)block_init_struct->saddr_update_val) & MDMA_ADDRESS_MASK);
        }

        if(UPDATE_DIR_DECREASE == block_init_struct->dstaddr_update_dir) {
            node->chxbtcfg_reg |= MDMA_CHXBTCFG_DADDRUM;
            /* write new chxmbaddru register value: destination repeat block offset */
            blockoffset = (uint32_t)block_init_struct->dstaddr_update_val;
            node->chxmbaddru_reg |= ((uint32_t)(blockoffset & MDMA_ADDRESS_MASK) << CHXMBADDRU_DADDRUV_OFFSET);
        } else {
            node->chxbtcfg_reg &= ~MDMA_CHXBTCFG_DADDRUM;
            /* write new chxmbaddru register value: destination repeat block offset */
            node->chxmbaddru_reg |= ((((uint32_t)block_init_struct->dstaddr_update_val) & MDMA_ADDRESS_MASK) << CHXMBADDRU_DADDRUV_OFFSET);
        }

        node->chxladdr_reg = 0U;
        node->reserved = 0U;
    }
}

/*!
    \brief      MDMA add node to link list (API_ID(0x000AU))
    \param[in]  pre_node: previous structure node pointer, members of the structure and the member values are shown as below:
                  chxcfg_reg: channel configure register
                  chxbtcfg_reg: channel block transfer configure register
                  chxsaddr_reg: channel source address register
                  chxdaddr_reg: channel destination address register
                  chxmbaddru_reg: channel multi-block address update register
                  chxladdr_reg: channel link address register
                  chxctl1_reg: channel control register 1
                  chxmaddr_reg: channel mask address register
                  chxmdata_reg: channel mask data register
    \param[in]  new_node: new node pointer, members of the structure and the member values are shown as below:
                  chxcfg_reg: channel configure register
                  chxbtcfg_reg: channel block transfer configure register
                  chxsaddr_reg: channel source address register
                  chxdaddr_reg: channel destination address register
                  chxmbaddru_reg: channel multi-block address update register
                  chxladdr_reg: channel link address register
                  chxctl1_reg: channel control register 1
                  chxmaddr_reg: channel mask address register
                  chxmdata_reg: channel mask data register
    \param[out] none
    \retval     none
*/
void mdma_node_add(mdma_link_node_parameter_struct *pre_node, mdma_link_node_parameter_struct *new_node)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(pre_node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000AU), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(new_node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000AU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        pre_node->chxladdr_reg = (uint32_t)new_node;
    }
}

/*!
    \brief      MDMA disconnect link list node (API_ID(0x000BU))
    \param[in]  pre_node: previous structure node pointer, members of the structure and the member values are shown as below:
                  chxcfg_reg: channel configure register
                  chxbtcfg_reg: channel block transfer configure register
                  chxsaddr_reg: channel source address register
                  chxdaddr_reg: channel destination address register
                  chxmbaddru_reg: channel multi-block address update register
                  chxladdr_reg: channel link address register
                  chxctl1_reg: channel control register 1
                  chxmaddr_reg: channel mask address register
                  chxmdata_reg: channel mask data register
    \param[in]  unused_node: unused link list node pointer, members of the structure and the member values are shown as below:
                  chxcfg_reg: channel configure register
                  chxbtcfg_reg: channel block transfer configure register
                  chxsaddr_reg: channel source address register
                  chxdaddr_reg: channel destination address register
                  chxmbaddru_reg: channel multi-block address update register
                  chxladdr_reg: channel link address register
                  chxctl1_reg: channel control register 1
                  chxmaddr_reg: channel mask address register
                  chxmdata_reg: channel mask data register
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus mdma_node_delete(mdma_link_node_parameter_struct *pre_node, mdma_link_node_parameter_struct *unused_node)
{
    ErrStatus reval = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(pre_node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000BU), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(unused_node)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000BU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(pre_node->chxladdr_reg != (uint32_t)unused_node) {
            /* link address unmatched */
            reval = ERROR;
        } else {
            /* link address matched, disconnect the unused node from link list */
            pre_node->chxladdr_reg = 0U;
            reval = SUCCESS;
        }
    }
    return reval;
}

/*!
    \brief      configure MDMA destination base address (API_ID(0x000CU))
    \param[in]  channelx: specify which MDMA channel to set peripheral base address
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  address: destination base address, ranges from 0x00000000 to 0xFFFFFFFF
    \param[out] none
    \retval     none
*/
void mdma_destination_address_config(mdma_channel_enum channelx, uint32_t address)
{
    MDMA_CHXDADDR(channelx) = address;
}

/*!
    \brief      configure MDMA source base address (API_ID(0x000DU))
    \param[in]  channelx: specify which MDMA channel to set Memory base address
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  address: source base address, ranges from 0x00000000 to 0xFFFFFFFF
    \param[out] none
    \retval     none
*/
void mdma_source_address_config(mdma_channel_enum channelx, uint32_t address)
{
    MDMA_CHXSADDR(channelx) = address;
}

/*!
    \brief      configure MDMA destination bus (API_ID(0x000EU))
    \param[in]  channelx: specify which MDMA channel to set bus
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  bus: destination bus
      \arg        MDMA_DESTINATION_AXI: destination bus is the system bus or AXI bus
      \arg        MDMA_DESTINATION_AHB_TCM: destination bus is AHB bus or TCM
    \param[out] none
    \retval     none
*/
void mdma_destination_bus_config(mdma_channel_enum channelx, uint32_t bus)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_DEST_BUS(bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000EU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXCTL1(channelx) &= ~MDMA_CHXCTL1_DBSEL;
        MDMA_CHXCTL1(channelx) |= bus;
    }
}

/*!
    \brief      configure MDMA source bus (API_ID(0x000FU))
    \param[in]  channelx: specify which MDMA channel to set bus
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  bus: source bus
      \arg        MDMA_SOURCE_AXI: source bus is the system bus or AXI bus
      \arg        MDMA_SOURCE_AHB_TCM: source bus is AHB bus or TCM
    \param[out] none
    \retval     none
*/
void mdma_source_bus_config(mdma_channel_enum channelx, uint32_t bus)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_SOURCE_BUS(bus)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x000FU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXCTL1(channelx) &= ~MDMA_CHXCTL1_SBSEL;
        MDMA_CHXCTL1(channelx) |= bus; 
    }
}

/*!
    \brief      configure priority level of MDMA channel (API_ID(0x0010U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  priority: priority level of this channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_PRIORITY_LOW: low priority
      \arg        MDMA_PRIORITY_MEDIUM: medium priority
      \arg        MDMA_PRIORITY_HIGH: high priority
      \arg        MDMA_PRIORITY_ULTRA_HIGH: ultra high priority
    \param[out] none
    \retval     none
*/
void mdma_priority_config(mdma_channel_enum channelx, uint32_t priority)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_PRIORITY(priority)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0010U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure priority level */
        MDMA_CHXCTL0(channelx) &= ~MDMA_CHXCTL0_PRIO;
        MDMA_CHXCTL0(channelx) |= priority;
    }
}

/*!
    \brief      configure endianness of MDMA channel (API_ID(0x0011U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  endianness: MDMA endianness
                only one parameter can be selected which is shown as below:
      \arg        MDMA_LITTLE_ENDIANNESS: little endianness preserve
      \arg        MDMA_BYTE_ENDIANNESS_EXCHANGE: exchange the order of the bytes in a half-word
      \arg        MDMA_HALFWORD_ENDIANNESS_EXCHANGE: exchange the order of the half-words in a word
      \arg        MDMA_WORD_ENDIANNESS_EXCHANGE: exchange the order of the words in a double word
    \param[out] none
    \retval     none
*/
void mdma_endianness_config(mdma_channel_enum channelx, uint32_t endianness)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_ENDIANNESS(endianness)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0011U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure MDMA endianness */
        MDMA_CHXCTL0(channelx) &= ~(MDMA_CHXCTL0_BES | MDMA_CHXCTL0_HWES | MDMA_CHXCTL0_WES);
        MDMA_CHXCTL0(channelx) |= endianness;
    }
}

/*!
    \brief      configure data alignment of MDMA channel (API_ID(0x0012U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  alignment: MDMA data alignment
                only one parameter can be selected which is shown as below:
      \arg        MDMA_DATAALIGN_PKEN: pack/unpack the source data to match the destination data size
      \arg        MDMA_DATAALIGN_RIGHT: right aligned, padded with 0s (default)
      \arg        MDMA_DATAALIGN_RIGHT_SIGNED: right aligned with sign extended, note: this mode is allowed only if the source data size is smaller than destination data size
      \arg        MDMA_DATAALIGN_LEFT: left aligned, padded with 0s in low bytes position when source data size smaller than destination data size, and only high byte of source is written when source data size larger than destination data size
    \param[out] none
    \retval     none
*/
void mdma_alignment_config(mdma_channel_enum channelx, uint32_t alignment)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_DATA_ALIGN(alignment)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0012U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure MDMA endianness */
        MDMA_CHXCFG(channelx) &= ~(MDMA_CHXCFG_PKEN | MDMA_CHXCFG_PAMOD);
        MDMA_CHXCFG(channelx) |= alignment;
    }
}

/*!
    \brief      configure transfer burst beats of source (API_ID(0x0013U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  sbeat: source transfer burst beats
      \arg        MDMA_SOURCE_BURST_SINGLE: single burst
      \arg        MDMA_SOURCE_BURST_2BEATS: 2-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_4BEATS: 4-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_8BEATS: 8-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_16BEATS: 16-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_32BEATS: 32-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_64BEATS: 64-beat incrementing burst
      \arg        MDMA_SOURCE_BURST_128BEATS: 128-beat incrementing burst
    \param[out] none
    \retval     none
*/
void mdma_source_burst_beats_config(mdma_channel_enum channelx, uint32_t sbeat)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_SOURCE_BURST(sbeat)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0013U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure transfer burst beats of source */
        MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_SBURST;
        MDMA_CHXCFG(channelx) |= sbeat;
    }
}

/*!
    \brief      configure transfer burst beats of destination (API_ID(0x0014U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in] dbeat: destination transfer burst beats
                only one parameter can be selected which is shown as below:
      \arg        MDMA_DESTINATION_BURST_SINGLE: single burst
      \arg        MDMA_DESTINATION_BURST_2BEATS: 2-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_4BEATS: 4-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_8BEATS: 8-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_16BEATS: 16-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_32BEATS: 32-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_64BEATS: 64-beat incrementing burst
      \arg        MDMA_DESTINATION_BURST_128BEATS: 128-beat incrementing burst
    \param[out] none
    \retval     none
*/
void mdma_destination_burst_beats_config(mdma_channel_enum channelx, uint32_t dbeat)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_DEST_BURST(dbeat)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0014U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure transfer burst beats of destination */
        MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_DBURST;
        MDMA_CHXCFG(channelx) |= dbeat;
    }
}

/*!
    \brief      configure data size of source (API_ID(0x0015U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  swidth: source data size
                only one parameter can be selected which is shown as below:
      \arg        MDMA_SOURCE_DATASIZE_8BIT: data size of source is 8-bit
      \arg        MDMA_SOURCE_DATASIZE_16BIT: data size of source is 16-bit
      \arg        MDMA_SOURCE_DATASIZE_32BIT: data size of source is 32-bit
      \arg        MDMA_SOURCE_DATASIZE_64BIT: data size of source is 64-bit
    \param[out] none
    \retval     none
*/
void mdma_source_width_config(mdma_channel_enum channelx, uint32_t swidth)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_SOURCE_DATA_SIZE(swidth)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0015U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure data size of source */
        MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_SWIDTH;
        MDMA_CHXCFG(channelx) |= swidth;
    }
}

/*!
    \brief      configure data size of destination (API_ID(0x0016U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  dwidth: destination data size
                only one parameter can be selected which is shown as below:
      \arg        MDMA_DESTINATION_DATASIZE_8BIT: data size of destination is 8-bit
      \arg        MDMA_DESTINATION_DATASIZE_16BIT: data size of destination is 16-bit
      \arg        MDMA_DESTINATION_DATASIZE_32BIT: data size of destination is 32-bit
      \arg        MDMA_DESTINATION_DATASIZE_64BIT: data size of destination is 64-bit
    \param[out] none
    \retval     none
*/
void mdma_destination_width_config(mdma_channel_enum channelx, uint32_t dwidth)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_DEST_DATA_SIZE(dwidth)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0016U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure data size of source */
        MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_DWIDTH;
        MDMA_CHXCFG(channelx) |= dwidth;
    }
}

/*!
    \brief      configure source address increment mode (API_ID(0x0017U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  sinc: source address increment mode
                only one parameter can be selected which is shown as below:
      \arg        MDMA_SOURCE_INCREASE_DISABLE: no increment
      \arg        MDMA_SOURCE_INCREASE_8BIT: source address is increased by 8-bit
      \arg        MDMA_SOURCE_INCREASE_16BIT: source address is increased by 16-bit
      \arg        MDMA_SOURCE_INCREASE_32BIT: source address is increased by 32-bit
      \arg        MDMA_SOURCE_INCREASE_64BIT: source address is increased by 64-bit
      \arg        MDMA_SOURCE_DECREASE_8BIT: source address is decreased by 8-bit
      \arg        MDMA_SOURCE_DECREASE_16BIT: source address is decreased by 16-bit
      \arg        MDMA_SOURCE_DECREASE_32BIT: source address is decreased by 32-bit
      \arg        MDMA_SOURCE_DECREASE_64BIT: source address is decreased by 64-bit
    \param[out] none
    \retval     none
*/
void mdma_source_increment_config(mdma_channel_enum channelx, uint32_t sinc)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_SOURCE_INC(sinc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0017U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure address increment mode of source */
        MDMA_CHXCFG(channelx) &= ~(MDMA_CHXCFG_SIMOD | MDMA_CHXCFG_SIOS);
        MDMA_CHXCFG(channelx) |= sinc;
    }
}

/*!
    \brief      configure destination address increment mode (API_ID(0x0018U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  dinc: destination address increment mode
                only one parameter can be selected which is shown as below:
      \arg        MDMA_DESTINATION_INCREASE_DISABLE: no increment
      \arg        MDMA_DESTINATION_INCREASE_8BIT: destination address is increased by 8-bit
      \arg        MDMA_DESTINATION_INCREASE_16BIT: destination address is increased by 16-bit
      \arg        MDMA_DESTINATION_INCREASE_32BIT: destination address is increased by 32-bit
      \arg        MDMA_DESTINATION_INCREASE_64BIT: destination address is increased by 64-bit
      \arg        MDMA_DESTINATION_DECREASE_8BIT: destination address is decreased by 8-bit
      \arg        MDMA_DESTINATION_DECREASE_16BIT: destination address is decreased by 16-bit
      \arg        MDMA_DESTINATION_DECREASE_32BIT: destination address is decreased by 32-bit
      \arg        MDMA_DESTINATION_DECREASE_64BIT: destination address is decreased by 64-bit
    \param[out] none
    \retval     none
*/
void mdma_destination_increment_config(mdma_channel_enum channelx, uint32_t dinc)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_DEST_INC(dinc)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0018U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure address increment mode of destination */
        MDMA_CHXCFG(channelx) &= ~(MDMA_CHXCFG_DIMOD | MDMA_CHXCFG_DIOS);
        MDMA_CHXCFG(channelx) |= dinc;
    }
}

/*!
    \brief      enable MDMA channel bufferable write mode (API_ID(0x0019U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_bufferable_write_enable(mdma_channel_enum channelx)
{
    MDMA_CHXCFG(channelx) |= MDMA_CHXCFG_BWMOD;
}

/*!
    \brief      disable MDMA channel bufferable write mode (API_ID(0x001AU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_bufferable_write_disable(mdma_channel_enum channelx)
{
    MDMA_CHXCFG(channelx) &= ~MDMA_CHXCFG_BWMOD;
}

/*!
    \brief      enable MDMA channel software request (API_ID(0x001BU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_software_request_enable(mdma_channel_enum channelx)
{
    MDMA_CHXCTL0(channelx) |= MDMA_CHXCTL0_SWREQ;
}

/*!
    \brief      enable MDMA channel (API_ID(0x001CU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_enable(mdma_channel_enum channelx)
{
    MDMA_CHXCTL0(channelx) |= MDMA_CHXCTL0_CHEN;
}

/*!
    \brief      disable MDMA channel (API_ID(0x001DU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     none
*/
void mdma_channel_disable(mdma_channel_enum channelx)
{
    MDMA_CHXCTL0(channelx) &= ~MDMA_CHXCTL0_CHEN;
}

/*!
    \brief      get MDMA transfer error direction (API_ID(0x001EU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     transfer error direction: MDMA_READ_ERROR or MDMA_WRITE_ERROR
*/
uint32_t mdma_transfer_error_direction_get(mdma_channel_enum channelx)
{
    return (MDMA_CHXSTAT1(channelx) & MDMA_CHXSTAT1_TERRD);
}

/*!
    \brief      get MDMA transfer error address (API_ID(0x001FU))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[out] none
    \retval     the low 7 bits of the transfer error address
*/
uint32_t mdma_transfer_error_address_get(mdma_channel_enum channelx)
{
    return (uint32_t)(MDMA_CHXSTAT1(channelx) & MDMA_CHXSTAT1_ERRADDR);
}

/*!
    \brief      get MDMA flag (API_ID(0x0020U))
    \param[in]  channelx: specify which MDMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_FLAG_ERR: transfer error flag
      \arg        MDMA_FLAG_CHTCF: channel transfer complete flag
      \arg        MDMA_FLAG_MBTCF: multi-block transfer complete flag
      \arg        MDMA_FLAG_BTCF: block transfer complete flag
      \arg        MDMA_FLAG_TCF: buffer transfer complete flag
      \arg        MDMA_FLAG_REQAF: request active flag
      \arg        MDMA_FLAG_LDTERR: link data transfer error flag in the last transfer of the channel
      \arg        MDMA_FLAG_MDTERR: mask data error flag
      \arg        MDMA_FLAG_ASERR: address and size error flag
      \arg        MDMA_FLAG_BZERR: block size error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mdma_flag_get(mdma_channel_enum channelx, uint32_t flag)
{
    FlagStatus reval = RESET;
    uint32_t flag_pos = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_FLAG(flag)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0020U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(STAT1_FLAG & flag) {
            /* get the flag in CHXSTAT1 */
            flag_pos = (flag & STAT1_FLAG_MASK);
            if(MDMA_CHXSTAT1(channelx) & flag_pos) {
                reval = SET;
            } else {
                reval = RESET;
            }
        } else {
            /* get the flag in CHXSTAT0 */
            if(MDMA_CHXSTAT0(channelx) & flag) {
                reval = SET;
            } else {
                reval = RESET;
            }
        }
    }
    return reval;
}

/*!
    \brief      clear MDMA flag (API_ID(0x0021U))
    \param[in]  channelx: specify which MDMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  flag: specify clear which flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_FLAG_ERR: transfer error flag
      \arg        MDMA_FLAG_CHTCF: channel transfer complete flag
      \arg        MDMA_FLAG_MBTCF: multi-block transfer complete flag
      \arg        MDMA_FLAG_BTCF: block transfer complete flag
      \arg        MDMA_FLAG_TCF: buffer transfer complete flag
      \arg        MDMA_FLAG_LDTERR: link data transfer error flag in the last transfer of the channel
      \arg        MDMA_FLAG_MDTERR: mask data error flag
      \arg        MDMA_FLAG_ASERR: address and size error flag
      \arg        MDMA_FLAG_BZERR: block size error flag
    \param[out] none
    \retval     none
*/
void mdma_flag_clear(mdma_channel_enum channelx, uint32_t flag)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_CLEAR_FLAG(flag)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0021U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(STAT1_FLAG & flag) {
            MDMA_CHXSTATC(channelx) |= MDMA_CHXSTATC_ERRC;
        } else {
            MDMA_CHXSTATC(channelx) |= flag;
        }
    }
}

/*!
    \brief      enable MDMA interrupt (API_ID(0x0022U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  interrupt: specify which interrupt to enbale
                one or more parameters can be selected which are shown as below:
      \arg        MDMA_INT_ERR: transfer error interrupt
      \arg        MDMA_INT_CHTC: channel transfer complete interrupt
      \arg        MDMA_INT_MBTC: multi-block transfer complete interrupt
      \arg        MDMA_INT_BTC: block transfer complete interrupt
      \arg        MDMA_INT_TC: buffer transfer complete interrupt
    \param[out] none
    \retval     none
*/
void mdma_interrupt_enable(mdma_channel_enum channelx, uint32_t interrupt)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_INT(interrupt)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0022U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXCTL0(channelx) |= interrupt;
    }
}

/*!
    \brief      disable MDMA interrupt (API_ID(0x0023U))
    \param[in]  channelx: specify which MDMA channel
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  interrupt: specify which interrupt to disbale
                one or more parameters can be selected which are shown as below:
      \arg        MDMA_INT_ERR: transfer error interrupt
      \arg        MDMA_INT_CHTC: channel transfer complete interrupt
      \arg        MDMA_INT_MBTC: multi-block transfer complete interrupt
      \arg        MDMA_INT_BTC: block transfer complete interrupt
      \arg        MDMA_INT_TC: buffer transfer complete interrupt
    \param[out] none
    \retval     none
*/
void mdma_interrupt_disable(mdma_channel_enum channelx, uint32_t interrupt)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_INT(interrupt)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0023U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXCTL0(channelx) &= ~interrupt;

    }
}

/*!
    \brief      get MDMA interrupt flag (API_ID(0x0024U))
    \param[in]  channelx: specify which MDMA channel to get flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  int_flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_INT_FLAG_ERR: transfer error interrupt flag
      \arg        MDMA_INT_FLAG_CHTCF: channel transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_MBTCF: multi-block transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_BTCF: block transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_TCF: buffer transfer complete interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mdma_interrupt_flag_get(mdma_channel_enum channelx, uint32_t int_flag)
{
    FlagStatus reval = RESET;
    uint32_t interrupt_enable = 0U, interrupt_flag = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_INT_FLAG(int_flag)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0024U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        switch(int_flag) {
        case MDMA_INT_FLAG_ERR:
            /* get error interrupt enable bit and flag bit */
            interrupt_enable = (MDMA_CHXCTL0(channelx) & MDMA_CHXCTL0_ERRIE);
            interrupt_flag = (MDMA_CHXSTAT0(channelx) & int_flag);
            break;
        case MDMA_INT_FLAG_CHTCF:
            /* get channel transfer complete interrupt enable bit and flag bit */
            interrupt_enable = (MDMA_CHXCTL0(channelx) & MDMA_CHXCTL0_CHTCIE);
            interrupt_flag = (MDMA_CHXSTAT0(channelx) & int_flag);
            break;
        case MDMA_INT_FLAG_MBTCF:
            /* get multi-block transfer complete interrupt enable bit and flag bit */
            interrupt_enable = (MDMA_CHXCTL0(channelx) & MDMA_CHXCTL0_MBTCIE);
            interrupt_flag = (MDMA_CHXSTAT0(channelx) & int_flag);
            break;
        case MDMA_INT_FLAG_BTCF:
            /* get block transfer complete interrupt enable bit and flag bit */
            interrupt_enable = (MDMA_CHXCTL0(channelx) & MDMA_CHXCTL0_BTCIE);
            interrupt_flag = (MDMA_CHXSTAT0(channelx) & int_flag);
            break;
        case MDMA_INT_FLAG_TCF:
            /* get buffer transfer complete interrupt enable bit and flag bit */
            interrupt_enable = (MDMA_CHXCTL0(channelx) & MDMA_CHXCTL0_TCIE);
            interrupt_flag = (MDMA_CHXSTAT0(channelx) & int_flag);
            break;
        default:
            break;
        }

        if(interrupt_flag && interrupt_enable) {
            reval = SET;
        } else {
            reval = RESET;
        }
    }
    return reval;
}

/*!
    \brief      clear MDMA interrupt flag (API_ID(0x0025U))
    \param[in]  channelx: specify which MDMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_CHx(x=0..15)
    \param[in]  int_flag: specify clear which flag
                only one parameter can be selected which is shown as below:
      \arg        MDMA_INT_FLAG_ERR: transfer error interrupt flag
      \arg        MDMA_INT_FLAG_CHTCF: channel transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_MBTCF: multi-block transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_BTCF: block transfer complete interrupt flag
      \arg        MDMA_INT_FLAG_TCF: buffer transfer complete interrupt flag
    \param[out] none
    \retval     none
*/
void mdma_interrupt_flag_clear(mdma_channel_enum channelx, uint32_t int_flag)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_MDMA_INT_FLAG(int_flag)) {
        fw_debug_report_err(MDMA_MODULE_ID, API_ID(0x0025U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        MDMA_CHXSTATC(channelx) |= int_flag;

    }
}

