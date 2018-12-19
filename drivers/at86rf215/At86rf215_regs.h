/**
 * @file       At86rf215_regs.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef AT86RF215_REGS_HPP_
#define AT86RF215_REGS_HPP_

#define AT86RF215_READ_CMD              ( 0x00 )
#define AT86RF215_WRITE_CMD             ( 0x80 )

#define AT86RF215_PN_ADDR               ( 0x000D )
#define AT86RF215_PN_215                ( 0x34 )
#define AT86RF215_PN_215_IQ             ( 0x35 )
#define AT86RF215_PN_215_M              ( 0x36 )

#define AT86RF215_VN_ADDR               ( 0x000E )
#define AT86RF215_VN_1_1                ( 0x01 )
#define AT86RF215_VN_1_3                ( 0x03 )

#define AT86RF215_CFG_ADDR              ( 0x0006 )
#define AT86RF215_CFG_IRQMM_0           ( 0 << 3 )
#define AT86RF215_CFG_IRQMM_1           ( 1 << 3 )
#define AT86RF215_CFG_IRQP_0            ( 0 << 2 )
#define AT86RF215_CFG_IRQP_1            ( 1 << 2 )
#define AT86RF215_CFG_IDRV_2            ( 0 << 0 )
#define AT86RF215_CFG_IDRV_4            ( 1 << 0 )
#define AT86RF215_CFG_IDRV_6            ( 2 << 0 )
#define AT86RF215_CFG_IDRV_8            ( 3 << 0 )

#define AT86RF215_CLKO_ADDR             ( 0x0007 )
#define AT86RF215_CLKO_DRV_0            ( 0 << 3 )
#define AT86RF215_CLKO_DRV_1            ( 1 << 3 )
#define AT86RF215_CLKO_DRV_2            ( 2 << 3 )
#define AT86RF215_CLKO_DRV_3            ( 3 << 3 )
#define AT86RF215_CLKO_OS_OFF           ( 0 << 0 )
#define AT86RF215_CLKO_OS_26M           ( 1 << 0 )
#define AT86RF215_CLKO_OS_32M           ( 2 << 0 )
#define AT86RF215_CLKO_OS_16M           ( 3 << 0 )
#define AT86RF215_CLKO_OS_8M            ( 4 << 0 )
#define AT86RF215_CLKO_OS_4M            ( 5 << 0 )
#define AT86RF215_CLKO_OS_2M            ( 6 << 0 )
#define AT86RF215_CLKO_OS_1M            ( 7 << 0 )

#define AT86RF215_RST_ADDR              ( 0x0005 )
#define AT86RF215_RST_CMD               ( 0x07 )

#define AT86RF215_IQIFC1_ADDR           ( 0x0005 )
#define AT86RF215_IQIFC1_BBRF           ( 0 << 4 )
#define AT86RF215_IQIFC1_RF             ( 1 << 4 )
#define AT86RF215_IQIFC1_BBRF09         ( 4 << 4 )
#define AT86RF215_IQIFC1_BBRF24         ( 5 << 4 )

#define AT86RF215_RF09_STATE_ADDR       ( 0x0102 )
#define AT86RF215_RF24_STATE_ADDR       ( 0x0202 )

#define AT86RF215_RFn_STATE_ERROR       ( 0 << 0 )
#define AT86RF215_RFn_STATE_TRXOFF      ( 2 << 0 )
#define AT86RF215_RFn_STATE_TXPREP      ( 3 << 0 )
#define AT86RF215_RFn_STATE_TX          ( 4 << 0 )
#define AT86RF215_RFn_STATE_RX          ( 5 << 0 )
#define AT86RF215_RFn_STATE_TRANS       ( 6 << 0 )
#define AT86RF215_RFn_STATE_RESET       ( 7 << 0 )

#define AT86RF215_RF09_CMD_ADDR         ( 0x0103 )
#define AT86RF215_RF24_CMD_ADDR         ( 0x0203 )

#define AT86RF215_RFn_CMD_NOP           ( 0 << 0 )
#define AT86RF215_RFn_CMD_SLEEP         ( 1 << 0 )
#define AT86RF215_RFn_CMD_TRXOFF        ( 2 << 0 )
#define AT86RF215_RFn_CMD_TXPREP        ( 3 << 0 )
#define AT86RF215_RFn_CMD_TX            ( 4 << 0 )
#define AT86RF215_RFn_CMD_RX            ( 5 << 0 )
#define AT86RF215_RFn_CMD_RESET         ( 7 << 0 )

#define AT86RF215_RF09_IRQS_ADDR        ( 0x0000 )
#define AT86RF215_RF24_IRQS_ADDR        ( 0x0001 )
#define AT86RF215_BBC0_IRQS_ADDR        ( 0x0002 )
#define AT86RF215_BBC1_IRQS_ADDR        ( 0x0003 )

#define AT86RF215_RF09_IRQM_ADDR        ( 0x0100 )
#define AT86RF215_RF24_IRQM_ADDR        ( 0x0200 )
#define AT86RF215_BBC0_IRQM_ADDR        ( 0x0300 )
#define AT86RF215_BBC1_IRQM_ADDR        ( 0x0400 )

#define AT86RF215_BBCn_IRQM_FBLI_(X)    ( X << 7 )
#define AT86RF215_BBCn_IRQM_AGCR_(X)    ( X << 6 )
#define AT86RF215_BBCn_IRQM_AGCH_(X)    ( X << 5 )
#define AT86RF215_BBCn_IRQM_TXFE_(X)    ( X << 4 )
#define AT86RF215_BBCn_IRQM_RXEM_(X)    ( X << 3 )
#define AT86RF215_BBCn_IRQM_RXAM_(X)    ( X << 2 )
#define AT86RF215_BBCn_IRQM_RXFE_(X)    ( X << 1 )
#define AT86RF215_BBCn_IRQM_RXFS_(X)    ( X << 0 )

#define AT86RF215_RFn_IRQM_IQIFSF_(X)   ( X << 5 )
#define AT86RF215_RFn_IRQM_TRXERR_(X)   ( X << 4 )
#define AT86RF215_RFn_IRQM_BATLOW_(X)   ( X << 3 )
#define AT86RF215_RFn_IRQM_EDC_(X)      ( X << 2 )
#define AT86RF215_RFn_IRQM_TRXRDY_(X)   ( X << 1 )
#define AT86RF215_RFn_IRQM_WAKEUP_(X)   ( X << 0 )

#define AT86RF215_RFn_IRQS_IQIFSF       ( 0x20 )
#define AT86RF215_RFn_IRQS_TRXERR       ( 0x10 )
#define AT86RF215_RFn_IRQS_BATLOW       ( 0x08 )
#define AT86RF215_RFn_IRQS_EDC          ( 0x04 )
#define AT86RF215_RFn_IRQS_TRXRDY       ( 0x02 )
#define AT86RF215_RFn_IRQS_WAKEUP       ( 0x01 )

#define AT86RF215_BBCn_IRQS_FBLI        ( 0x80 )
#define AT86RF215_BBCn_IRQS_AGCR        ( 0x40 )
#define AT86RF215_BBCn_IRQS_AGCH        ( 0x20 )
#define AT86RF215_BBCn_IRQS_TXFE        ( 0x10 )
#define AT86RF215_BBCn_IRQS_RXEM        ( 0x08 )
#define AT86RF215_BBCn_IRQS_RXAM        ( 0x04 )
#define AT86RF215_BBCn_IRQS_RXFE        ( 0x02 )
#define AT86RF215_BBCn_IRQS_RXFS        ( 0x01 )

#define AT86RF215_RF09_RSSI_ADDR        ( 0x010D )
#define AT86RF215_RF24_RSSI_ADDR        ( 0x020D )

#define AT86RF215_RF09_EDC_ADDR         ( 0x010E )
#define AT86RF215_RF24_EDC_ADDR         ( 0x020E )

#define AT86RF215_RFn_EDC_EDM_AUTO      ( 0x00 )
#define AT86RF215_RFn_EDC_EDM_SINGLE    ( 0x01 )
#define AT86RF215_RFn_EDC_EDM_CONT      ( 0x02 )
#define AT86RF215_RFn_EDC_EDM_OFF       ( 0x03 )

#define AT86RF215_RF09_EDD_ADDR         ( 0x010F )
#define AT86RF215_RF24_EDD_ADDR         ( 0x020F )

#define AT86RF215_RFn_EDC_DF_US(X)      ( X << 2 )
#define AT86RF215_RFn_EDC_DF_DTB_2US    ( 0x00 )
#define AT86RF215_RFn_EDC_DF_DTB_8US    ( 0x01 )
#define AT86RF215_RFn_EDC_DF_DTB_32US   ( 0x02 )
#define AT86RF215_RFn_EDC_DF_DTB_128US  ( 0x03 )

#define AT86RF215_RF09_EDV_ADDR         ( 0x0110 )
#define AT86RF215_RF24_EDV_ADDR         ( 0x0210 )

#endif /* AT86RF215_REGS_HPP_ */
