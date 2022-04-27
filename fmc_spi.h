#pragma once
#include "../wr_base.h"

#define FALSE 0L
#define TRUE 1L

#define XST_DEVICE_BUSY 21L	  /*!< Device is busy */

#define FmcSpi_IsIntrGlobalEnabled() (Vpx_read_spi(fmc_spi.BaseAddr + FmcSP_DGIER_OFFSET) == FmcSP_GINTR_ENABLE_MASK)
#define FmcSpi_IntrGlobalDisable() Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_DGIER_OFFSET, 0)
#define FmcSpi_GetControlReg() Vpx_read_spi(fmc_spi.BaseAddr + FmcSP_CR_OFFSET)
#define FmcSpi_GetStatusReg() Vpx_read_spi(fmc_spi.BaseAddr + FmcSP_SR_OFFSET)
#define FmcSpi_WriteReg(RegOffset, RegisterValue) Vpx_write_spi(fmc_spi.BaseAddr + RegOffset, RegisterValue)
#define FmcSpi_SetSlaveSelectReg(Mask) Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_SSR_OFFSET, Mask)
#define FmcSpi_SetControlReg(Mask) Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_CR_OFFSET, Mask)
#define FmcSpi_Reset(Mask) Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_SRR_OFFSET, Mask)
#define FmcSpi_IntrGetStatus() Vpx_read_spi(fmc_spi.BaseAddr + FmcSP_IISR_OFFSET)
#define FmcSpi_IntrClear(ClearMask) Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_IISR_OFFSET, FmcSpi_IntrGetStatus() | ClearMask)
#define FmcSpi_ReadReg(RegOffset) Vpx_read_spi(fmc_spi.BaseAddr + RegOffset)

#define FmcSP_DGIER_OFFSET 0x1C /**< Global Intr Enable Reg */
#define FmcSP_IISR_OFFSET 0x20  /**< Interrupt status Reg */
#define FmcSP_IIER_OFFSET 0x28  /**< Interrupt Enable Reg */
#define FmcSP_SRR_OFFSET 0x40   /**< Software Reset register */
#define FmcSP_CR_OFFSET 0x60    /**< Control register */
#define FmcSP_SR_OFFSET 0x64    /**< Status Register */
#define FmcSP_DTR_OFFSET 0x68   /**< Data transmit */
#define FmcSP_DRR_OFFSET 0x6C   /**< Data receive */
#define FmcSP_SSR_OFFSET 0x70   /**< 32-bit slave select */
#define FmcSP_TFO_OFFSET 0x74   /**< Tx FIFO occupancy */
#define FmcSP_RFO_OFFSET 0x78   /**< Rx FIFO occupancy */

#define FmcSP_CR_LOOPBACK_MASK 0x00000001      /**< Local loopback mode */
#define FmcSP_CR_ENABLE_MASK 0x00000002        /**< System enable */
#define FmcSP_CR_MASTER_MODE_MASK 0x00000004   /**< Enable master mode */
#define FmcSP_CR_CLK_POLARITY_MASK 0x00000008  /**< Clock polarity high or low */
#define FmcSP_CR_CLK_PHASE_MASK 0x00000010     /**< Clock phase 0 or 1 */
#define FmcSP_CR_TXFIFO_RESET_MASK 0x00000020  /**< Reset transmit FIFO */
#define FmcSP_CR_RXFIFO_RESET_MASK 0x00000040  /**< Reset receive FIFO */
#define FmcSP_CR_MANUAL_SS_MASK 0x00000080     /**< Manual slave selectassert */
#define FmcSP_CR_TRANS_INHIBIT_MASK 0x00000100 /**< Master transactioninhibit */

#define FmcSP_SR_RX_EMPTY_MASK 0x00000001   /**< Receive Reg/FIFO is empty */
#define FmcSP_SR_RX_FULL_MASK 0x00000002    /**< Receive Reg/FIFO is full */
#define FmcSP_SR_TX_EMPTY_MASK 0x00000004   /**< Transmit Reg/FIFO is empty */
#define FmcSP_SR_TX_FULL_MASK 0x00000008    /**< Transmit Reg/FIFO is full */
#define FmcSP_SR_MODE_FAULT_MASK 0x00000010 /**< Mode fault error */
#define FmcSP_SR_SLAVE_MODE_MASK 0x00000020 /**< Slave mode select */
#define FmcSP_GINTR_ENABLE_MASK 0x80000000  /**< Global interrupt enable */

#define FmcSP_INTR_MODE_FAULT_MASK 0x00000001       /**< Mode fault error */
#define FmcSP_INTR_SLAVE_MODE_FAULT_MASK 0x00000002 /**< Selected as slave while disabled */
#define FmcSP_INTR_TX_EMPTY_MASK 0x00000004         /**< DTR/TxFIFO is empty */
#define FmcSP_INTR_TX_UNDERRUN_MASK 0x00000008      /**< DTR/TxFIFO underrun */
#define FmcSP_INTR_RX_FULL_MASK 0x00000010          /**< DRR/RxFIFO is full */
#define FmcSP_INTR_RX_OVERRUN_MASK 0x00000020       /**< DRR/RxFIFO overrun */
#define FmcSP_INTR_TX_HALF_EMPTY_MASK 0x00000040    /**< TxFIFO is half empty */
#define FmcSP_INTR_SLAVE_MODE_MASK 0x00000080       /**< Slave select mode */
#define FmcSP_INTR_RX_NOT_EMPTY_MASK 0x00000100     /**< RxFIFO not empty */

#define FmcSP_SRR_RESET_MASK 0x0000000A

#define TOTAL_BUFFER_SIZE 3

struct XSpiFmc
{
    uint32_t BaseAddr;                                  /**< Base address of device (IPIF) */
    uint8_t NumSlaveBits = 1;                           /**< Number of slave selects for this device */
    uint8_t DataWidth = 8;                              /**< Data Transfer Width 8 or 16 or 32 */
    uint32_t SlaveSelectMask = (1 << NumSlaveBits) - 1; /**< Mask that matches the number of SS bits */
    uint32_t SlaveSelectReg = (1 << NumSlaveBits) - 1;  /**< Slave select register */
    int IsBusy = 0;                                     /**< A transfer is in progress (state) */
};

class FmcSpi
{
public:
    FmcSpi(Wr_base *base, u32 fpga, u32 ba, uint8_t size);
    ~FmcSpi();
    
    void Write(uint16_t addr, uint8_t data);
    void WriteAdf5355(uint32_t reg_addr, uint32_t data);
    uint8_t Read(uint16_t addr);
    void Init();
    void WriteLpf(uint16_t data);
private:
   

    void Vpx_write_spi(u32 a, u32 d);
    u32 Vpx_read_spi(u32 a);
    
    int XSpi_Transfer(u8 *SendBufPtr, u8 *RecvBufPtr, unsigned int ByteCount);

    uint8_t *Execute(size_t num2read, double timeout_s = 10.0);

    u32 m_fpga = 0;
    Wr_base *wr_base = nullptr;
    XSpiFmc fmc_spi;
    size_t mCurrWriteBufInx = 0;
    uint8_t *mWriteBuf;
    uint8_t *mReadBuf;
};
