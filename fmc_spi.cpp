#include "fmc_spi.h"

FmcSpi::FmcSpi(Wr_base *base, u32 fpga, u32 ba, uint8_t size)
{
    fmc_spi.BaseAddr = ba;
	m_fpga = fpga;
	wr_base = base;
	mWriteBuf = new uint8_t[size];
	mReadBuf = new uint8_t[size];
}

FmcSpi::~FmcSpi()
{
	delete mWriteBuf;
	delete mReadBuf;
};

void FmcSpi::Init()
{
	uint32_t ControlReg = 0;
	FmcSpi_WriteReg(FmcSP_SRR_OFFSET, FmcSP_SRR_RESET_MASK);
	ControlReg |= FmcSP_CR_TXFIFO_RESET_MASK | FmcSP_CR_RXFIFO_RESET_MASK | FmcSP_CR_ENABLE_MASK | FmcSP_CR_MASTER_MODE_MASK | FmcSP_CR_MANUAL_SS_MASK| FmcSP_CR_TRANS_INHIBIT_MASK;
	Vpx_write_spi(fmc_spi.BaseAddr + FmcSP_CR_OFFSET, ControlReg);
	fmc_spi.SlaveSelectMask = ~(u32(1));
	FmcSpi_SetSlaveSelectReg(0xFFFFFFFF);
}

void FmcSpi::Vpx_write_spi(u32 a, u32 d)
{
	if (m_fpga == FPGA_TX)
		wr_base->tx_write(a, d);
	else if (m_fpga == FPGA_RX)
		wr_base->rx_write(a, d);
}

u32 FmcSpi::Vpx_read_spi(u32 a)
{
	if (m_fpga == FPGA_TX)
		return wr_base->tx_read(a);
	else if (m_fpga == FPGA_RX)
		return wr_base->rx_read(a);
	return -1;
}

uint8_t FmcSpi::Read(uint16_t addr)
{
	mCurrWriteBufInx = 0;
    mWriteBuf[mCurrWriteBufInx++] =  0x80 | (addr >> 8);
    mWriteBuf[mCurrWriteBufInx++] = addr & 0xFF;
	return *Execute(1);
}

void FmcSpi::Write(uint16_t addr, uint8_t data) 
{
	mCurrWriteBufInx = 3;
    mWriteBuf[0] = (uint8_t)((0 << 7) | (addr >> 8));
    mWriteBuf[1] = (uint8_t)(addr);
    mWriteBuf[2] = (data);
	Execute(0);
}

void FmcSpi::WriteAdf5355(uint32_t reg_addr, uint32_t data)
{
	mCurrWriteBufInx = 4;
	data = data | reg_addr;
    mWriteBuf[0] = data >> 24;;
    mWriteBuf[1] = data >> 16;
    mWriteBuf[2] = data >> 8;
    mWriteBuf[3] = data;
	Execute(0);
}

void FmcSpi::WriteLpf(uint16_t data)
{
	mCurrWriteBufInx = 0;
    mWriteBuf[mCurrWriteBufInx++] = data >> 8;;
    mWriteBuf[mCurrWriteBufInx++] = data & 0xFF;
	Execute(0);
}

uint8_t* FmcSpi::Execute(size_t num2read, double timeout_s)
{
	//const int status = XSpi_Transfer(mWriteBuf, num2read ? mReadBuf : NULL, mCurrWriteBufInx + num2read);
	XSpi_Transfer(mWriteBuf, num2read ? mReadBuf : NULL, mCurrWriteBufInx + num2read);

	const auto rez = num2read ? (mReadBuf + mCurrWriteBufInx) : NULL;
	return rez;
}

int FmcSpi::XSpi_Transfer(u8 *SendBufPtr, u8 *RecvBufPtr, unsigned int ByteCount)
{
	u32 ControlReg;
	u32 StatusReg;
	u32 Data = 0;
	const size_t ASPI_FIFO_DEPTH = 256;

	if (fmc_spi.IsBusy) return XST_DEVICE_BUSY;

	fmc_spi.IsBusy = TRUE;

	// Enable slave select
	FmcSpi_SetSlaveSelectReg(0xFFFFFFFE);

	while (ByteCount > 0)
	{
		// Wait for fifos to be empty, then we can write a block if we know the fifo depth
		const uint8_t desired_bits_set = FmcSP_SR_RX_EMPTY_MASK | FmcSP_SR_TX_EMPTY_MASK;
		while ((FmcSpi_GetStatusReg() & desired_bits_set) != desired_bits_set){};

		// Fill the tx fifo
		size_t filled_count = 0;
		while ((ByteCount > 0) && (filled_count < ASPI_FIFO_DEPTH))
		{
			Data = *SendBufPtr++;
			FmcSpi_WriteReg(FmcSP_DTR_OFFSET, Data);
			ByteCount--;
			filled_count++;
		}

		ControlReg = FmcSpi_GetControlReg();
		ControlReg &= ~FmcSP_CR_TRANS_INHIBIT_MASK;
		FmcSpi_SetControlReg(ControlReg);

		do
		{
			StatusReg = FmcSpi_IntrGetStatus();
		} while ((StatusReg & FmcSP_INTR_TX_EMPTY_MASK) == 0);

		FmcSpi_IntrClear(FmcSP_INTR_TX_EMPTY_MASK);

		ControlReg = FmcSpi_GetControlReg();
		FmcSpi_SetControlReg(ControlReg | FmcSP_CR_TRANS_INHIBIT_MASK);

		if (RecvBufPtr)
		{
			while (filled_count--)
			{
				Data = FmcSpi_ReadReg(FmcSP_DRR_OFFSET);
				if (RecvBufPtr)
				{
					*RecvBufPtr++ = (u8)Data;
				}
			}
		}
		else
		{
			ControlReg = FmcSpi_GetControlReg();
			ControlReg |= FmcSP_CR_RXFIFO_RESET_MASK;
			FmcSpi_SetControlReg(ControlReg);
		}
	}

	FmcSpi_SetSlaveSelectReg(0xFFFFFFFF);
	fmc_spi.IsBusy = FALSE;
	return 0;
}
