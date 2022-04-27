#include "flash_spi.h"
#include <stdio.h>

Cfg_flash_spi::Cfg_flash_spi(Wr_base *base, u32 fpga, u32 ba)
{
	spi_flash.BaseAddr = ba;
	m_fpga = fpga;
	wr_base = base;
	spi_flash.SlaveSelectReg = ~((u32)1);
	RegisterStatusCallback(MyStatusCallback);
}

Cfg_flash_spi::~Cfg_flash_spi(){};

void Cfg_flash_spi::Vpx_write_flash_spi(u32 a, u32 d)
{
	if (m_fpga == FPGA_TX)
		wr_base->tx_write(a, d);
	else if (m_fpga == FPGA_RX)
		wr_base->rx_write(a, d);
}

u32 Cfg_flash_spi::Vpx_read_flash_spi(u32 a)
{
	if (m_fpga == FPGA_TX)
		return wr_base->tx_read(a);
	else if (m_fpga == FPGA_RX)
		return wr_base->rx_read(a);
	return -1;
}

int Cfg_flash_spi::XSpi_Transfer(XSpi *InstancePtr, u8 *SendBufPtr,
								 u8 *RecvBufPtr, unsigned int ByteCount)
{
	u32 ControlReg;
	u32 StatusReg;
	u32 Data = 0;

	const size_t ASPI_FIFO_DEPTH = 256;

	/*
	 * Make sure there is not a transfer already in progress. No need to
	 * worry about a critical section here. Even if the Isr changes the bus
	 * flag just after we read it, a busy error is returned and the caller
	 * can retry when it gets the status handler callback indicating the
	 * transfer is done.
	 */
	if (InstancePtr->IsBusy)
	{
		return XST_DEVICE_BUSY;
	}
	/*
	 * Set the busy flag, which will be cleared when the transfer
	 * is completely done.
	 */
	InstancePtr->IsBusy = TRUE;

	/*
	 * Set up buffer pointers.
	 */
	InstancePtr->SendBufferPtr = SendBufPtr;
	InstancePtr->RecvBufferPtr = RecvBufPtr;

	InstancePtr->RequestedBytes = ByteCount;
	InstancePtr->RemainingBytes = ByteCount;

	// Enable slave select
	XSpi_SetSlaveSelectReg(InstancePtr, InstancePtr->SlaveSelectReg);
	while (InstancePtr->RemainingBytes > 0)
	{
		// Wait for fifos to be empty, then we can write a block if we know the fifo depth
		const uint8_t desired_bits_set = XSP_SR_RX_EMPTY_MASK | XSP_SR_TX_EMPTY_MASK;
		while ((XSpi_GetStatusReg(InstancePtr) & desired_bits_set) != desired_bits_set)
		{
			StatusReg = XSpi_GetStatusReg(InstancePtr);
		}

		// Fill the tx fifo
		size_t filled_count = 0;
		while ((InstancePtr->RemainingBytes > 0) && (filled_count < ASPI_FIFO_DEPTH))
		{
			Data = *InstancePtr->SendBufferPtr++;
			XSpi_WriteReg(InstancePtr->BaseAddr, XSP_DTR_OFFSET, Data);
			InstancePtr->RemainingBytes--;
			filled_count++;
		}

		// PrintStatusReg();

		/*
		 * Start the transfer by no longer inhibiting the transmitter and
		 * enabling the device. For a master, this will in fact start the
		 * transfer, but for a slave it only prepares the device for a transfer
		 * that must be initiated by a master.
		 */
		ControlReg = XSpi_GetControlReg(InstancePtr);
		ControlReg &= ~XSP_CR_TRANS_INHIBIT_MASK;
		XSpi_SetControlReg(InstancePtr, ControlReg);

		/*
		 * Wait for the transfer to be done by polling the
		 * Transmit empty status bit
		 */
		do
		{
			StatusReg = XSpi_IntrGetStatus(InstancePtr);
		} while ((StatusReg & XSP_INTR_TX_EMPTY_MASK) == 0);

		XSpi_IntrClear(InstancePtr, XSP_INTR_TX_EMPTY_MASK);

		/*
		 * A transmit has just completed. Process received data
		 * and check for more data to transmit. Always inhibit
		 * the transmitter while the transmit register/FIFO is
		 * being filled, or make sure it is stopped if we're
		 * done.
		 */

		ControlReg = XSpi_GetControlReg(InstancePtr);
		XSpi_SetControlReg(InstancePtr, ControlReg | XSP_CR_TRANS_INHIBIT_MASK);
		// Read out if user wants it
		if (InstancePtr->RecvBufferPtr)
		{
			while (filled_count--)
			{
				Data = XSpi_ReadReg(InstancePtr->BaseAddr, XSP_DRR_OFFSET);
				InstancePtr->Stats.BytesTransferred++;
				if (InstancePtr->RecvBufferPtr)
				{
					*InstancePtr->RecvBufferPtr++ = (u8)Data;
				}
			}
		}
		else
		{
			ControlReg = XSpi_GetControlReg(InstancePtr);
			ControlReg |= XSP_CR_RXFIFO_RESET_MASK;
			XSpi_SetControlReg(InstancePtr, ControlReg);
		}
	}
	/*
	 * Select the slave on the SPI bus when the transfer is
	 * complete, this is necessary for some SPI devices,
	 * such as serial EEPROMs work correctly as chip enable
	 * may be connected to slave select
	 */
	XSpi_SetSlaveSelectReg(InstancePtr, InstancePtr->SlaveSelectMask);
	InstancePtr->IsBusy = FALSE;

	return 0;
}
void Cfg_flash_spi::PrintStatusReg()
{
	u32 StatusReg;
	StatusReg = XSpi_GetStatusReg(&spi_flash);

	printf("Rx_Empty = %d  \n", StatusReg & 1);
	printf("Rx_Full = %d  \n", (StatusReg >> 1) & 1);
	printf("Tx_Empty = %d  \n", (StatusReg >> 2) & 1);
	printf("Tx_Full = %d  \n", (StatusReg >> 3) & 1);
	printf("MODF = %d  \n", (StatusReg >> 4) & 1);
	printf("Slave_Mode_Select = %d \n", (StatusReg >> 5) & 1);
	printf("CPOL_CPHA_Error = %d  \n", (StatusReg >> 6) & 1);
	printf("Slave Mode_Error = %d \n", (StatusReg >> 7) & 1);
	printf("MSB Error = %d  \n", (StatusReg >> 8) & 1);
	printf("Loopback Error = %d \n", (StatusReg >> 9) & 1);
	printf("Command Error= %d  \n", (StatusReg >> 10) & 1);
}

int Cfg_flash_spi::SpiFlashGetStatus(XSpi *SpiPtr)
{
	int Status;

	WriteBuffer[0] = COMMAND_STATUSREG_READ;

	Status = XSpi_Transfer(SpiPtr, WriteBuffer, ReadBuffer,
						   STATUS_READ_BYTES);
	if (Status != 0)
	{
		return 1;
	}
	return 0;
}
int Cfg_flash_spi::SpiFlashWaitForFlashReady(void)
{
	int Status;
	u8 StatusReg;

	while (1)
	{

		/*
		 * Get the Status Register. The status register content is
		 * stored at the second byte pointed by the ReadBuffer.
		 */
		Status = SpiFlashGetStatus(&spi_flash);
		if (Status != 0)
		{
			return 1;
		}

		/*
		 * Check if the flash is ready to accept the next command.
		 * If so break.
		 */
		StatusReg = ReadBuffer[1];
		if ((StatusReg & FLASH_SR_IS_READY_MASK) == 0)
		{
			break;
		}
	}

	return 0;
}
int Cfg_flash_spi::SpiFlashReadID()
{
	int Status;
	WriteBuffer[0] = 0x9F;

	Status = XSpi_Transfer(&spi_flash, WriteBuffer, ReadBuffer, 21);
	if (Status != 0)
	{
		g_lite->Error("Transfer spi failed, status = %d\n", Status);
		return 1;
	}
	g_lite->Info("Manufacturer ID: %02X\r\n", ReadBuffer[1]);
	g_lite->Info("Device ID: %02X %02X\r\n", ReadBuffer[2], ReadBuffer[3]);
	g_lite->Info("Unique ID: %02X %02X %02X\r\n", ReadBuffer[4], ReadBuffer[5],
				 ReadBuffer[6]);
	return 0;
}
int Cfg_flash_spi::SpiFlashWriteStatusReg()
{
	int Status;
	WriteBuffer[0] = 0x01;

	Status = XSpi_Transfer(&spi_flash, WriteBuffer, ReadBuffer, 1);
	if (Status != 0)
	{
		return -1;
	}
	g_lite->Info("WriteStatusReg: %02X\r\n", ReadBuffer[0]);

	return 0;
}
void Cfg_flash_spi::flash_init()
{
	uint32_t ControlReg = 0;
	XSpi_WriteReg(spi_flash.BaseAddr, XSP_SRR_OFFSET, XSP_SRR_RESET_MASK);
	ControlReg |= XSP_CR_TXFIFO_RESET_MASK | XSP_CR_RXFIFO_RESET_MASK | XSP_CR_ENABLE_MASK | XSP_CR_MASTER_MODE_MASK | XSP_CR_MANUAL_SS_MASK | XSP_CR_TRANS_INHIBIT_MASK;
	Vpx_write_flash_spi(spi_flash.BaseAddr + XSP_CR_OFFSET, ControlReg);
	XSpi_SetSlaveSelectReg(&spi_flash, u32(~1) );
}
int Cfg_flash_spi::SpiGetSR(XSpi *SpiPtr)
{
	u32 StatusReg;
	StatusReg = Vpx_read_flash_spi((SpiPtr)->BaseAddr + XSP_CR_OFFSET);
	g_lite->Info("Control Register = 0x%03X\n", StatusReg);
	StatusReg = Vpx_read_flash_spi((SpiPtr)->BaseAddr + XSP_SR_OFFSET);
	g_lite->Info("Status Register = 0x%03X\n", StatusReg);
	StatusReg = Vpx_read_flash_spi((SpiPtr)->BaseAddr + 0x20);
	g_lite->Info("IP interrupt status register = 0x%03X\n", StatusReg);
	StatusReg = Vpx_read_flash_spi((SpiPtr)->BaseAddr + 0x1C);
	g_lite->Info("Device global interrupt enable register= 0x%03X\n", StatusReg);
	return 0;
}

int Cfg_flash_spi::SpiFlashWriteEnable(XSpi *SpiPtr)
{
	int Status;

	Status = SpiFlashWaitForFlashReady();
	if (Status != 0)
	{
		return 1;
	}

	WriteBuffer[0] = COMMAND_WRITE_ENABLE;

	Status = XSpi_Transfer(SpiPtr, WriteBuffer, NULL,
						   WRITE_ENABLE_BYTES);
	if (Status != 0)
	{
		return 1;
	}
	return 0;
}

void Cfg_flash_spi::EraseRange(uint32_t addr, size_t len)
{
	if (addr & (FLASH_ENFORCED_SECTOR_BYTES - 1))
	{
		throw std::runtime_error("Flash address must be on an even page of " + std::to_string(FLASH_ENFORCED_SECTOR_BYTES) + " bytes");
	}

	std::lock_guard<decltype(mMutex)> lock(mMutex);

	// Clear error bits
	ClearStatusRegister();

	// For now, no range checking. Just erase
	size_t num_sectors_erased = 0;
	size_t erased_bytes = 0;
	while (erased_bytes < len)
	{
		WriteEnable();
		SectorErase(addr + erased_bytes);
		erased_bytes += FLASH_SECTOR_BYTES;
		SayStatus("Erased Sector", static_cast<double>(erased_bytes) / static_cast<double>(len));
		num_sectors_erased++;
	}

	SayStatus("Erased " + std::to_string(num_sectors_erased) + " sectors", 1.0);
	// For erase, because they take so long, we'll wait here to avoid skewing stats
	WaitForFlashNotBusy(FLASH_ERASE_TIMEOUT_S);

	// Check for errors
	const auto stat = GetStatusRegister();
	if (stat & SR_ANY_ERR_MASK)
	{
		SayStatus("Warning: Flash indicated an error while erasing");
	}
}

void Cfg_flash_spi::program_flash(const char *file)
{
	try
	{
		std::vector<uint8_t> data_to_write_flash1;
		std::vector<uint8_t> data_to_write_flash2;

		LoadBit(file, &data_to_write_flash1, &data_to_write_flash2);

		printf("\nread data from file HALF1\n");
		printf("Size buffer = %ld\n", data_to_write_flash1.size());

		for (size_t i = 0; i < data_to_write_flash1.size(); i++)
		{
			if (i % 16 == 0)
				printf("\n");
			printf("0x%02X   ", data_to_write_flash1.at(i));
		}
		printf("\nread data ENDDDD from file HALF1nn");

		program_half(data_to_write_flash1, 1, true);
		program_half(data_to_write_flash2, 2, true);
	}
	catch (const std::exception &ex)
	{
		printf("\nException occurred: %s\n", ex.what());
	}
}

void Cfg_flash_spi::Write(uint32_t flash_addr, const uint8_t *src, const size_t len)
{
	size_t numwritten = 0;

	std::lock_guard<decltype(mMutex)> lock(mMutex);

	// Clear error bits
	ClearStatusRegister();

	while (numwritten < len)
	{
		const auto flash_page_bytes = FLASH_PAGE_BYTES;
		const size_t real_count = std::min(flash_page_bytes, len - numwritten);

		WriteEnable();
		StartCommand(CMD_PAGEPROGRAM_WRITE);
		AddAddr(flash_addr);
		AddFromBuffer(src + numwritten, real_count);
		Execute(0);

		flash_addr += real_count;
		numwritten += real_count;
	}

	const auto stat = GetStatusRegister();

	if (stat & SR_ANY_ERR_MASK)
	{
		SayStatus("Warning: Flash indicated an error while writing");
	}
}

void Cfg_flash_spi::RegisterStatusCallback(const status_callback_t &cb)
{
	mCallBacks.push_back(cb);
}

void Cfg_flash_spi::LoadBit(const char *fname, std::vector<uint8_t> *v_flash1, std::vector<uint8_t> *v_flash2)
{
	FILE *fp = fopen(fname, "rb");
	if (fp)
	{
		uint8_t a, b, c;
		uint8_t temp[] = {1, 1, 1, 1};
		unsigned int offset = 0;
		unsigned int dummy = 0x82;
		// obtain file size
		fseek(fp, 0, SEEK_END);
		const auto fsize = ftell(fp);

		for (int i = 0; i < 512; i++)
		{
			fseek(fp, i, SEEK_SET);
			fread((char *)temp, 1, 4, fp);

			if (temp[0] == 0x0 && temp[1] == 0x0 && temp[2] == 0x0 && temp[3] == 0x12)
			{
				offset = i + 4;
				printf("found sync word !!! num = %d \n", offset);
				break;
			}
		}
		fseek(fp, 0, SEEK_SET);
		v_flash1->reserve(fsize);
		v_flash2->reserve(fsize);

		for (unsigned int i = 0; i < offset; i++)
		{
			fread(&a, 1, 1, fp);
			v_flash1->push_back(a);
			v_flash2->push_back(dummy);
		}
		int jump = (fsize - offset) / 2;
		for (int i = 0; i < jump; i++)
		{
			fread(&a, 1, 1, fp);
			fread(&b, 1, 1, fp);
			c = (((a << 4) & 0xF0) | (b & 0x0F));
			v_flash1->push_back(c);
			c = ((a & 0xF0) | ((b >> 4) & 0x0F));
			;
			v_flash2->push_back(c);
		}
		if (jump & 1)
		{
			fread(&a, 1, 1, fp);
			b = 0;
			c = ((a & 0x0F) | ((b & 0x0F) << 4));
			v_flash1->push_back(c);
			c = ((a >> 4) | (b & 0xF0));
			;
			v_flash2->push_back(c);
		}
		printf("offset = %d  ,  jump  = %d, ALL = %d", offset, jump, offset + jump + 1);
		fclose(fp);
	}
	else
	{
		printf("file not opened !!!!!!!!!");
	}
}

void Cfg_flash_spi::program_half(std::vector<uint8_t> &data_to_write, int num_flash, bool verify)
{
	// Mark erase/program start
	const auto elap_start = std::chrono::steady_clock::now();

	spi_flash.SlaveSelectReg = ~num_flash;

	XSpi_SetSlaveSelectReg(&spi_flash, spi_flash.SlaveSelectReg);

	// Erase
	printf("\nErasing...\n");
	auto start = std::chrono::steady_clock::now();
	EraseRange(0, data_to_write.size());
	std::chrono::duration<double> dt = std::chrono::steady_clock::now() - start;
	printf("\nErased in %.3fs...\n", dt.count());

	// Program
	printf("\nProgramming...\n");
	start = std::chrono::steady_clock::now();
	Write(0, data_to_write.data(), data_to_write.size());
	dt = std::chrono::steady_clock::now() - start;
	printf("\nProgrammed in %.3fs...\n", dt.count());

	// Report erase/program time
	dt = std::chrono::steady_clock::now() - elap_start;
	printf("\nErase/Program took %.3fs (%.0fKiB/s)\n", dt.count(), ((static_cast<double>(data_to_write.size()) / dt.count())) / 1024.0);

	// Verify
	if (verify)
	{
		printf("\nVerifying...\n");
		start = std::chrono::steady_clock::now();

		// Read section into vector
		std::vector<uint8_t> tmp(data_to_write.size());
		Read(0, tmp.data(), tmp.size());

		dt = std::chrono::steady_clock::now() - start;
		printf("\nRead in %.3fs...\n", dt.count());

		// Check and report
		if (tmp != data_to_write)
		{
			printf("\nVerify failed\n");
			for (size_t i = 0; i < tmp.size(); i++)
			{
				if (i % 16 == 0)
					printf("\n");
				printf("0x%02X   ", tmp.at(i));
			}
			printf("\n");
			// << early exit
		}
		else
		{
			printf("\nVerify OK\n");
		}
	}
}

void Cfg_flash_spi::Read(uint32_t flash_addr, uint8_t *dst, const size_t len)
{
	size_t numread = 0;
	while (numread < len)
	{
		StartCommand(CMD_RANDOM_READ);
		AddAddr(flash_addr);
		offset();
		//  Read up to one page
		const auto flash_page_bytes = FLASH_PAGE_BYTES; // Needed to compile C++11/C++14. Fixed in C++17. See https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
		const size_t real_count = std::min(flash_page_bytes, len - numread);
		const auto *rezbuf = Execute(real_count);

		memcpy(dst + numread, rezbuf, real_count);
		flash_addr += real_count;
		numread += real_count;
		// Report status
		// std::stringstream ss;
		// ss << "Read " << numread << " bytes";
		// SayStatus(ss.str(), static_cast<double>(numread) / static_cast<double>(len));
	}
}

void Cfg_flash_spi::ClearStatusRegister(void)
{
	StartCommand(CMD_STATUSREG_CLEAR);
	Execute(0);
}

void Cfg_flash_spi::StartCommand(uint8_t cmd)
{
	mCurrWriteBufInx = 0;
	mWriteBuf[mCurrWriteBufInx++] = cmd;
}

uint8_t* Cfg_flash_spi::Execute(size_t num2read, double timeout_s)
{
	if (0 == mCurrWriteBufInx)
	{
		throw std::runtime_error("No command specified");
	}

	if (((mCurrWriteBufInx - 1) + num2read) >= TOTAL_BUFFER_SIZE + 4)
	{
		throw std::runtime_error("Attempt to read/write too many bytes");
	}

	// Make sure flash isn't busy
	WaitForFlashNotBusy(timeout_s);

	// Execute
	const int status = XSpi_Transfer(&spi_flash, mWriteBuf, num2read ? mReadBuf : NULL, mCurrWriteBufInx + num2read); //num2read ? num2read + 4 : num2read
	if (status != 0)
	{
		throw std::runtime_error("SPI transaction failed"
								 " code " +
								 std::to_string((int)status) + " cmd " + std::to_string((int)mWriteBuf[0]));
	}

	// Get the return value before zeroing index
	const auto rez = num2read ? (mReadBuf + mCurrWriteBufInx) : NULL;
	// mCurrWriteBufInx = 0;

	// Return a pointer to the first byte read (if any)
	return rez;
}

void Cfg_flash_spi::WriteEnable(void)
{
	StartCommand(CMD_WRITE_ENABLE);
	Execute(0);
}

void Cfg_flash_spi::SectorErase(uint32_t addr) // was u32
{
	StartCommand(CMD_SECTOR_ERASE);
	AddAddr(addr);
	Execute(0);
}

void Cfg_flash_spi::AddAddr(uint32_t addr)
{
	mWriteBuf[mCurrWriteBufInx++] = (uint8_t)(addr >> 24);
	mWriteBuf[mCurrWriteBufInx++] = (uint8_t)(addr >> 16);
	mWriteBuf[mCurrWriteBufInx++] = (uint8_t)(addr >> 8);
	mWriteBuf[mCurrWriteBufInx++] = (uint8_t)(addr);
}
void Cfg_flash_spi::offset()
{
	mCurrWriteBufInx += 4;

}

void Cfg_flash_spi::WaitForFlashNotBusy(double wait_s)
{
	// Get the current time
	const auto start = std::chrono::steady_clock::now();
	while (1)
	{
		if ((GetStatusRegister() & SR_IS_READY_MASK) == 0)
		{
			break;
		}

		// Get the elapsed time in s
		const std::chrono::duration<double> dt = std::chrono::steady_clock::now() - start;
		if (dt.count() > wait_s)
		{
			throw std::runtime_error("Timeout waiting for flash ready");
		}
	}
}

uint8_t Cfg_flash_spi::GetStatusRegister(void)
{
	// Special case- don't call execute
	// Don't need to wait for flash to be un-busy before reading status register
	uint8_t sendbuf[16];
	uint8_t recvbuf[16];

	sendbuf[0] = CMD_STATUSREG_READ;

	// Execute
	const int status = XSpi_Transfer(&spi_flash, sendbuf, recvbuf, 2);
	if (status != 0)
	{
		throw std::runtime_error("SPI transaction failed getting status register code " + std::to_string((int)status));
	}

	return recvbuf[1];
}

void Cfg_flash_spi::SayStatus(const std::string &msg, double pcnt_cplt)
{
	// Build a status structure
	pgm_status_s stat;
	stat.msg = msg;
	stat.pcnt_cmplt = pcnt_cplt;

	// Call all callbacks.
	for (auto &cb : mCallBacks)
	{
		cb(stat);
	}
}

void Cfg_flash_spi::AddFromBuffer(const uint8_t *buf, size_t len)
{
	if ((len + mCurrWriteBufInx) > TOTAL_BUFFER_SIZE)
	{
		throw std::runtime_error("Attempt to write too many bytes");
	}

	memcpy(mWriteBuf + mCurrWriteBufInx, buf, len);
	mCurrWriteBufInx += len;
}