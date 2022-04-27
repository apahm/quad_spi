#pragma once
#include "wr_base.h"
#include "CONFIG.h"
#include <cstdlib>
#include <sys/stat.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <sstream>
#include <vector>
#define FALSE 0L
#define TRUE 1L
#define XST_DEVICE_IS_STOPPED 6L
#define XST_DEVICE_BUSY 21L	  /*!< Device is busy */
#define XST_SPI_NO_SLAVE 1155 /*!< no slave has been selected yet */
#define XIL_COMPONENT_IS_READY 0x11111111U
#define XIL_COMPONENT_IS_STARTED 0x22222222U
/**
 * SPI Software Reset Register (SRR) mask.
 */
#define XSP_SRR_RESET_MASK		0x0000000A

#define XSP_DGIER_OFFSET 0x1C /**< Global Intr Enable Reg */
#define XSP_IISR_OFFSET 0x20  /**< Interrupt status Reg */
#define XSP_IIER_OFFSET 0x28  /**< Interrupt Enable Reg */
#define XSP_SRR_OFFSET 0x40	  /**< Software Reset register */
#define XSP_CR_OFFSET 0x60	  /**< Control register */
#define XSP_SR_OFFSET 0x64	  /**< Status Register */
#define XSP_DTR_OFFSET 0x68	  /**< Data transmit */
#define XSP_DRR_OFFSET 0x6C	  /**< Data receive */
#define XSP_SSR_OFFSET 0x70	  /**< 32-bit slave select */
#define XSP_TFO_OFFSET 0x74	  /**< Tx FIFO occupancy */
#define XSP_RFO_OFFSET 0x78	  /**< Rx FIFO occupancy */

#define XSP_CR_LOOPBACK_MASK 0x00000001		 /**< Local loopback mode */
#define XSP_CR_ENABLE_MASK 0x00000002		 /**< System enable */
#define XSP_CR_MASTER_MODE_MASK 0x00000004	 /**< Enable master mode */
#define XSP_CR_CLK_POLARITY_MASK 0x00000008	 /**< Clock polarity high or low */
#define XSP_CR_CLK_PHASE_MASK 0x00000010	 /**< Clock phase 0 or 1 */
#define XSP_CR_TXFIFO_RESET_MASK 0x00000020	 /**< Reset transmit FIFO */
#define XSP_CR_RXFIFO_RESET_MASK 0x00000040	 /**< Reset receive FIFO */
#define XSP_CR_MANUAL_SS_MASK 0x00000080	 /**< Manual slave selectassert */
#define XSP_CR_TRANS_INHIBIT_MASK 0x00000100 /**< Master transactioninhibit */

/** @name Status Register (SR) masks
 *
 * @{
 */
#define XSP_SR_RX_EMPTY_MASK 0x00000001	  /**< Receive Reg/FIFO is empty */
#define XSP_SR_RX_FULL_MASK 0x00000002	  /**< Receive Reg/FIFO is full */
#define XSP_SR_TX_EMPTY_MASK 0x00000004	  /**< Transmit Reg/FIFO is empty */
#define XSP_SR_TX_FULL_MASK 0x00000008	  /**< Transmit Reg/FIFO is full */
#define XSP_SR_MODE_FAULT_MASK 0x00000010 /**< Mode fault error */
#define XSP_SR_SLAVE_MODE_MASK 0x00000020 /**< Slave mode select */

#define XSP_GINTR_ENABLE_MASK 0x80000000 /**< Global interrupt enable */
/** @name SPI Device Interrupt Status/Enable Registers
 *
 * <b> Interrupt Status Register (IPISR) </b>
 *
 * This register holds the interrupt status flags for the Spi device.
 *
 * <b> Interrupt Enable Register (IPIER) </b>
 *
 * This register is used to enable interrupt sources for the Spi device.
 * Writing a '1' to a bit in this register enables the corresponding Interrupt.
 * Writing a '0' to a bit in this register disables the corresponding Interrupt.
 *
 * ISR/IER registers have the same bit definitions and are only defined once.
 * @{
 */
#define XSP_INTR_MODE_FAULT_MASK 0x00000001		  /**< Mode fault error */
#define XSP_INTR_SLAVE_MODE_FAULT_MASK 0x00000002 /**< Selected as slave while disabled */
#define XSP_INTR_TX_EMPTY_MASK 0x00000004		  /**< DTR/TxFIFO is empty */
#define XSP_INTR_TX_UNDERRUN_MASK 0x00000008	  /**< DTR/TxFIFO underrun */
#define XSP_INTR_RX_FULL_MASK 0x00000010		  /**< DRR/RxFIFO is full */
#define XSP_INTR_RX_OVERRUN_MASK 0x00000020		  /**< DRR/RxFIFO overrun */
#define XSP_INTR_TX_HALF_EMPTY_MASK 0x00000040	  /**< TxFIFO is half empty */
#define XSP_INTR_SLAVE_MODE_MASK 0x00000080		  /**< Slave select mode */
#define XSP_INTR_RX_NOT_EMPTY_MASK 0x00000100	  /**< RxFIFO not empty */

#define XSpi_IsIntrGlobalEnabled(InstancePtr) (Vpx_read_flash_spi((InstancePtr)->BaseAddr + XSP_DGIER_OFFSET) == XSP_GINTR_ENABLE_MASK)
#define XSpi_IntrGlobalDisable(InstancePtr) Vpx_write_flash_spi((InstancePtr)->BaseAddr + XSP_DGIER_OFFSET, 0)
#define XSpi_GetControlReg(InstancePtr) Vpx_read_flash_spi((InstancePtr)->BaseAddr + XSP_CR_OFFSET)
#define XSpi_GetStatusReg(InstancePtr) Vpx_read_flash_spi((InstancePtr)->BaseAddr + XSP_SR_OFFSET)
#define XSpi_WriteReg(BaseAddress, RegOffset, RegisterValue) Vpx_write_flash_spi((BaseAddress) + (RegOffset), (RegisterValue))
#define XSpi_SetSlaveSelectReg(InstancePtr, Mask) Vpx_write_flash_spi((InstancePtr)->BaseAddr + XSP_SSR_OFFSET, (Mask))
#define XSpi_SetControlReg(InstancePtr, Mask) Vpx_write_flash_spi((InstancePtr)->BaseAddr + XSP_CR_OFFSET, (Mask))

#define XSpi_Reset(InstancePtr, Mask) Vpx_write_flash_spi((InstancePtr)->BaseAddr + XSP_SRR_OFFSET, (Mask))

#define XSpi_IntrGetStatus(InstancePtr) Vpx_read_flash_spi((InstancePtr)->BaseAddr + XSP_IISR_OFFSET)
#define XSpi_IntrClear(InstancePtr, ClearMask) Vpx_write_flash_spi((InstancePtr)->BaseAddr + XSP_IISR_OFFSET, XSpi_IntrGetStatus(InstancePtr) | (ClearMask))
#define XSpi_ReadReg(BaseAddress, RegOffset) Vpx_read_flash_spi(BaseAddress + RegOffset)

#define MT25Q_SECTOR_SIZE 65536
#define MT25Q_READ_WRITE_EXTRA_BYTES_4BYTEADDR 5 // Read/Write extra bytes
#define MT25Q_READ_WRITE_EXTRA_BYTES 4			 // Read/Write extra bytes
#define MT25Q_WRITE_ENABLE_BYTES 1				 // Write Enable bytes
#define MT25Q_SECTOR_ERASE_BYTES 4				 // Sector erase extra bytes
#define MT25Q_SECTOR_ERASE_BYTES_4BYTEADDR 5	 // Sector erase extra bytes
#define MT25Q_BULK_ERASE_BYTES 1				 // Bulk erase extra bytes
#define MT25Q_STATUS_READ_BYTES 2				 // Status read bytes count
#define MT25Q_STATUS_WRITE_BYTES 2				 // Status write bytes count
#define MT25Q_READ_ID_BYTES 21					 // Read ID bytes count
#define MT25Q_COMMAND_READ_ID 0x9F				 // Read ID command
#define MT25Q_COMMAND_PAGE_PROGRAM 0x02			 // Page Program command
#define MT25Q_COMMAND_QUAD_WRITE 0x32			 // Quad Input Fast Program
#define MT25Q_COMMAND_RANDOM_READ 0x03			 // Random read command
#define MT25Q_COMMAND_DUAL_READ 0x3B			 // Dual Output Fast Read
#define MT25Q_COMMAND_DUAL_IO_READ 0xBB			 // Dual IO Fast Read
#define MT25Q_COMMAND_QUAD_READ 0x6B			 // Quad Output Fast Read
#define MT25Q_COMMAND_QUAD_IO_READ 0xEB			 // Quad IO Fast Read
#define MT25Q_COMMAND_WRITE_ENABLE 0x06			 // Write Enable command
#define MT25Q_COMMAND_SECTOR_ERASE 0xD8			 // Sector Erase command
#define MT25Q_COMMAND_BULK_ERASE 0xC7			 // Bulk Erase command
#define MT25Q_COMMAND_STATUSREG_READ 0x05		 // Status read command
#define MT25Q_COMMAND_EXTADDRSREG_READ 0xC8		 // Extended address register read command
#define MT25Q_COMMAND_EXTADDRSREG_WRITE 0xC5	 // Extended address register write command
#define MT25Q_COMMAND_4BYTE_QUAD_READ 0x6C		 // 4-byte Quad Output Fast Read
#define MT25Q_COMMAND_ENABLE_4BYTE 0xB7			 // Enter 4-byte address mode
#define MT25Q_COMMAND_DISABLE_4BYTE 0xE9		 // Exit 4-byte address mode

#define COMMAND_PAGE_PROGRAM 0x02	/* Page Program command */
#define COMMAND_QUAD_WRITE 0x32		/* Quad Input Fast Program */
#define COMMAND_RANDOM_READ 0x13	/* Random read command */
#define COMMAND_DUAL_READ 0x3B		/* Dual Output Fast Read */
#define COMMAND_DUAL_IO_READ 0xBB	/* Dual IO Fast Read */
#define COMMAND_QUAD_READ 0x6B		/* Quad Output Fast Read */
#define COMMAND_QUAD_IO_READ 0xEB	/* Quad IO Fast Read */
#define COMMAND_WRITE_ENABLE 0x06	/* Write Enable command */
#define COMMAND_SECTOR_ERASE 0xD8	/* Sector Erase command */
#define COMMAND_BULK_ERASE 0xC7		/* Bulk Erase command */
#define COMMAND_STATUSREG_READ 0x05 /* Status read command */

#define DUAL_READ_DUMMY_BYTES 2
#define QUAD_READ_DUMMY_BYTES 4

#define DUAL_IO_READ_DUMMY_BYTES 2
#define QUAD_IO_READ_DUMMY_BYTES 5
/*
 * Flash not busy mask in the status register of the flash device.
 */
#define FLASH_SR_IS_READY_MASK 0x01 /* Ready mask */

/*
 * Number of bytes per page in the flash device.
 */
#define MT25Q_PAGE_SIZE 256
/*
 * Address of the page to perform Erase, Write and Read operations.
 */
#define FLASH_TEST_ADDRESS 0x00

/**
 * This definitions specify the EXTRA bytes in each of the command
 * transactions. This count includes Command byte, address bytes and any
 * don't care bytes needed.
 */
#define READ_WRITE_EXTRA_BYTES 4 /* Read/Write extra bytes */
#define WRITE_ENABLE_BYTES 1	 /* Write Enable bytes */
#define SECTOR_ERASE_BYTES 4	 /* Sector erase extra bytes */
#define BULK_ERASE_BYTES 1		 /* Bulk erase extra bytes */
#define STATUS_READ_BYTES 2		 /* Status read bytes count */
#define STATUS_WRITE_BYTES 2	 /* Status write bytes count */

struct XSpi_Stats
{
	u32 ModeFaults = 0;		  /**< Number of mode fault errors */
	u32 XmitUnderruns = 0;	  /**< Number of transmit underruns */
	u32 RecvOverruns = 0;	  /**< Number of receive overruns */
	u32 SlaveModeFaults = 0;  /**< Num of selects as slave while disabled */
	u32 BytesTransferred = 0; /**< Number of bytes transferred */
	u32 NumInterrupts = 0;	  /**< Number of transmit/receive interrupts */
};

struct XSpi
{
	XSpi_Stats Stats;					  /**< Statistics */
	u32 BaseAddr;						  /**< Base address of device (IPIF) */
	int IsReady = XIL_COMPONENT_IS_READY; /**< Device is initialized and ready */
	int IsStarted = 0;					  /**< Device has been started */
	int HasFifos = 1;					  /**< Device is configured with FIFOs or not */
	u32 SlaveOnly = 0;					  /**< Device is configured to be slave only */
	u8 NumSlaveBits = 2;				  /**< Number of slave selects for this device */
	u8 DataWidth = 8;					  /**< Data Transfer Width 8 or 16 or 32 */
	u8 SpiMode = 2;						  /**< Standard/Dual/Quad mode */
	u32 SlaveSelectMask = (1 << 2) - 1;	  /**< Mask that matches the number of SS bits */
	u32 SlaveSelectReg = (1 << 2) - 1;	  /**< Slave select register */

	u8 *SendBufferPtr = NULL;		 /**< Buffer to send  */
	u8 *RecvBufferPtr = NULL;		 /**< Buffer to receive */
	unsigned int RequestedBytes = 0; /**< Total bytes to transfer (state) */
	unsigned int RemainingBytes = 0; /**< Bytes left to transfer (state) */
	int IsBusy = 0;					 /**< A transfer is in progress (state) */

	void *StatusRef; /**< Callback reference for status handler */
};

struct pgm_status_s
{
	std::string msg;
	double pcnt_cmplt = std::numeric_limits<double>::quiet_NaN();
};

typedef std::function<void(const pgm_status_s &stat)> status_callback_t;

class Cfg_flash_spi
{
public:
	Cfg_flash_spi(Wr_base *base, u32 fpga, u32 ba);
	~Cfg_flash_spi();
	int XSpi_Transfer(XSpi *InstancePtr, u8 *SendBufPtr,
					  u8 *RecvBufPtr, unsigned int ByteCount);
	int SpiFlashReadID();
	int SpiFlashWriteStatusReg();
	int SpiFlashWriteEnable(XSpi *SpiPtr);
	int SpiFlashWaitForFlashReady(void);
	int SpiFlashGetStatus(XSpi *SpiPtr);
	int Cfg_rand_read(u32 Addr);
	int Cfg_read(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 ReadCmd);
	int SpiFlashWrite(XSpi *SpiPtr, u32 Addr, u32 ByteCount, u8 WriteCmd);
	int SpiGetSR(XSpi *SpiPtr);
	int Cfg_rand_read2file();
	int Cfg_status_read();
	int SpiFlashSectorErase(XSpi *SpiPtr, u32 Addr);
	int flash_program(const char *nfile);

	void EraseRange(uint32_t addr, size_t len);
	void flash_init();
	void PrintStatusReg();

	void read_flash(uint32_t flash_addr, size_t len)
	{
		std::vector<uint8_t> buff(len, 0x99);
		Read(flash_addr, buff.data(), len);
		for (size_t i = 0; i < len; i++)
		{
			if (i % 16 == 0)
				printf("\n");
			printf("0x%02X   ", buff.at(i));
		}
	}

	void flash_reset()
	{
		XSpi_Reset(&spi_flash, 0x0000000A);
	}
	void Write(uint32_t flash_addr, const uint8_t *src, const size_t len);

	void program_flash(const char *file);

	XSpi spi_flash;

private:
	u32 m_fpga = 0;
	Wr_base *wr_base = nullptr;
	void Vpx_write_flash_spi(u32 a, u32 d);
	u32 Vpx_read_flash_spi(u32 a);
	u8 ReadBuffer[264];
	u8 WriteBuffer[260];

	static void MyStatusCallback(const pgm_status_s &stat)
	{
		printf("\r(%.1f%%):%-60s", stat.pcnt_cmplt * 100.0, stat.msg.c_str());
	}
	void RegisterStatusCallback(const status_callback_t &cb);

	void LoadBit(const char *fname, std::vector<uint8_t> *v_flash1, std::vector<uint8_t> *v_flash2);
	void program_half(std::vector<uint8_t> &data_to_write, int num_flash, bool verify);

	void Read(uint32_t flash_addr, uint8_t *dst, const size_t len);
	void ClearStatusRegister(void);
	void StartCommand(uint8_t cmd);
	uint8_t *Execute(size_t num2read, double timeout_s = FLASH_DEFAULT_CMD_TIMEOUT_S);
	void WriteEnable(void);
	void SectorErase(uint32_t addr);
	void AddAddr(uint32_t addr);
	void offset();
	void WaitForFlashNotBusy(double wait_s);
	uint8_t GetStatusRegister(void);
	void SayStatus(const std::string &msg, double pcnt_cplt = std::numeric_limits<double>::quiet_NaN());
	void AddFromBuffer(const uint8_t *buf, size_t len);

	static constexpr uint8_t SR_E_ERR_MASK = 0x20;								// D5 is 1 if erase error
	static constexpr uint8_t SR_P_ERR_MASK = 0x10;								// D4 is 1 if program error
	static constexpr uint8_t SR_ANY_ERR_MASK = (SR_P_ERR_MASK | SR_E_ERR_MASK); // Any error
	static constexpr uint8_t CMD_STATUSREG_READ = 0x05;							// or 0x05 --------
	static constexpr uint8_t SR_IS_READY_MASK = 0x01;							// D0 is 1 when busy
	static constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
	static constexpr size_t FLASH_ENFORCED_SECTOR_BYTES = 256 * 1024;
	static constexpr uint8_t CMD_STATUSREG_CLEAR = 0x50;

	static constexpr double FLASH_ERASE_TIMEOUT_S = 10.0;
	static constexpr double FLASH_DEFAULT_CMD_TIMEOUT_S = 10.0;

	static constexpr size_t FLASH_MAX_CMD_BYTES = 5;
	static constexpr size_t FLASH_PAGE_BYTES = 256;
	static constexpr uint8_t CMD_PAGEPROGRAM_WRITE = 0x34; ///             12                    //

	static constexpr uint8_t QUAD_PAGEPROGRAM = 0x34;
	static constexpr uint8_t QUAD_READ = 0x13; 
	static constexpr uint8_t CMD_SECTOR_ERASE = 0x5C;
	static constexpr size_t FLASH_SECTOR_BYTES = 32 * 1024;

	static constexpr size_t TOTAL_BUFFER_SIZE = FLASH_MAX_CMD_BYTES + FLASH_PAGE_BYTES;
	static constexpr uint8_t CMD_RANDOM_READ = 0x6C;  //         13                      // 6C

	size_t mCurrWriteBufInx = 0;
	uint8_t mWriteBuf[TOTAL_BUFFER_SIZE + 4];
	uint8_t mReadBuf[TOTAL_BUFFER_SIZE + 4];
	// Mutex to make the API thread safe. Must be locked by any public functions
	std::recursive_mutex mMutex;
	// Registered callback functions
	std::vector<status_callback_t> mCallBacks;
};
