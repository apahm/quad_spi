`timescale 1ns / 1ps
module buf400tx (
    input [3:0] DAC1_TXD_p,
    input [3:0] DAC1_TXD_n,
    //
    input [3:0] DAC2_TXD_p,
    input [3:0] DAC2_TXD_n,
    // AD9152 DAC#1 control
    input  DAC1_RFOUT_EN,
    input  DAC1_TXEN,
    input  DAC1_RSTn,
    output DAC1_IRQn,
    output DAC1_PROTOUT,
    /*
    input  DAC1_CSn,
    input  DAC1_CLK,
    output DAC1_SDO, // reserved for 4 wire operation       
    output DAC1_SDIO_i,
    input  DAC1_SDIO_o,
    input  DAC1_SDIO_t,
    */
    // AD9152 DAC#2 control
    input  DAC2_RFOUT_EN,
    input  DAC2_TXEN,
    input  DAC2_RSTn,
    output DAC2_IRQn,
    output DAC2_PROTOUT,

    /*
    input  DAC2_CSn,
    input  DAC2_CLK,
    output DAC2_SDO, //reserved for 4 wire operation    
    output DAC2_SDIO_i,
    input  DAC2_SDIO_o,
    input  DAC2_SDIO_t,
    */
    
    // AD5355#1 control
    /*
    input  SYNT1_CLK,
    input  SYNT1_SDI,
    input  SYNT1_LE,
    */
    input  SYNT1_CE,
    input  SYNT1_PDRFn,
    output SYNT1_MUXOUT,
    output SYNT1_MUXOUT_FD,
    input  SYNT1_MUXOUT_FD_CLR,
    // AD5355#2 control
    /*
    input  SYNT2_CLK,
    input  SYNT2_SDI,
    input  SYNT2_LE,
    */
    input  SYNT2_CE,
    input  SYNT2_PDRFn,
    output SYNT2_MUXOUT,
    output SYNT2_MUXOUT_FD,
    input  SYNT2_MUXOUT_FD_CLR,
    //
    output DAC1_SYNCOUT,
    output DAC2_SYNCOUT,
    output SYSREF_OUT,
    //
    output FPGA_GBTCLK0,        //fmc_GBTCLK0_M2C_p FPGA_GBT_O_CLK
    output FPGA_GBTCLK0_ODIV2,  //fmc_GBTCLK0_M2C_ODIV2_p FPGA_GBT_ODIV2_O_CLK
    output FPGA_REFCLK,         //CLK0_M2C_P    FPGA_REF_O_CLK
    output EXTSYNC_IN,          //CLK1_M2C_P    FMC_REF_O_CLK
    input  REFCLK2FMC,          //CLK2_BIDIR_P  REF2FMC_I_CLK
    //
    output LMK_STATLD1,
    output LMK_STATLD2,
    input  LMK_CLKSEL0,
    input  LMK_CLKSEL1,
    //
    /*
    input  LMK_SDIO_o,
    output LMK_SDIO_i,
    input  LMK_SDIO_t,
    input  LMK_CS,
    input  LMK_SCK,
    */
    //
    input  LMK_SYNC,
    input  LMK_RESET,
    //
    input  REFCLKOUT_EN,
    input  QZ_ENn,
    output MON_ALERT,
    // FMC //
    input  fmc_gbtclk0_m2c_p,   // FPGA_GBT_I_CLK опорный тактовый сигнал приёмопередатчиков MGT JESD204B FPGA 250 МГц DCLKout0/0
    input  fmc_gbtclk0_m2c_n,   // FPGA_GBT_I_CLK опорный тактовый сигнал приёмопередатчиков MGT JESD204B FPGA 250 МГц DCLKout0/0
    input  fmc_clk0_m2c_p,      // FPGA_REF_I_CLK сигнал глобального тактирования FPGA несущего модуля (ядро JESD204B) 250 МГц SDCLKout3/3
    input  fmc_clk0_m2c_n,      // FPGA_REF_I_CLK сигнал глобального тактирования FPGA несущего модуля (ядро JESD204B) 250 МГц SDCLKout3/3
    input  fmc_clk1_m2c_p,      // FMC_REF_I_CLK входной опорный тактовый сигнал LVDS для синтезатора LMK04828 (CLKin2)
    input  fmc_clk1_m2c_n,      // FMC_REF_I_CLK входной опорный тактовый сигнал LVDS для синтезатора LMK04828 (CLKin2)
    output fmc_clk2_bidir_p,    // дифференциальный REFCLK2FMC
    output fmc_clk2_bidir_n,    // дифференциальный REFCLK2FMC
    // 8 линий данных на ЦАП
    output fmc_dp00_c2m_p,
    output fmc_dp00_c2m_n,
    output fmc_dp01_c2m_p,
    output fmc_dp01_c2m_n,
    output fmc_dp02_c2m_p,
    output fmc_dp02_c2m_n,
    output fmc_dp03_c2m_p,
    output fmc_dp03_c2m_n,
    output fmc_dp04_c2m_p,
    output fmc_dp04_c2m_n,
    output fmc_dp05_c2m_p,
    output fmc_dp05_c2m_n,
    output fmc_dp06_c2m_p,
    output fmc_dp06_c2m_n,
    output fmc_dp07_c2m_p,
    output fmc_dp07_c2m_n,

    input  fmc_la00_p, // FPGA_SYSREF_N -сигнал синхронизации для ядра JESD204B FPGA несущего модуля
    input  fmc_la00_n, // FPGA_SYSREF_P -сигнал синхронизации для ядра JESD204B FPGA несущего модуля
    input  fmc_la01_p, // DAC1_IRQn     -сигнал запроса прерывания ЦАП 1
    output fmc_la01_n, // 1DAC_RESET#   -сигнал сброса ЦАП1
    output fmc_la04_p, // 1DAC_CS#      -сигнал разрешения SPI ЦАП 1
    output fmc_la05_p, // LMK_CS        -сигнал разрешения SPI LMK04828
    inout  fmc_la05_n, // LMK_SDIO      -сигнал данных SPI LMK04828
    output fmc_la06_p, // 1DAC_TXEN     -сигнал разрешения выхода ЦАП 1
    output fmc_la06_n, // LMK_SCK       -тактовый сигнал SPI LMK04828
    output fmc_la07_p, // 1SYNT_PDRF    -сигнал отключения выхода синтезатора канала 1
    input  fmc_la07_n, // 1DAC_SDO      -сигнал выходных данных SPI ЦАП 1
    output fmc_la08_p, // 1RFOUT_EN     -сигнал разрешения выхода канала 1
    input  fmc_la08_n, // DAC1_PROTOUT  -сигнал PROTECT OUT ЦАП 1
    input  fmc_la09_p, // 1SYNT_MOUT    -сигнал MUXOUT синтезатора канала 1
    output fmc_la09_n, // 1SYNT_CE      -сигнал разрешения работы синтезатора канала 1
    input  fmc_la10_n, // MON_ALERT     -инвертированный сигнал прерывания АЦП мониторинга
    output fmc_la11_p, // 1SYNT_LE      -загрузка данных SPI синтезатора канала 1
    output fmc_la11_n, // 1DAC_SCLK     -тактовый сигнал SPI ЦАП 1
    inout  fmc_la12_p, // 1DAC_SDIO     -двунаправленный сигнал данных SPI ЦАП 1
    output fmc_la12_n, // QZ_ENn        -сигнал разрешения выхода опорного генератора 25 МГц
    output fmc_la13_p, // SYNT1_DATA    -сигнал данных SPI синтезатора канала 1
    output fmc_la13_n, // SYNT1_CLK     -тактовый сигнал SPI синтезатора канала 1
    output fmc_la14_p, // 2SYNT_PDRF    -сигнал отключения выхода синтезатора канала 2
    output fmc_la14_n, // SYNT2_CE      -сигнал разрешения работы синтезатора канала 2
    input  fmc_la15_p, // LMK_STATLD1   -сигнал статуса синтезатора LMK04828 LD1
    input  fmc_la15_n, // LMK_STATLD2   -сигнал статуса синтезатора LMK04828 LD2
    output fmc_la16_p, // REFCLKOUT_EN  -сигнал разрешения выхода внешнего опорного тактирования
    input  fmc_la17_p, // 1DAC_SYNCOUT_P-выходной сигнал синхронизации ЦАП 1
    input  fmc_la17_n, // 1DAC_SYNCOUT_N-выходной сигнал синхронизации ЦАП 1
    input  fmc_la18_p, // DAC2_SYNCOUT_p-выходной сигнал синхронизации ЦАП 2
    input  fmc_la18_n, // DAC2_SYNCOUT_n-выходной сигнал синхронизации ЦАП 2
    output fmc_la19_p, // LMK_SYNC      -входной сигнал LMK04828 запроса синхронизации интерфейса JESD204B
    output fmc_la19_n, // LMK_CLKSEL0   -сигнал CLKin SEL0 LMK04828
    input  fmc_la20_p, // 2SYNT_MOUT    -сигнал MUXOUT синтезатора канала 2
    output fmc_la20_n, // LMK_CLKSEL1   -сигнал CLKin SEL1 LMK04828
    output fmc_la21_p, // 2SYNT_CLK     -тактовый сигнал SPI синтезатора канала 2
    output fmc_la21_n, // 2SYNT_DATA    -сигнал данных SPI синтезатора канала 2
    output fmc_la22_p, // LMK_RESET     -сигнал сброса синтезатора LMK04828
    output fmc_la22_n, // SYNT2_LE      -загрузка данных SPI синтезатора канала 2
    output fmc_la24_p, // 2DAC_TXEN     -сигнал разрешения выхода ЦАП 2
    output fmc_la24_n, // 2DAC_RESET    -сигнал сброса ЦАП 2
    input  fmc_la29_p, // DAC2_IRQn     -сигнал запроса прерывания ЦАП 2
    output fmc_la30_p, // 2DAC_SCLK     -тактовый сигнал SPI ЦАП 2
    inout  fmc_la30_n, // 2DAC_SDIO     -двунаправленный сигнал данных SPI ЦАП 2
    input  fmc_la32_p, // 2DAC_PROTOUT  -сигнал PROTECT OUT ЦАП 2
    output fmc_la32_n, // 2RFOUT_EN     -сигнал разрешения выхода канала 2
    input  fmc_la33_p, // 2DAC_SDO      -сигнал выходных данных SPI ЦАП 2
    output fmc_la33_n,  // 2DAC_CS#      -сигнал разрешения SPI ЦАП 2

    input   wire            s_sck,
    input   wire            s_cs,
    output  wire            s_miso,
    input   wire            s_mosi,

    input   wire [2:0]      mux_spi

);


wire DAC1_CSn;
wire DAC1_CLK;
wire DAC1_SDO;        
wire DAC1_SDIO_i;
wire DAC1_SDIO_o;

wire DAC2_CSn;
wire DAC2_CLK;
wire DAC2_SDO;        
wire DAC2_SDIO_i;
wire DAC2_SDIO_o;

wire  LMK_SDIO_o;
wire  LMK_SDIO_i;
wire  LMK_CS;
wire  LMK_SCK;

wire  SYNT1_CLK;
wire  SYNT1_SDI;
wire  SYNT1_LE;

wire  SYNT2_CLK;
wire  SYNT2_SDI;
wire  SYNT2_LE;

reg SYNT1_MUXOUT_D_s;
reg SYNT1_MUXOUT_DD_s;
reg SYNT1_MUXOUT_FD_s;
reg SYNT2_MUXOUT_D_s;
reg SYNT2_MUXOUT_DD_s;
reg SYNT2_MUXOUT_FD_s;
wire FPGA_REFCLK_i;
wire FPGA_REFCLK_s;
BUFG bufg0 (
    .I(FPGA_REFCLK_i),
    .O(FPGA_REFCLK_s)
);
always @(posedge FPGA_REFCLK_s, posedge SYNT1_MUXOUT_FD_CLR) begin
    if (SYNT1_MUXOUT_FD_CLR) begin
        SYNT1_MUXOUT_D_s <= 0;
        SYNT1_MUXOUT_DD_s <= 0;
        SYNT1_MUXOUT_FD_s <= 0;
    end else begin
        SYNT1_MUXOUT_D_s <= fmc_la09_p;
        SYNT1_MUXOUT_DD_s <= SYNT1_MUXOUT_D_s;
        SYNT1_MUXOUT_FD_s <= SYNT1_MUXOUT_FD_s |
            (SYNT1_MUXOUT_DD_s & ~SYNT1_MUXOUT_D_s); 
    end
end
assign SYNT1_MUXOUT_FD = SYNT1_MUXOUT_FD_s;
always @(posedge FPGA_REFCLK_s, posedge SYNT2_MUXOUT_FD_CLR) begin
    if (SYNT2_MUXOUT_FD_CLR) begin
        SYNT2_MUXOUT_D_s <= 0;
        SYNT2_MUXOUT_DD_s <= 0;
        SYNT2_MUXOUT_FD_s <= 0;
    end else begin
        SYNT2_MUXOUT_D_s <= fmc_la20_p;
        SYNT2_MUXOUT_DD_s <= SYNT2_MUXOUT_D_s;
        SYNT2_MUXOUT_FD_s <= SYNT2_MUXOUT_FD_s |
            (SYNT2_MUXOUT_DD_s & ~SYNT2_MUXOUT_D_s); 
    end
end
assign SYNT2_MUXOUT_FD = SYNT2_MUXOUT_FD_s;

assign DAC1_IRQn        = fmc_la01_p;
assign fmc_la01_n       = DAC1_RSTn;
assign fmc_la04_p       = DAC1_CSn;
assign fmc_la05_p       = LMK_CS;
assign fmc_la06_p       = DAC1_TXEN;
assign fmc_la06_n       = LMK_SCK;
assign fmc_la07_p       = SYNT1_PDRFn;
assign DAC1_SDO         = fmc_la07_n; //only for 4wire operation   
assign fmc_la08_p       = DAC1_RFOUT_EN; 
assign DAC1_PROTOUT     = fmc_la08_n; //   : in std_logic; --DAC1_PROTOUT
assign SYNT1_MUXOUT     = fmc_la09_p; //   
assign fmc_la09_n       = SYNT1_CE;
assign MON_ALERT        = fmc_la10_n; //   : in std_logic; --MON_ALERT
assign fmc_la11_p       = SYNT1_LE;
assign fmc_la11_n       = DAC1_CLK;
assign fmc_la12_n       = QZ_ENn;
assign fmc_la13_p       = SYNT1_SDI;
assign fmc_la13_n       = SYNT1_CLK;
assign fmc_la14_p       = SYNT2_PDRFn;
assign fmc_la14_n       = SYNT2_CE;
assign LMK_STATLD1      = fmc_la15_p;
assign LMK_STATLD2      = fmc_la15_n;
assign fmc_la16_p       = REFCLKOUT_EN;
assign fmc_la19_p       = LMK_SYNC;
assign fmc_la19_n       = LMK_CLKSEL0;
assign SYNT2_MUXOUT     = fmc_la20_p;
assign fmc_la20_n       = LMK_CLKSEL1;
assign fmc_la21_p       = SYNT2_CLK;
assign fmc_la21_n       = SYNT2_SDI;
assign fmc_la22_p       = LMK_RESET;
assign fmc_la22_n       = SYNT2_LE;
assign fmc_la24_p       = DAC2_TXEN;
assign fmc_la24_n       = DAC2_RSTn;
assign DAC2_IRQn        = fmc_la29_p;
assign fmc_la30_p       = DAC2_CLK;
assign DAC2_PROTOUT     = fmc_la32_p;
assign fmc_la32_n       = DAC2_RFOUT_EN; 
assign DAC2_SDO         = fmc_la33_p;
assign fmc_la33_n       = DAC2_CSn;
//
assign fmc_dp00_c2m_p = DAC1_TXD_p[0];
assign fmc_dp00_c2m_n = DAC1_TXD_n[0];
assign fmc_dp01_c2m_p = DAC1_TXD_p[1];
assign fmc_dp01_c2m_n = DAC1_TXD_n[1];
assign fmc_dp02_c2m_p = DAC1_TXD_p[2];
assign fmc_dp02_c2m_n = DAC1_TXD_n[2];
assign fmc_dp03_c2m_p = DAC1_TXD_p[3];
assign fmc_dp03_c2m_n = DAC1_TXD_n[3];
//
assign fmc_dp04_c2m_p = DAC2_TXD_p[0];
assign fmc_dp04_c2m_n = DAC2_TXD_n[0];
assign fmc_dp05_c2m_p = DAC2_TXD_p[1];
assign fmc_dp05_c2m_n = DAC2_TXD_n[1];
assign fmc_dp06_c2m_p = DAC2_TXD_p[2];
assign fmc_dp06_c2m_n = DAC2_TXD_n[2];
assign fmc_dp07_c2m_p = DAC2_TXD_p[3];
assign fmc_dp07_c2m_n = DAC2_TXD_n[3];

daq2_spi 
lmk_spi_inst (
    .spi_csn (fmc_la05_p), 
    .spi_clk (fmc_la06_n), 
    .spi_mosi (LMK_SDIO_o),
    .spi_miso (LMK_SDIO_i),

    .spi_sdio (fmc_la05_n),
    .spi_dir ()
);

daq2_spi 
dac1_spi_inst (
    .spi_csn (DAC1_CSn), 
    .spi_clk (DAC1_CLK), 
    .spi_mosi (DAC1_SDIO_o),
    .spi_miso (DAC1_SDIO_i),

    .spi_sdio (fmc_la12_p),
    .spi_dir ()
);

daq2_spi 
dac2_spi_inst (
    .spi_csn (DAC2_CSn), 
    .spi_clk (DAC2_CLK), 
    .spi_mosi (DAC2_SDIO_o),
    .spi_miso (DAC2_SDIO_i),

    .spi_sdio (fmc_la30_n),
    .spi_dir ()
);

localparam SPI_COUNT = 5;

wire    [SPI_COUNT-1:0]       m_sck;
wire    [SPI_COUNT-1:0]       m_cs;
wire    [SPI_COUNT-1:0]       m_miso;
wire    [SPI_COUNT-1:0]       m_mosi;

assign DAC1_CSn = m_cs[0];
assign DAC1_CLK = m_sck[0];
assign m_miso[0] = DAC1_SDIO_i;
assign DAC1_SDIO_o = m_mosi[0];

assign DAC2_CSn = m_cs[1];
assign DAC2_CLK = m_sck[1];
assign m_miso[1] = DAC2_SDIO_i;
assign DAC2_SDIO_o = m_mosi[1];

assign  LMK_CS = m_cs[2];
assign  LMK_SCK = m_sck[2];
assign  m_miso[2] = LMK_SDIO_i; 
assign  LMK_SDIO_o = m_mosi[2];

assign  SYNT1_LE = m_cs[3];
assign  SYNT1_CLK = m_sck[3];
assign  SYNT1_SDI = m_mosi[3];

assign  SYNT2_LE = m_cs[4];
assign  SYNT2_CLK  = m_sck[4];
assign  SYNT2_SDI  = m_mosi[4];

demux_spi#(
    .M_COUNT(SPI_COUNT)
)
demux_spi_inst
(
    .s_sck(s_sck),
    .s_cs(s_cs),
    .s_miso(s_miso),
    .s_mosi(s_mosi),

    .m_sck(m_sck),
    .m_cs(m_cs),
    .m_miso(m_miso),
    .m_mosi(m_mosi),
   
    .mux(mux_spi)
);

IBUFDS_GTE3 IBUF_GBTCLK0_i (
    .O(FPGA_GBTCLK0),           // 1-bit output: Refer to Transceiver User Guide
    .ODIV2(FPGA_GBTCLK0_ODIV2), // 1-bit output: Refer to Transceiver User Guide
    .CEB(0),                    // 1-bit input: Refer to Transceiver User Guide
    .I(fmc_gbtclk0_m2c_p),      // 1-bit input: Refer to Transceiver User Guide
    .IB(fmc_gbtclk0_m2c_n)      // 1-bit input: Refer to Transceiver User Guide
);
IBUFDS #(
    .DIFF_TERM(1),
    .IBUF_LOW_PWR(0),
    .IOSTANDARD("LVDS")
    ) IBUF_REFCLK_i (
    .O(FPGA_REFCLK_i),  // 1-bit output: Refer to Transceiver User Guide
    .I(fmc_clk0_m2c_p), // 1-bit input: Refer to Transceiver User Guide
    .IB(fmc_clk0_m2c_n) // 1-bit input: Refer to Transceiver User Guide
);
assign FPGA_REFCLK = FPGA_REFCLK_i;

IBUFDS #(
    .DIFF_TERM(1),
    .IBUF_LOW_PWR(1),
    .IOSTANDARD("LVDS")
    ) IBUFDS_SYSREF_i (
    .O(SYSREF_OUT), // 1-bit output: Buffer output
    .I(fmc_la00_p), // 1-bit input: Diff_p buffer input (connect directly to top-level port)
    .IB(fmc_la00_n) // 1-bit input: Diff_n buffer input (connect directly to top-level port)
);

IBUFDS #(
    .DIFF_TERM(1),
    .IBUF_LOW_PWR(1),
    .IOSTANDARD("LVDS")
    ) IBUFDS_EXTSYNCIN_i (
    .O(EXTSYNC_IN),     // 1-bit output: Buffer output
    .I(fmc_clk1_m2c_p), // 1-bit input: Diff_p buffer input (connect directly to top-level port)
    .IB(fmc_clk1_m2c_n) // 1-bit input: Diff_n buffer input (connect directly to top-level port)
);

OBUFDS #( 
    .IOSTANDARD("LVDS"),// Specify the output I/O standard
    .SLEW("FAST")       // Specify the output slew rate
    ) OBUFDS_SYNC1_i (
    .O(fmc_clk2_bidir_p),   // Diff_p output (connect directly to top-level port)
    .OB(fmc_clk2_bidir_n),  // Diff_n output (connect directly to top-level port)
    .I(REFCLK2FMC)      // Buffer input 
);

IBUFDS #(
    .DIFF_TERM(1),
    .IBUF_LOW_PWR(1),
    .IOSTANDARD("LVDS")
    ) IBUFDS_DAC1SYNC_i (
    .O(DAC1_SYNCOUT),   // 1-bit output: Buffer output
    .I(fmc_la17_p),     // 1-bit input: Diff_p buffer input (connect directly to top-level port)
    .IB(fmc_la17_n)     // 1-bit input: Diff_n buffer input (connect directly to top-level port)
);

IBUFDS #(
    .DIFF_TERM(1),
    .IBUF_LOW_PWR(1),
    .IOSTANDARD("LVDS")
    ) IBUFDS_DAC2SYNC_i (
    .O(DAC2_SYNCOUT),   // 1-bit output: Buffer output
    .I(fmc_la18_p),     // 1-bit input: Diff_p buffer input (connect directly to top-level port)
    .IB(fmc_la18_n)     // 1-bit input: Diff_n buffer input (connect directly to top-level port)
);

endmodule
