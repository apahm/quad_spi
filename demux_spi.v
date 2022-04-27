module demux_spi#(
    parameter M_COUNT = 5,
    parameter MUX_ADDR = $clog2(M_COUNT) 
)
(
    input   wire                        s_sck,
    input   wire                        s_cs,
    output  reg                         s_miso,
    input   wire                        s_mosi,

    output  reg    [M_COUNT-1:0]        m_sck,
    output  reg    [M_COUNT-1:0]        m_cs,
    input   wire    [M_COUNT-1:0]       m_miso,
    output  reg    [M_COUNT-1:0]        m_mosi,

    input   wire    [MUX_ADDR - 1:0]        mux
);
integer n;

initial begin
    for (n = 0; n < M_COUNT; n = n + 1) begin
            m_sck[n] = 1'b0;
            m_cs[n] = 1'b1;
            s_miso = 1'b0;
            m_mosi[n] = 1'b0;
    end
end



always @(*)
begin
    for (n = 0; n < M_COUNT; n = n + 1) begin
        if(mux == n) begin
            m_sck[n] = s_sck;
            m_cs[n] = s_cs;
            s_miso = m_miso[n];
            m_mosi[n] = s_mosi;
        end
        else
        begin
            m_sck[n] = 1'b0;
            m_cs[n] = 1'b1;
            m_mosi[n] = 1'b0;
        end
    end
end

endmodule