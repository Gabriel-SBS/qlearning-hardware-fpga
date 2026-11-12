// =============================================================================
// uart_tx.v — Transmissor UART 8N1
// =============================================================================
// Baud rate configurável via parâmetro CLKS_PER_BIT:
//   Para 50 MHz e 115200 baud: CLKS_PER_BIT = 50_000_000 / 115_200 ≈ 434
//
// Interface:
//   i_Tx_DV    : 1 ciclo de pulso para iniciar transmissão
//   i_Tx_Byte  : byte a transmitir
//   o_Tx_Active: 1 enquanto transmitindo
//   o_Tx_Done  : ATENÇÃO — fica em nível alto por **2 ciclos** (asserted no
//                fim de STOP e reafirmado em CLEAN), NÃO 1 ciclo como uma
//                versão anterior deste cabeçalho afirmava. Qualquer lógica
//                que consuma o_Tx_Done DEVE usar detecção de borda de subida
//                (ver tx_done_pulse em qlearning_top_v2.v); sem isso, uma FSM
//                que avança em "if (tx_done)" pula 2 estados por byte —
//                exatamente o bug de embaralhamento de pacotes encontrado e
//                corrigido via co-simulação cocotb.
//   o_Tx_Serial: saída serial (1 = idle/mark)
// =============================================================================

module uart_tx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       i_Tx_DV,
    input  wire [7:0] i_Tx_Byte,
    output reg        o_Tx_Active,
    output wire       o_Tx_Serial,
    output reg        o_Tx_Done
);

    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
    localparam DATA  = 3'd2;
    localparam STOP  = 3'd3;
    localparam CLEAN = 3'd4;

    reg [2:0]  state;
    reg [8:0]  clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  tx_data;
    reg        tx_serial;

    assign o_Tx_Serial = tx_serial;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            o_Tx_Active <= 1'b0;
            o_Tx_Done   <= 1'b0;
            tx_serial   <= 1'b1;
            clk_cnt     <= 9'd0;
            bit_idx     <= 3'd0;
        end else begin
            o_Tx_Done <= 1'b0;
            case (state)
                IDLE: begin
                    tx_serial   <= 1'b1;
                    clk_cnt     <= 9'd0;
                    bit_idx     <= 3'd0;
                    if (i_Tx_DV) begin
                        o_Tx_Active <= 1'b1;
                        tx_data     <= i_Tx_Byte;
                        state       <= START;
                    end
                end

                START: begin
                    tx_serial <= 1'b0;  // bit de start
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 9'd0;
                        state   <= DATA;
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                DATA: begin
                    tx_serial <= tx_data[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 9'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= STOP;
                        end else
                            bit_idx <= bit_idx + 3'd1;
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                STOP: begin
                    tx_serial <= 1'b1;  // bit de stop
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        o_Tx_Done   <= 1'b1;
                        o_Tx_Active <= 1'b0;
                        clk_cnt     <= 9'd0;
                        state       <= CLEAN;
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                CLEAN: begin
                    o_Tx_Done <= 1'b1;
                    state     <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
