// =============================================================================
// uart_rx.v — Receptor UART 8N1
// =============================================================================
// Baud rate configurável via parâmetro CLKS_PER_BIT (mesmo valor do uart_tx):
//   Para 50 MHz e 115200 baud: CLKS_PER_BIT = 50_000_000 / 115_200 ≈ 434
//
// Interface:
//   i_Rx_Serial: entrada serial (1 = idle/mark)
//   o_Rx_DV    : pulso de 1 ciclo quando byte válido está em o_Rx_Byte
//   o_Rx_Byte  : byte recebido (válido no ciclo em que o_Rx_DV = 1)
//
// Detalhes de implementação:
//   * Sincronizador de 2 FF na entrada (evita metaestabilidade)
//   * Amostragem no MEIO do período de bit (CLKS_PER_BIT/2 após borda de start)
//     garantindo máxima margem de ruído
//   * Sem paridade (8N1: 1 start, 8 dados, 1 stop)
//   * Estado CLEAN espelha uart_tx: garante que o_Rx_DV dure exatamente 1 ciclo
// =============================================================================

module uart_rx #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       i_Rx_Serial,
    output reg        o_Rx_DV,
    output reg  [7:0] o_Rx_Byte
);

    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
    localparam DATA  = 3'd2;
    localparam STOP  = 3'd3;
    localparam CLEAN = 3'd4;

    // ── Sincronizador de 2 FF (metaestabilidade) ───────────────────────────
    reg rx_ff1, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff1  <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_ff1  <= i_Rx_Serial;
            rx_sync <= rx_ff1;
        end
    end

    // ── FSM ───────────────────────────────────────────────────────────────
    reg [2:0]  state;
    reg [8:0]  clk_cnt;   // mesmo tamanho do uart_tx
    reg [2:0]  bit_idx;
    reg [7:0]  rx_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            clk_cnt <= 9'd0;
            bit_idx <= 3'd0;
            rx_data <= 8'd0;
            o_Rx_DV   <= 1'b0;
            o_Rx_Byte <= 8'd0;
        end else begin
            o_Rx_DV <= 1'b0;

            case (state)
                // ── Aguarda borda de descida (start bit) ──────────────────
                IDLE: begin
                    clk_cnt <= 9'd0;
                    bit_idx <= 3'd0;
                    if (rx_sync == 1'b0)       // detectou início do start bit
                        state <= START;
                end

                // ── Confirma start bit no meio do período ─────────────────
                // Amostra em CLKS_PER_BIT/2: se ainda for 0, é start legítimo
                START: begin
                    if (clk_cnt == (CLKS_PER_BIT/2) - 1) begin
                        if (rx_sync == 1'b0) begin
                            clk_cnt <= 9'd0;   // zera para alinhar DATA no centro
                            state   <= DATA;
                        end else begin
                            state   <= IDLE;   // glitch — descarta
                        end
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                // ── Amostra cada bit de dados no centro do período ────────
                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt            <= 9'd0;
                        rx_data[bit_idx]   <= rx_sync;   // LSB primeiro (UART padrão)
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= STOP;
                        end else
                            bit_idx <= bit_idx + 3'd1;
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                // ── Verifica stop bit e publica byte ──────────────────────
                STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        if (rx_sync == 1'b1) begin   // stop bit válido
                            o_Rx_DV   <= 1'b1;
                            o_Rx_Byte <= rx_data;
                        end
                        // stop inválido (framing error): descarta silenciosamente
                        clk_cnt <= 9'd0;
                        state   <= CLEAN;
                    end else
                        clk_cnt <= clk_cnt + 9'd1;
                end

                // ── Espelha estado CLEAN do uart_tx (1 ciclo extra) ───────
                CLEAN: begin
                    o_Rx_DV <= 1'b0;
                    state   <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule