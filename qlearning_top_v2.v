// =============================================================================
// qlearning_top_v2.v — Top-Level Q-Learning 32x32 (FSM otimizada, RAM 5 bancos)
// =============================================================================
// Protocolo UART RX (1 byte por mensagem, exceto upload de mapa):
//
//   0xA0        : start — inicia/reinicia o treinamento
//   0xFF        : reset global (soft reset de 8 ciclos)
//   0xB0        : início de upload de mapa — os próximos 128 bytes são os dados
//                 do WALL_MAP (byte 0 = bits[7:0] … byte 127 = bits[1023:1016]).
//                 O treinamento não pode iniciar enquanto map_ready = 0.
//                 Após o byte 127, map_ready sobe e o treinamento pode começar.
//   Outros      : ignorados
//
// Protocolo UART TX (5 bytes por episódio):
//   Byte 0 : episode[15:8]
//   Byte 1 : episode[7:0]
//   Byte 2 : steps[11:4]
//   Byte 3 : {steps[3:0], reward[7:4]}
//   Byte 4 : {reward[3:0], goal, action[2:0]}   (goal = meta atingida)
//
// Parâmetros:
//   CLKS_PER_BIT : divisor de baud rate (padrão 434 → 115200 @ 50 MHz)
// =============================================================================

module qlearning_top_v2 #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    // UART
    input  wire i_Rx_Serial,
    output wire o_Tx_Serial,
    // LEDs / status
    output wire [3:0] led_ep,
    output wire training_done_o
);

    // =========================================================================
    // Sinais internos — núcleo Q-Learning
    // =========================================================================
    wire        ram_we;
    wire [9:0]  ram_addr, ram_addr_n;
    wire [2:0]  ram_wr_action;
    wire signed [15:0] ram_din;
    wire signed [15:0] q0, q1, q2, q3, q4;
    wire signed [15:0] qn0, qn1, qn2, qn3, qn4;

    wire [7:0]  rand_byte, rand_high;
    wire        lfsr_en;

    wire [9:0]  mon_state;
    wire [2:0]  mon_action;
    wire signed [7:0]  mon_reward;
    wire [15:0] mon_episode;
    wire [11:0] mon_steps;
    wire        mon_done;
    wire        mon_valid;
    wire        mon_goal;
    wire        training_done;

    // ── Sinal de prontidão do mapa (vindo do maze_env) ─────────────────────
    wire map_ready;

    // =========================================================================
    // Reset combinado (hardware rst_n  +  soft reset via UART 0xFF)
    // =========================================================================
    reg [3:0] soft_rst_cnt;
    wire      soft_rst_active = (soft_rst_cnt != 4'd0);
    wire      rst_n_int       = rst_n & ~soft_rst_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            soft_rst_cnt <= 4'd0;
        else if (soft_rst_active)
            soft_rst_cnt <= soft_rst_cnt - 4'd1;
    end

    // =========================================================================
    // UART RX — recepção de comandos e dados do mapa
    // =========================================================================
    wire       rx_dv;
    wire [7:0] rx_byte;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) rx_inst (
        .clk        (clk),
        .rst_n      (rst_n_int),
        .i_Rx_Serial(i_Rx_Serial),
        .o_Rx_DV    (rx_dv),
        .o_Rx_Byte  (rx_byte)
    );

    // ── Máquina de recepção de mapa ─────────────────────────────────────────
    // Estados: aguarda 0xB0, depois recebe 128 bytes e escreve no maze_env.
    localparam RX_CMD  = 1'b0;   // aguardando byte de comando
    localparam RX_MAP  = 1'b1;   // recebendo bytes do mapa

    reg        rx_fsm;
    reg [6:0]  map_byte_cnt;     // 0..127

    reg        prog_we;
    reg [6:0]  prog_byte_addr;
    reg [7:0]  prog_byte_data;
    reg        uart_start_pulse;

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            rx_fsm          <= RX_CMD;
            map_byte_cnt    <= 7'd0;
            prog_we         <= 1'b0;
            prog_byte_addr  <= 7'd0;
            prog_byte_data  <= 8'd0;
            uart_start_pulse <= 1'b0;
            // soft_rst_cnt carregado pelo seu próprio bloco
        end else begin
            prog_we          <= 1'b0;   // pulso de 1 ciclo por padrão
            uart_start_pulse <= 1'b0;

            case (rx_fsm)
                // ── Interpreta byte de comando ──────────────────────────────
                RX_CMD: begin
                    if (rx_dv) begin
                        case (rx_byte)
                            8'hA0: uart_start_pulse <= 1'b1;
                            8'hFF: soft_rst_cnt     <= 4'd8;
                            8'hB0: begin
                                // Inicia recepção de mapa
                                map_byte_cnt <= 7'd0;
                                rx_fsm       <= RX_MAP;
                            end
                            default: ;
                        endcase
                    end
                end

                // ── Recebe os 128 bytes do mapa e repassa ao maze_env ───────
                // Byte N do stream → prog_byte_addr = N (addr 0 = bits[7:0])
                RX_MAP: begin
                    if (rx_dv) begin
                        prog_we        <= 1'b1;
                        prog_byte_addr <= map_byte_cnt;
                        prog_byte_data <= rx_byte;

                        if (map_byte_cnt == 7'd127) begin
                            map_byte_cnt <= 7'd0;
                            rx_fsm       <= RX_CMD;
                        end else begin
                            map_byte_cnt <= map_byte_cnt + 7'd1;
                        end
                    end
                end

                default: rx_fsm <= RX_CMD;
            endcase
        end
    end

    // Start combinado: botão físico OU comando UART, apenas se mapa estiver pronto
    wire start_int = (start | uart_start_pulse) & map_ready;

    // =========================================================================
    // UART TX — transmissão de pacote ao fim de cada episódio
    // =========================================================================
    // CORREÇÃO: o gatilho antigo era a borda de subida de mon_done, que só
    // ocorre quando o episódio termina COM META ATINGIDA. Episódios encerrados
    // por timeout de MAX_STEPS nunca disparavam pacote — em labirintos densos
    // (onde a política aleatória inicial raramente acha a meta) o treinamento
    // inteiro rodava em silêncio absoluto na UART. Agora usamos mon_valid, um
    // pulso de 1 ciclo emitido pela FSM em S_NEW_EP para TODO fim de episódio,
    // no mesmo instante em que mon_episode/mon_steps são registrados.
    wire ep_done_pulse = mon_valid;

    reg [15:0] pkt_episode;
    reg [11:0] pkt_steps;
    reg  [7:0] pkt_reward;
    reg  [2:0] pkt_action;
    reg        pkt_goal;

    localparam [2:0]
        TX_IDLE = 3'd0,
        TX_B0   = 3'd1,
        TX_B1   = 3'd2,
        TX_B2   = 3'd3,
        TX_B3   = 3'd4,
        TX_B4   = 3'd5;

    reg [2:0] tx_seq;
    reg       tx_dv;
    reg [7:0] tx_byte;

    wire tx_done;
    wire tx_active;

    // o_Tx_Done do uart_tx.v fica em alto por 2 ciclos (CLEAN e o 1o ciclo de
    // IDLE), embora o comentário do módulo diga "pulso de 1 ciclo". Sem
    // detecção de borda aqui, tx_seq amostra tx_done=1 duas vezes seguidas e
    // avança dois estados por byte, descartando 1 a cada 2 bytes do pacote
    // (bug encontrado via testbench cocotb "modo FPGA" ao decodificar os
    // pacotes de episódio). Aplicamos aqui o mesmo padrão de borda já usado
    // para ep_done_pulse acima.
    reg tx_done_r;
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) tx_done_r <= 1'b0;
        else            tx_done_r <= tx_done;
    end
    wire tx_done_pulse = tx_done & ~tx_done_r;

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            tx_seq      <= TX_IDLE;
            tx_dv       <= 1'b0;
            tx_byte     <= 8'd0;
            pkt_episode <= 16'd0;
            pkt_steps   <= 12'd0;
            pkt_reward  <= 8'd0;
            pkt_action  <= 3'd0;
            pkt_goal    <= 1'b0;
        end else begin
            tx_dv <= 1'b0;

            case (tx_seq)
                TX_IDLE: begin
                    if (ep_done_pulse) begin
                        pkt_episode <= mon_episode;
                        pkt_steps   <= mon_steps;
                        pkt_reward  <= {mon_reward};
                        pkt_action  <= mon_action;
                        pkt_goal    <= mon_goal;   // flag explícita de meta
                        tx_byte <= mon_episode[15:8];
                        tx_dv   <= 1'b1;
                        tx_seq  <= TX_B0;
                    end
                end
                TX_B0: if (tx_done_pulse) begin tx_byte <= pkt_episode[7:0];                          tx_dv <= 1'b1; tx_seq <= TX_B1; end
                TX_B1: if (tx_done_pulse) begin tx_byte <= pkt_steps[11:4];                           tx_dv <= 1'b1; tx_seq <= TX_B2; end
                TX_B2: if (tx_done_pulse) begin tx_byte <= {pkt_steps[3:0], pkt_reward[7:4]};         tx_dv <= 1'b1; tx_seq <= TX_B3; end
                TX_B3: if (tx_done_pulse) begin tx_byte <= {pkt_reward[3:0], pkt_goal, pkt_action};       tx_dv <= 1'b1; tx_seq <= TX_B4; end
                TX_B4: if (tx_done_pulse) tx_seq <= TX_IDLE;
                default: tx_seq <= TX_IDLE;
            endcase
        end
    end

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) tx_inst (
        .clk        (clk),
        .rst_n      (rst_n_int),
        .i_Tx_DV    (tx_dv),
        .i_Tx_Byte  (tx_byte),
        .o_Tx_Active(tx_active),
        .o_Tx_Serial(o_Tx_Serial),
        .o_Tx_Done  (tx_done)
    );

    // =========================================================================
    // Núcleo Q-Learning
    // =========================================================================
    q_table_ram5 ram_inst (
        .clk(clk), .we(ram_we),
        .addr(ram_addr), .addr_n(ram_addr_n),
        .wr_action(ram_wr_action), .din(ram_din),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .qn0(qn0), .qn1(qn1), .qn2(qn2), .qn3(qn3), .qn4(qn4)
    );

    lfsr16 lfsr_inst (
        .clk(clk), .rst_n(rst_n_int), .en(lfsr_en),
        .rand_out(rand_byte), .rand_high(rand_high)
    );

    // ── maze_env com porta de programação dinâmica ──────────────────────────
    // ARQUITETURA: o maze_env é instanciado AQUI (top-level), e não dentro da
    // FSM, para que a porta de programação prog_* (recarga do mapa via UART)
    // fique acessível à máquina de recepção acima. O laço combinacional
    // fecha assim, tudo dentro do mesmo ciclo de clock:
    //
    //   fsm (cur_state_o/cur_action_o) ──▶ maze_env ──▶ (next_state/reward/
    //   done) ──▶ fsm (env_next_state/env_reward/env_done)
    //
    // A FSM (fsm_qlearning_v2) já possui as portas env_* / cur_*_o — este
    // arranjo está completo e validado; nenhuma refatoração pendente.
    wire [9:0]  fsm_state_out;   // cur_state da FSM → entrada 'state' do maze_env
    wire [2:0]  fsm_action_out;  // cur_action da FSM → entrada 'action' do maze_env

    wire [9:0]  env_next_state;
    wire signed [7:0] env_reward;
    wire        env_done;

    maze_env maze_inst (
        // Treinamento (combinacional) — usa cur_state/cur_action da FSM
        .state       (fsm_state_out),
        .action      (fsm_action_out),
        .next_state  (env_next_state),
        .reward      (env_reward),
        .done        (env_done),
        // Programação dinâmica do mapa
        .clk         (clk),
        .rst_n       (rst_n_int),
        .prog_we     (prog_we),
        .prog_byte_addr(prog_byte_addr),
        .prog_byte_data(prog_byte_data),
        .map_ready   (map_ready)
    );

    fsm_qlearning_v2 #(
        .MAX_STEPS(1000), .MAX_EPISODES(5000),
        .EPSILON_INIT(254), .EPSILON_MIN(10),
        .EPSILON_DECAY(15), .EPSILON_DECAY_START(300)
    ) fsm_inst (
        .clk(clk), .rst_n(rst_n_int), .start(start_int),
        .ram_we(ram_we), .ram_addr(ram_addr), .ram_addr_n(ram_addr_n),
        .ram_wr_action(ram_wr_action), .ram_din(ram_din),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .qn0(qn0), .qn1(qn1), .qn2(qn2), .qn3(qn3), .qn4(qn4),
        .rand_byte(rand_byte), .rand_high(rand_high), .lfsr_en(lfsr_en),
        // Saídas de ambiente injetadas externamente (mapa dinâmico)
        .env_next_state(env_next_state),
        .env_reward    (env_reward),
        .env_done      (env_done),
        .mon_goal      (mon_goal),
        // Estado/ação atuais → alimentam o maze_env no top-level
        .cur_state_o   (fsm_state_out),
        .cur_action_o  (fsm_action_out),
        // Monitor
        .mon_state(mon_state), .mon_action(mon_action),
        .mon_reward(mon_reward), .mon_episode(mon_episode),
        .mon_steps(mon_steps), .mon_done(mon_done), .mon_valid(mon_valid),
        .training_done(training_done)
    );

    // =========================================================================
    // Saídas
    // =========================================================================
    assign led_ep          = mon_episode[3:0];
    assign training_done_o = training_done;

endmodule