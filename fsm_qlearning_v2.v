// =============================================================================
// fsm_qlearning_v2.v — FSM + Datapath do Agente Q-Learning (32x32) — OTIMIZADA
// =============================================================================
// Otimizacao em relacao a versao original (fsm_qlearning.v, 23 estados):
//
//   * RAM 5120x16 (1 porta, addr=state*5+action) -> q_table_ram5 (5 bancos de
//     1024x16, um por acao, endereçados por 'state', com 2 portas de leitura).
//     Q(s,0..4) e Q(s',0..4) ficam disponiveis no MESMO ciclo, sem precisar
//     dos estados sequenciais S_RQ0..S_RQ4 / S_RQN0..S_RQN4.
//
//   * maze_env, argmax5 e ula_bellman ja sao combinacionais. Logo a cadeia
//       Q(s,.) -> argmax -> acao -> ambiente -> Q(s',.) -> argmax -> Bellman
//     inteira e resolvida em UM ciclo, dentro do estado S_STEP.
//
//   * Resultado: 23 estados -> 5 estados; ~11 ciclos/passo -> 1 ciclo/passo
//     (fora o overhead fixo de S_INIT_EP/S_NEW_EP por episodio).
//
//   * TRADE-OFF: o caminho combinacional de S_STEP agora e longo (2x leitura
//     de RAM + 2x argmax5 + maze_env + ula_bellman em serie), reduzindo o
//     Fmax em troca de muito menos ciclos totais. Se isso for um problema na
//     sintese, registre nxt_state/cur_action/max_q_nxt em um estagio extra
//     (pipeline de 2 estados) -> 2 ciclos/passo, ainda ~5x mais rapido que
//     o original.
// =============================================================================

module fsm_qlearning_v2 #(
    parameter MAX_STEPS           = 3000,
    parameter MAX_EPISODES        = 5000,
    parameter EPSILON_INIT        = 254,
    parameter EPSILON_MIN         = 10,
    parameter EPSILON_DECAY       = 15,
    parameter EPSILON_DECAY_START = 300
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Interface com q_table_ram5 (porta dupla, 5 bancos)
    output wire        ram_we,
    output wire [9:0]  ram_addr,    // = cur_state  (leitura/escrita Q(s,.))
    output wire [9:0]  ram_addr_n,  // = nxt_state  (leitura Q(s',.))
    output wire [2:0]  ram_wr_action,
    output wire signed [15:0] ram_din,
    input  wire signed [15:0] q0, q1, q2, q3, q4,       // Q(ram_addr,   0..4)
    input  wire signed [15:0] qn0, qn1, qn2, qn3, qn4,  // Q(ram_addr_n, 0..4)

    input  wire [7:0]  rand_byte,   // escolha da acao aleatoria
    input  wire [7:0]  rand_high,   // decisao epsilon
    output reg         lfsr_en,

    output reg  [9:0]  mon_state,
    output reg  [2:0]  mon_action,
    output reg  signed [7:0]  mon_reward,
    output reg  [15:0] mon_episode,
    output reg  [11:0] mon_steps,
    output reg         mon_done,
    // Pulso de 1 ciclo em TODO fim de episódio (meta atingida OU timeout de
    // MAX_STEPS). É o gatilho correto para a telemetria UART no top-level:
    // usar apenas a borda de mon_done fazia episódios encerrados por timeout
    // nunca emitirem pacote (labirintos difíceis => silêncio total na UART).
    output reg         mon_valid,
    // Flag "meta atingida" PERSISTENTE durante todo o fim de episódio.
    // mon_done é limpo por default a cada ciclo e já vale 0 no ciclo em
    // que mon_valid fica visível ao top — por isso NÃO serve para ser
    // amostrado pela telemetria UART. mon_goal é sticky: sobe em S_STEP
    // quando ep_done=1 e só é limpo no S_INIT_EP do episódio seguinte.
    output reg         mon_goal,
    output reg         training_done,

    // ── Ambiente externo (mapa dinâmico injetado pelo top-level) ────────────
    // O maze_env é instanciado no top para suportar reprogramação via UART.
    // A FSM recebe os resultados combinacionais através destas portas.
    input  wire [9:0]           env_next_state,
    input  wire signed [7:0]    env_reward,
    input  wire                 env_done,

    // ── Estado e ação atuais expostos ao top-level para alimentar o maze_env
    output wire [9:0]           cur_state_o,
    output wire [2:0]           cur_action_o
);

    // ── Estados da FSM ────────────────────────────────────────────────────
    localparam [2:0]
        S_IDLE    = 3'd0,
        S_INIT_EP = 3'd1,
        S_STEP    = 3'd2,
        S_NEW_EP  = 3'd3,
        S_DONE    = 3'd4;

    reg [2:0]  fsm_state;
    reg [9:0]  cur_state;
    reg [15:0] episode_cnt;
    reg [11:0] step_cnt;
    reg [7:0]  epsilon;
    reg [7:0]  decay_cnt;

    // ── argmax sobre Q(s,.) — combinacional, direto dos bancos ─────────────
    wire [2:0] best_act;
    wire signed [15:0] max_q_cur;
    argmax5 argmax_cur (
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .best_action(best_act), .max_q(max_q_cur)
    );

    // ── politica epsilon-greedy (combinacional) ────────────────────────────
    wire [2:0] cur_action = (rand_high < epsilon) ? (rand_byte % 5) : best_act;

    // ── ambiente (combinacional, via portas externas do top-level) ─────────
    // O maze_env foi movido para qlearning_top_v2 para suportar reprogramação
    // dinâmica do mapa via UART. Os resultados chegam aqui como entradas.
    wire [9:0]        nxt_state  = env_next_state;
    wire signed [7:0] cur_reward = env_reward;
    wire              ep_done    = env_done;

    assign cur_state_o  = cur_state;
    assign cur_action_o = cur_action;

    assign ram_addr   = cur_state;
    assign ram_addr_n = nxt_state;

    // ── argmax sobre Q(s',.) — 2a porta de leitura, endereçada por nxt_state
    wire [2:0] best_act_n;
    wire signed [15:0] max_q_nxt;
    argmax5 argmax_nxt (
        .q0(qn0), .q1(qn1), .q2(qn2), .q3(qn3), .q4(qn4),
        .best_action(best_act_n), .max_q(max_q_nxt)
    );

    // ── Bellman (combinacional) ─────────────────────────────────────────────
    wire signed [15:0] q_old =
        (cur_action == 3'd0) ? q0 :
        (cur_action == 3'd1) ? q1 :
        (cur_action == 3'd2) ? q2 :
        (cur_action == 3'd3) ? q3 : q4;

    wire signed [15:0] q_new_wire;
    ula_bellman ula_inst (
        .q_old(q_old), .reward(cur_reward), .max_q_next(max_q_nxt),
        .q_new(q_new_wire)
    );

    assign ram_we        = (fsm_state == S_STEP);
    assign ram_wr_action = cur_action;
    assign ram_din       = q_new_wire;

    // ── FSM principal ─────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state     <= S_IDLE;
            training_done <= 1'b0;
            episode_cnt   <= 16'd0;
            step_cnt      <= 12'd0;
            epsilon       <= EPSILON_INIT[7:0];
            decay_cnt     <= 8'd0;
            lfsr_en       <= 1'b0;
            mon_done      <= 1'b0;
            mon_valid     <= 1'b0;
            mon_goal      <= 1'b0;
            cur_state     <= 10'd0;
        end else begin
            lfsr_en  <= 1'b1;
            mon_done  <= 1'b0;
            mon_valid <= 1'b0;

            case (fsm_state)
                S_IDLE: begin
                    training_done <= 1'b0;
                    if (start) begin
                        episode_cnt <= 16'd0;
                        epsilon     <= EPSILON_INIT[7:0];
                        fsm_state   <= S_INIT_EP;
                    end
                end

                S_INIT_EP: begin
                    cur_state <= 10'd0;
                    step_cnt  <= 12'd0;
                    mon_goal  <= 1'b0;   // limpa flag do episódio anterior
                    fsm_state <= S_STEP;
                end

                // ── Unico estado de "passo": leitura, argmax, ambiente,
                //    leitura de Q(s',.), argmax_n e Bellman ja estao prontos
                //    de forma combinacional acima; aqui so registramos.
                S_STEP: begin
                    // Escreve Q(cur_state, cur_action) atualizado no banco
                    // correspondente a 'cur_action' (ram_we/ram_din/ram_wr_action
                    // sao combinacionais, derivados deste estado)

                    // Monitoramento
                    mon_state  <= cur_state;
                    mon_action <= cur_action;
                    mon_reward <= cur_reward;
                    mon_done   <= ep_done;
                    if (ep_done) mon_goal <= 1'b1;   // sticky até S_INIT_EP

                    step_cnt <= step_cnt + 12'd1;

                    if (ep_done || step_cnt >= MAX_STEPS[11:0] - 12'd1) begin
                        fsm_state <= S_NEW_EP;
                    end else begin
                        cur_state <= nxt_state;
                        fsm_state <= S_STEP;
                    end
                end

                S_NEW_EP: begin
                    mon_episode <= episode_cnt;
                    mon_steps   <= step_cnt;
                    mon_valid   <= 1'b1;  // pulso: episódio terminou (meta OU timeout)
                    if (episode_cnt >= MAX_EPISODES[15:0] - 16'd1)
                        fsm_state <= S_DONE;
                    else begin
                        episode_cnt <= episode_cnt + 16'd1;

                        if (episode_cnt >= EPSILON_DECAY_START[15:0]) begin
                            if (decay_cnt == 8'd0) begin
                                if (epsilon > EPSILON_MIN[7:0])
                                    epsilon <= epsilon - 8'd1;
                            end
                            if (decay_cnt >= EPSILON_DECAY[7:0] - 8'd1)
                                decay_cnt <= 8'd0;
                            else
                                decay_cnt <= decay_cnt + 8'd1;
                        end

                        fsm_state <= S_INIT_EP;
                    end
                end

                // S_DONE aceita um novo start (botão ou 0xA0 via UART) para
                // reiniciar o treinamento SEM reset: preserva o mapa enviado
                // por UART e a Q-table aprendida (a RAM não tem reset), apenas
                // zera contadores e epsilon. Antes, S_DONE era um beco sem
                // saída — só o soft reset 0xFF escapava dela, e ele tem o
                // efeito colateral de restaurar o WALL_MAP_DEFAULT.
                S_DONE: begin
                    training_done <= 1'b1;
                    if (start) begin
                        training_done <= 1'b0;
                        episode_cnt   <= 16'd0;
                        epsilon       <= EPSILON_INIT[7:0];
                        decay_cnt     <= 8'd0;
                        fsm_state     <= S_INIT_EP;
                    end
                end

                default: fsm_state <= S_IDLE;
            endcase
        end
    end

endmodule