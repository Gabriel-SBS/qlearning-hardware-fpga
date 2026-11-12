// =============================================================================
// fsm_qlearning_v2_tb.v — Testbench Individual: fsm_qlearning_v2 (5 estados)
// =============================================================================
// Substitui o antigo fsm_qlearning_tb.v (que validava a FSM v1 de 23 estados
// com RAM única 5120x16 endereçada por state*5+action). A v2 usa a
// q_table_ram5 (5 bancos, 2 portas de leitura) e fecha o passo inteiro de
// treinamento combinacionalmente em S_STEP (1 ciclo/passo).
//
// Instancia o DUT junto com q_table_ram5, lfsr16 e maze_env — a MESMA
// topologia do qlearning_top_v2 (o maze_env externo à FSM, com o mapa
// default e a porta de programação inativa) — mas com MAX_STEPS /
// MAX_EPISODES reduzidos para permitir simulação completa em tempo aceitável.
//
// Verifica:
//   1. Estado de reset: fsm_state = S_IDLE, training_done = 0; permanece em
//      S_IDLE sem start
//   2. Contrato de endereçamento da RAM5: durante S_STEP, ram_addr ==
//      cur_state e ram_addr_n == env_next_state (as duas portas de leitura
//      entregam Q(s,·) e Q(s',·) no MESMO ciclo — não existem mais os
//      estados S_RQ0..S_RQ4 da v1)
//   3. ram_we asserted exatamente em S_STEP (1 escrita por passo), com
//      ram_wr_action == cur_action; toda escrita realmente aterrissa no
//      banco/endereço certos (read-back via hierarquia após cada we)
//   4. Total de escritas == MAX_EPISODES × MAX_STEPS (partindo de (0,0) e
//      com meta em (31,31), poucos passos ⇒ todo episódio fecha por timeout)
//   5. mon_valid pulsa exatamente 1 vez por episódio (== MAX_EPISODES) e
//      mon_steps == MAX_STEPS em episódios por timeout; mon_goal permanece 0
//   6. Decaimento de epsilon ao longo dos episódios
//   7. training_done asserted após MAX_EPISODES completos
//   8. S_DONE aceita novo start SEM reset: contadores/epsilon reiniciam,
//      Q-table é preservada (regressão do antigo "beco sem saída")
//
// Compilação (Icarus):
//   iverilog -g2012 -o sim_fsm_v2 fsm_qlearning_v2_tb.v fsm_qlearning_v2.v \
//            q_table_ram5.v ula_bellman.v argmax5.v lfsr16.v maze_env.v
//   vvp sim_fsm_v2
// =============================================================================
`timescale 1ns/1ns

module fsm_qlearning_v2_tb;

    localparam MAX_STEPS_TB    = 20;
    localparam MAX_EPISODES_TB = 4;
    localparam EPS_INIT_TB     = 20;
    localparam EPS_MIN_TB      = 10;
    localparam EPS_DECAY_TB    = 1;
    localparam EPS_DECAY_START = 0;

    reg clk;
    reg rst_n;
    reg start;

    // Interface FSM <-> q_table_ram5
    wire        ram_we;
    wire [9:0]  ram_addr, ram_addr_n;
    wire [2:0]  ram_wr_action;
    wire signed [15:0] ram_din;
    wire signed [15:0] q0, q1, q2, q3, q4;
    wire signed [15:0] qn0, qn1, qn2, qn3, qn4;

    // LFSR
    wire [7:0]  rand_byte, rand_high;
    wire        lfsr_en;

    // Ambiente (combinacional, externo à FSM — topologia do top v2)
    wire [9:0]        env_next_state;
    wire signed [7:0] env_reward;
    wire              env_done;
    wire [9:0]        cur_state_w;
    wire [2:0]        cur_action_w;
    wire              map_ready;

    // Monitor
    wire [9:0]  mon_state;
    wire [2:0]  mon_action;
    wire signed [7:0]  mon_reward;
    wire [15:0] mon_episode;
    wire [11:0] mon_steps;
    wire        mon_done, mon_valid, mon_goal;
    wire        training_done;

    integer pass = 0;
    integer fail = 0;

    // --- Subcomponentes (mesma topologia do qlearning_top_v2) -------------
    q_table_ram5 ram_inst (
        .clk(clk), .we(ram_we),
        .addr(ram_addr), .addr_n(ram_addr_n),
        .wr_action(ram_wr_action), .din(ram_din),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .qn0(qn0), .qn1(qn1), .qn2(qn2), .qn3(qn3), .qn4(qn4)
    );

    lfsr16 lfsr_inst (
        .clk(clk), .rst_n(rst_n), .en(lfsr_en),
        .rand_out(rand_byte), .rand_high(rand_high)
    );

    // maze_env com o WALL_MAP_DEFAULT (porta de programação inativa)
    maze_env maze_inst (
        .state       (cur_state_w),
        .action      (cur_action_w),
        .next_state  (env_next_state),
        .reward      (env_reward),
        .done        (env_done),
        .clk         (clk),
        .rst_n       (rst_n),
        .prog_we     (1'b0),
        .prog_byte_addr(7'd0),
        .prog_byte_data(8'd0),
        .map_ready   (map_ready)
    );

    fsm_qlearning_v2 #(
        .MAX_STEPS(MAX_STEPS_TB), .MAX_EPISODES(MAX_EPISODES_TB),
        .EPSILON_INIT(EPS_INIT_TB), .EPSILON_MIN(EPS_MIN_TB),
        .EPSILON_DECAY(EPS_DECAY_TB), .EPSILON_DECAY_START(EPS_DECAY_START)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .ram_we(ram_we), .ram_addr(ram_addr), .ram_addr_n(ram_addr_n),
        .ram_wr_action(ram_wr_action), .ram_din(ram_din),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .qn0(qn0), .qn1(qn1), .qn2(qn2), .qn3(qn3), .qn4(qn4),
        .rand_byte(rand_byte), .rand_high(rand_high), .lfsr_en(lfsr_en),
        .env_next_state(env_next_state),
        .env_reward    (env_reward),
        .env_done      (env_done),
        .cur_state_o   (cur_state_w),
        .cur_action_o  (cur_action_w),
        .mon_state(mon_state), .mon_action(mon_action),
        .mon_reward(mon_reward), .mon_episode(mon_episode),
        .mon_steps(mon_steps), .mon_done(mon_done),
        .mon_valid(mon_valid), .mon_goal(mon_goal),
        .training_done(training_done)
    );

    // --- Acesso interno para checagem (via hierarquia, simulação apenas) --
    wire [2:0] fsm_state_w = dut.fsm_state;
    wire [7:0] epsilon_w   = dut.epsilon;

    localparam S_IDLE    = 3'd0;
    localparam S_INIT_EP = 3'd1;
    localparam S_STEP    = 3'd2;
    localparam S_NEW_EP  = 3'd3;
    localparam S_DONE    = 3'd4;

    // Clock 10ns
    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_eq;
        input [63:0] got;
        input [63:0] exp;
        input [511:0] test_id;
        begin
            if (got === exp) begin
                $display("  [PASS] %0s: %0d", test_id, got);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: obtido=%0d esperado=%0d", test_id, got, exp);
                fail = fail + 1;
            end
        end
    endtask

    // Variante para valores signed de 16 bits (exibição com sinal correto)
    task check_eq_s16;
        input signed [15:0] got;
        input signed [15:0] exp;
        input [511:0] test_id;
        begin
            if (got === exp) begin
                $display("  [PASS] %0s: %0d", test_id, got);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: obtido=%0d esperado=%0d", test_id, got, exp);
                fail = fail + 1;
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Monitor 1: contrato combinacional das 2 portas da RAM5 em S_STEP
    //   ram_addr   == cur_state       (leitura Q(s,·) e escrita)
    //   ram_addr_n == env_next_state  (leitura Q(s',·) no MESMO ciclo)
    // -----------------------------------------------------------------
    integer addr_mismatches;
    always @(posedge clk) begin
        if (!rst_n) begin
            addr_mismatches <= 0;
        end else begin
            #1; // amostra depois da atualização dos regs no mesmo timestep
            if (fsm_state_w == S_STEP) begin
                if (ram_addr   !== cur_state_w)    addr_mismatches <= addr_mismatches + 1;
                if (ram_addr_n !== env_next_state) addr_mismatches <= addr_mismatches + 1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Monitor 2: pulsos de escrita — só em S_STEP, com wr_action ==
    // cur_action; captura (addr, banco, din) e confere no ciclo seguinte
    // (read-back via hierarquia) que a escrita aterrissou no lugar certo
    // -----------------------------------------------------------------
    integer we_pulses;
    integer we_ctl_mismatches;   // we fora de S_STEP ou wr_action errado
    integer we_data_mismatches;  // read-back diferente do din escrito
    reg        wr_pend;
    reg [9:0]  wr_addr_r;
    reg [2:0]  wr_act_r;
    reg signed [15:0] wr_din_r;

    function signed [15:0] mem_rd;
        input [2:0] bank;
        input [9:0] a;
        begin
            case (bank)
                3'd0: mem_rd = ram_inst.mem0[a];
                3'd1: mem_rd = ram_inst.mem1[a];
                3'd2: mem_rd = ram_inst.mem2[a];
                3'd3: mem_rd = ram_inst.mem3[a];
                default: mem_rd = ram_inst.mem4[a];
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            we_pulses          <= 0;
            we_ctl_mismatches  <= 0;
            we_data_mismatches <= 0;
            wr_pend            <= 1'b0;
        end else begin
            // read-back da escrita capturada na borda anterior
            if (wr_pend && mem_rd(wr_act_r, wr_addr_r) !== wr_din_r)
                we_data_mismatches <= we_data_mismatches + 1;
            wr_pend <= 1'b0;

            #1; // amostra sinais combinacionais/regs já atualizados
            if (ram_we) begin
                we_pulses <= we_pulses + 1;
                if (fsm_state_w !== S_STEP || ram_wr_action !== cur_action_w)
                    we_ctl_mismatches <= we_ctl_mismatches + 1;
                // OBS: esta amostra (pós-borda) captura os valores do PRÓXIMO
                // passo; para o read-back precisamos dos valores vigentes NA
                // borda — capturados no bloco síncrono abaixo.
            end
        end
    end

    // Captura dos valores vigentes exatamente na borda de escrita
    always @(posedge clk) begin
        if (rst_n && ram_we) begin
            wr_pend   <= 1'b1;
            wr_addr_r <= ram_addr;
            wr_act_r  <= ram_wr_action;
            wr_din_r  <= ram_din;
        end
    end

    // -----------------------------------------------------------------
    // Monitor 3: fins de episódio — mon_valid deve pulsar 1x por episódio;
    // registra epsilon vigente e steps/goal reportados
    // -----------------------------------------------------------------
    integer ep_seen;
    integer steps_bad;   // mon_steps != MAX_STEPS em episódio por timeout
    integer goal_seen;   // mon_goal em 1 num mapa que não é resolvido em 20 passos
    reg [7:0] eps_hist [0:MAX_EPISODES_TB-1];

    always @(posedge clk) begin
        #1; // espera atualização dos regs
        if (!rst_n) begin
            ep_seen   <= 0;
            steps_bad <= 0;
            goal_seen <= 0;
        end else if (fsm_state_w == S_NEW_EP) begin
            // S_NEW_EP dura exatamente 1 ciclo: aqui mon_valid ainda vai
            // subir na próxima borda, mas o epsilon vigente do episódio que
            // termina ainda não decaiu (o decaimento é aplicado NESTA borda)
            if (ep_seen < MAX_EPISODES_TB)
                eps_hist[ep_seen] <= epsilon_w;
            ep_seen <= ep_seen + 1;
        end else if (mon_valid) begin
            // ciclo seguinte a S_NEW_EP: telemetria registrada e válida
            if (mon_steps !== MAX_STEPS_TB) steps_bad <= steps_bad + 1;
            if (mon_goal)                   goal_seen <= goal_seen + 1;
        end
    end

    integer timeout;

    initial begin
        $display("-----------------------------------------------");
        $display("  TB_FSM_QLEARNING_V2 - Inicio dos testes");
        $display("  (MAX_STEPS=%0d, MAX_EPISODES=%0d)", MAX_STEPS_TB, MAX_EPISODES_TB);
        $display("-----------------------------------------------");

        rst_n = 1'b0;
        start = 1'b0;

        repeat (3) @(negedge clk);

        // ---------------------------------------------------------------
        // Teste 1: Estado apos reset
        // ---------------------------------------------------------------
        check_eq(fsm_state_w, S_IDLE, "Teste 1a: fsm_state == S_IDLE apos reset");
        check_eq(training_done, 1'b0, "Teste 1b: training_done == 0 apos reset");

        rst_n = 1'b1;
        @(negedge clk);
        check_eq(fsm_state_w, S_IDLE, "Teste 1c: permanece em S_IDLE sem start");
        check_eq(map_ready, 1'b1, "Teste 1d: map_ready == 1 (WALL_MAP_DEFAULT valido)");

        // ---------------------------------------------------------------
        // Dispara o treinamento
        // ---------------------------------------------------------------
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // ---------------------------------------------------------------
        // Roda o treinamento completo (MAX_EPISODES_TB episodios) com timeout
        // (v2: 1 ciclo/passo -> ~4*(20+2) ciclos; margem folgada)
        // ---------------------------------------------------------------
        timeout = 0;
        while (!training_done && timeout < 2000) begin
            @(negedge clk);
            timeout = timeout + 1;
        end

        // ---------------------------------------------------------------
        // Teste 2: treinamento concluido dentro do timeout
        // ---------------------------------------------------------------
        check_eq(training_done, 1'b1, "Teste 2: training_done == 1 ao final");

        // ---------------------------------------------------------------
        // Teste 3: contrato das 2 portas de leitura (Q(s,.) e Q(s',.)
        // disponiveis no MESMO ciclo, sem estados S_RQx)
        // ---------------------------------------------------------------
        check_eq(addr_mismatches, 0,
                 "Teste 3: ram_addr==cur_state e ram_addr_n==next_state em todo S_STEP");

        // ---------------------------------------------------------------
        // Teste 4: escritas — 1 por passo, todas em S_STEP, no banco da
        // acao tomada, e com read-back correto
        // ---------------------------------------------------------------
        check_eq(we_pulses, MAX_EPISODES_TB * MAX_STEPS_TB,
                 "Teste 4a: total de escritas == MAX_EPISODES*MAX_STEPS");
        check_eq(we_ctl_mismatches, 0,
                 "Teste 4b: we so em S_STEP e wr_action == cur_action sempre");
        check_eq(we_data_mismatches, 0,
                 "Teste 4c: read-back de toda escrita bate com ram_din");

        // ---------------------------------------------------------------
        // Teste 5: telemetria de fim de episodio
        // ---------------------------------------------------------------
        check_eq(ep_seen, MAX_EPISODES_TB, "Teste 5a: mon_valid pulsou 1x por episodio");
        check_eq(steps_bad, 0, "Teste 5b: mon_steps == MAX_STEPS em todo timeout");
        check_eq(goal_seen, 0, "Teste 5c: mon_goal == 0 (meta inalcancavel em 20 passos)");
        check_eq(mon_steps, MAX_STEPS_TB, "Teste 5d: mon_steps final == MAX_STEPS");

        // ---------------------------------------------------------------
        // Teste 6: decaimento de epsilon (EPS_INIT=20, decai 1/episodio)
        //   eps_hist[k] = epsilon vigente durante o episodio k
        // ---------------------------------------------------------------
        check_eq(eps_hist[0], EPS_INIT_TB,     "Teste 6a: epsilon no episodio 0 == EPSILON_INIT");
        check_eq(eps_hist[1], EPS_INIT_TB - 1, "Teste 6b: epsilon no episodio 1 == EPSILON_INIT-1");
        check_eq(eps_hist[2], EPS_INIT_TB - 2, "Teste 6c: epsilon no episodio 2 == EPSILON_INIT-2");
        check_eq(eps_hist[3], EPS_INIT_TB - 3, "Teste 6d: epsilon no episodio 3 == EPSILON_INIT-3");

        // ---------------------------------------------------------------
        // Teste 7: reinicio a partir de S_DONE SEM reset (regressao do
        // "beco sem saida"): training_done cai, contadores/epsilon
        // reiniciam e a Q-table aprendida e PRESERVADA
        // ---------------------------------------------------------------
        repeat (3) @(negedge clk);
        check_eq(fsm_state_w, S_DONE, "Teste 7a: FSM estacionada em S_DONE");
        check_eq(training_done, 1'b1, "Teste 7b: training_done permanece em 1 em S_DONE");

        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        @(negedge clk); // S_INIT_EP: nenhuma escrita ainda desde o restart

        check_eq(training_done, 1'b0, "Teste 7c: training_done == 0 apos novo start");
        check_eq(dut.episode_cnt, 16'd0, "Teste 7d: episode_cnt zerado apos restart");
        check_eq(epsilon_w, EPS_INIT_TB, "Teste 7e: epsilon == EPSILON_INIT apos restart");
        // ultima escrita do treino anterior ainda intacta na RAM (sem reset)
        check_eq_s16(mem_rd(wr_act_r, wr_addr_r), wr_din_r,
                 "Teste 7f: Q-table preservada atraves do restart");

        // segundo treinamento tambem completa
        timeout = 0;
        while (!training_done && timeout < 2000) begin
            @(negedge clk);
            timeout = timeout + 1;
        end
        check_eq(training_done, 1'b1, "Teste 7g: segundo treinamento tambem conclui");
        check_eq(we_pulses, 2 * MAX_EPISODES_TB * MAX_STEPS_TB,
                 "Teste 7h: escritas do 2o treino tambem contabilizadas");

        $display("-----------------------------------------------");
        $display("  RESULTADO: %0d PASS / %0d FAIL", pass, fail);
        $display("-----------------------------------------------");
        if (fail == 0)
            $display("fsm_qlearning_v2 funcionando.");
        else
            $display("Falhas no fsm_qlearning_v2!");
        $finish;
    end

endmodule
