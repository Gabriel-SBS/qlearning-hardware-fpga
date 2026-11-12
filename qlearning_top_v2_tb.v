// =============================================================================
// qlearning_top_v2_tb.v — Testbench Q-Learning 32x32 (versao otimizada)
// =============================================================================
// Testbench de sistema da arquitetura atual (fsm_qlearning_v2 + q_table_ram5).
// Mantém o formato do testbench de sistema da v1 (não incluído nesta pasta):
//   - estado de fim de episodio: S_NEW_EP = 3'd3 (era 5'd21 na v1)
//   - escrita na RAM e' detectada em q_table_ram5 (5 bancos paralelos)
//
// Uso:
//   iverilog -g2012 -o sim_v2 qlearning_top_v2_tb.v qlearning_top_v2.v \
//            fsm_qlearning_v2.v q_table_ram5.v ula_bellman.v argmax5.v \
//            lfsr16.v maze_env.v uart_tx.v uart_rx.v
//   vvp sim_v2
// =============================================================================

`timescale 1ns/1ns

module qlearning_top_v2_tb;

    reg  clk, rst_n, start;
    wire [3:0] led_ep;
    wire training_done_o;

    qlearning_top_v2 dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .led_ep(led_ep), .training_done_o(training_done_o)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    integer ep_count   = 0;
    integer goal_count = 0;
    integer max_ep     = 5000;

    initial begin
        rst_n = 0; start = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        start = 1; @(posedge clk); start = 0;

        $display("=== SIMULACAO Q-LEARNING 32x32 (FSM OTIMIZADA) INICIADA ===");
        $display("  Ep  | Passos | Meta? | Epsilon");
        $display("------|--------|-------|--------");
    end

    always @(posedge clk) begin
        if (dut.fsm_inst.fsm_state == 3'd3) begin  // S_NEW_EP
            ep_count = ep_count + 1;
            if (dut.fsm_inst.mon_done) begin
                goal_count = goal_count + 1;
                $display("%5d | %6d | SIM   | %3d",
                    dut.fsm_inst.episode_cnt,
                    dut.fsm_inst.step_cnt,
                    dut.fsm_inst.epsilon);
            end else if (ep_count % 200 == 0) begin
                $display("%5d | %6d | nao   | %3d",
                    dut.fsm_inst.episode_cnt,
                    dut.fsm_inst.step_cnt,
                    dut.fsm_inst.epsilon);
            end

            if (ep_count >= max_ep) begin
                $display("");
                $display("=== RESUMO APOS %0d EPISODIOS ===", max_ep);
                $display("Metas : %0d / %0d (%.1f%%)",
                    goal_count, max_ep, 100.0*goal_count/max_ep);
                $display("Escrit. RAM : %0d", write_count);
                $display("Epsilon final: %0d/255", dut.fsm_inst.epsilon);
                $finish;
            end
        end

        if (training_done_o) begin
            $display("=== TREINAMENTO COMPLETO -- metas: %0d ===", goal_count);
            $finish;
        end
    end

    // Contagem de escritas na Q-Table (5 bancos)
    integer write_count = 0;
    always @(posedge clk)
        if (dut.ram_inst.we) write_count = write_count + 1;

    // Verificacao: deve haver escrita nos primeiros 2000 ciclos
    initial begin
        #40000;
        if (write_count == 0) begin
            $display("ERRO: nenhuma escrita na Q-Table apos 2000 ciclos!");
            $finish;
        end else
            $display("CHECK OK: %0d escritas na Q-Table ate agora", write_count);
    end

endmodule