// =============================================================================
// argmax5_tb.v — Testbench Individual: argmax5
// =============================================================================
// Verifica:
//   1. Seleção correta do maior Q-value em casos sem empate
//   2. Prioridade de menor índice em caso de empate
//   3. Valores negativos com sinal
//   4. Máximo no último índice (ação 4)
//   5. Todos os valores iguais (empate total → ação 0)
// =============================================================================
`timescale 1ns/1ns

module argmax5_tb;

    reg  signed [15:0] q0, q1, q2, q3, q4;
    wire signed [15:0] max_q;
    wire        [2:0]  best_action;

    argmax5 argmax5_inst (
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .best_action(best_action),
        .max_q(max_q)
    );

    integer pass = 0;
    integer fail = 0;

    task check;
        input        [2:0]  exp_act;
        input signed [15:0] exp_max;
        input        [63:0] test_id;
        begin
            #1; // Espera os sinais combinacionais estabilizarem
            if (best_action === exp_act && max_q === exp_max) begin
                $display("  [PASS] Teste %0d: best_action=%0d, max_q=%0d",
                         test_id, best_action, max_q);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] Teste %0d: esperado act=%0d max=%0d | obtido act=%0d max=%0d",
                         test_id, exp_act, exp_max, best_action, max_q);
                fail = fail + 1;
            end
            #1;
        end
    endtask

    initial begin
        $display("-----------------------------------------------");
        $display("  TB_ARGMAX5 - Início dos testes");
        $display("-----------------------------------------------");

        // Teste 1: máximo em q0
        q0= 16'sd100; q1= 16'sd50;  q2= 16'sd30;  q3= 16'sd10;  q4= 16'sd0;
        check(3'd0, 16'sd100, 1);

        // Teste 2: máximo em q1
        q0= 16'sd10;  q1= 16'sd200; q2= 16'sd50;  q3= 16'sd30;  q4= 16'sd20;
        check(3'd1, 16'sd200, 2);

        // Teste 3: máximo em q2
        q0= 16'sd5;   q1= 16'sd10;  q2= 16'sd300; q3= 16'sd50;  q4= 16'sd1;
        check(3'd2, 16'sd300, 3);

        // Teste 4: máximo em q3
        q0= 16'sd1;   q1= 16'sd2;   q2= 16'sd3;   q3= 16'sd999; q4= 16'sd4;
        check(3'd3, 16'sd999, 4);

        // Teste 5: máximo em q4
        q0= 16'sd1;   q1= 16'sd2;   q2= 16'sd3;   q3= 16'sd4;   q4= 16'sd500;
        check(3'd4, 16'sd500, 5);

        // Teste 6: todos iguais → prioridade ação 0
        q0= 16'sd50;  q1= 16'sd50;  q2= 16'sd50;  q3= 16'sd50;  q4= 16'sd50;
        check(3'd0, 16'sd50, 6);

        // Teste 7: q0=q1 empatados, q2..4 menores → prioridade ação 0
        q0= 16'sd100; q1= 16'sd100; q2= 16'sd99;  q3= 16'sd99;  q4= 16'sd99;
        check(3'd0, 16'sd100, 7);

        // Teste 8: q1=q2 empatados, q0 menor → prioridade ação 1
        q0= 16'sd10;  q1= 16'sd100; q2= 16'sd100; q3= 16'sd50;  q4= 16'sd30;
        check(3'd1, 16'sd100, 8);

        // Teste 9: todos negativos — menos negativo = maior
        q0=-16'sd5;   q1=-16'sd3;   q2=-16'sd10;  q3=-16'sd1;   q4=-16'sd8;
        check(3'd3, -16'sd1, 9);

        // Teste 10: mistura negativo/positivo
        q0=-16'sd100; q1=-16'sd200; q2= 16'sd0;   q3=-16'sd50;  q4= 16'sd1;
        check(3'd4, 16'sd1, 10);

        // Teste 11: valores iniciais da Q-Table (-16 em Q4 = -1 normalizado)
        q0=-16'sd16;  q1=-16'sd16;  q2=-16'sd16;  q3=-16'sd16;  q4=-16'sd16;
        check(3'd0, -16'sd16, 11);

        // Teste 12: valor máximo positivo representável (16-bit signed)
        q0= 16'sd32767; q1= 16'sd0; q2= 16'sd0; q3= 16'sd0; q4= 16'sd0;
        check(3'd0, 16'sd32767, 12);

        // Teste 13: descrescente — máximo em q4 mesmo sendo o último
        q0= 16'sd50;  q1= 16'sd40;  q2= 16'sd30;  q3= 16'sd20;  q4= 16'sd100;
        check(3'd4, 16'sd100, 13);

        $display("-----------------------------------------------");
        $display("  RESULTADO: %0d PASS / %0d FAIL", pass, fail);
        $display("-----------------------------------------------");
        if (fail == 0)
            $display("argmax5 funcionando.");
        else
            $display("Falhas no argmax5!");
        $finish;
    end

endmodule