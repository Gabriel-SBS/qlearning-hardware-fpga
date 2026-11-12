// =============================================================================
// ula_bellman_tb.v — Testbench Individual: ULA Bellman (Escala Q4)
// =============================================================================
// Parâmetros de Projeto Verificados:
//   - Resolução temporal / Escala: Q4.4 (1/16 = 0.0625 de resolução)
//   - Fator de Aprendizado (Alpha): 0.5 (implementado via shift à direita `>>> 1`)
//   - Fator de Desconto (Gamma): 0.9375 (15/16, simulado nos cálculos esperados)
// =============================================================================

`timescale 1ns / 1ps

module ula_bellman_tb;

    reg  signed [15:0] q_old;
    reg  signed [7:0]  reward;
    reg  signed [15:0] max_q_next;
    wire signed [15:0] q_new;

    ula_bellman uut (
        .q_old(q_old),
        .reward(reward),
        .max_q_next(max_q_next),
        .q_new(q_new)
    );

    real real_q_old, real_reward, real_max_q_next, real_q_new;
    
    initial begin
        $display("=====================================================================================");
        $display("                 INÍCIO DOS TESTES: ULA BELLMAN Q4 (ESCALA x16)                      ");
        $display("=====================================================================================");
        $display(" Tempo |    Q_old (Real)   | Reward |   Max_Q_next (Real)  |    Q_new (Hex)   |   Q_new (Real)  ");
        $display("-------+-------------------+--------+----------------------+------------------+-----------------");

        // R = 100 -> R_q4 = 1600. TD = 1600. Q_new = 0 + 0.5 * 1600 = 800 (50.0)
        q_old      = 16'd0;
        reward     = 8'd100;
        max_q_next = 16'd0;
        #10;
        $display("[CASO 1] Condição Inicial (Tudo zero, recebe primeira recompensa)");
        print_signals();
        $display("-------+-------------------+--------+----------------------+------------------+-----------------");

        // R = 0, γ = 0.9375 -> γ*max = 15. TD = 15 - 0 = 15. Q_new = 0 + 15>>>1 = 7 (0.4375)
        q_old      = 16'd0;
        reward     = 8'd0;
        max_q_next = 16'd16; // 1.0 em Q4
        #10;
        $display("[CASO 2] Propagação de Longo Prazo (Sinal baixo sobrevivendo ao shift)");
        print_signals();
        $display("-------+-------------------+--------+----------------------+------------------+-----------------");

        // Q_old = 1500 (93.75), R = 120 (-> 1920), Max_Q = 1600 (100.0)
        // γ*max = 1600 - 100 = 1500. TD = 1920 + 1500 - 1500 = 1920.
        // Q_new = 1500 + 1920>>>1 = 1500 + 960 = 2460 (153.75)
        q_old      = 16'd1500; 
        reward     = 8'd120;   
        max_q_next = 16'd1600; 
        #10;
        $display("[CASO 3] Estado Estacionário / Convergência Próxima ao Alvo");
        print_signals();
        $display("-------+-------------------+--------+----------------------+------------------+-----------------");

        // R = -10 (Punição por colisão). Q_old = 500 (31.25), Max_Q = 400 (25.0)
        // R_q4 = -160. γ*max = 400 - 25 = 375.
        // TD = -160 + 375 - 500 = -285. 
        // Q_new = 500 + (-285 >>> 1) = 500 - 143 = 357 (22.3125)
        q_old      = 16'd500;
        reward     = -8'd10;
        max_q_next = 16'd400;
        #10;
        $display("[CASO 4] Punição / Valores Negativos (Sinalizado)");
        print_signals();
        $display("-------+-------------------+--------+----------------------+------------------+-----------------");

        // Q_old perto do limite (~30720). Vamos ver se não estoura o signed de 15 bits.
        // Max_Q_next = 30000. γ*max = 30000 - 1875 = 28125.
        q_old      = 16'd28000;
        reward     = 8'd100; // +1600 em Q4
        max_q_next = 16'd30000;
        #10;
        $display("[CASO 5] Teste de Estresse / Próximo ao Limite Máximo Teórico");
        print_signals();

        $display("=====================================================================================");
        $display("                               TESTES FINALIZADOS                                    ");
        $display("=====================================================================================");
        $finish;
    end

    // --- Tarefa Auxiliar para Printar e Converter Q4 para Real ---
    task print_signals;
        begin
            real_q_old      = convert_real(q_old) / 16.0;
            // Extensão de sinal correta para o reward (8 bits signed -> 16 bits)
            real_reward     = convert_real({{8{reward[7]}}, reward}); 
            real_max_q_next = convert_real(max_q_next) / 16.0;
            real_q_new      = convert_real(q_new) / 16.0;

            // Formatação alinhada com strings de tamanho fixo e indicação explícita de sinal (+/-)
            $display("%5dns | %+6d (%+9.4f) | %+6d | %+6d (%+10.4f) |    16'h%4h    | %+9.4f", 
                     $time, q_old, real_q_old, reward, max_q_next, real_max_q_next, q_new, real_q_new);
        end
    endtask

    // Função auxiliar para conversão de sinal com sinal em real
    function real convert_real(input reg [15:0] val);
        integer signed_val;
        begin
            signed_val = val;
            if (val[15]) signed_val = val - 65536;
            convert_real = signed_val;
        end
    endfunction

endmodule