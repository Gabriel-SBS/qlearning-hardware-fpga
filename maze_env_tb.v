// =============================================================================
// maze_env_tb - Testbench de Validação para o Ambiente do Labirinto 32x32
// =============================================================================
// ATUALIZADO para a interface atual do maze_env (mapa dinâmico): o módulo
// agora possui clk/rst_n e porta de programação (prog_*). Este TB mantém a
// porta de programação inativa e valida a lógica combinacional de transição
// sobre o WALL_MAP_DEFAULT (a porta prog_* é coberta por tb_prog_port.v e
// tb_uart_map_e2e.v). É obrigatório aplicar reset antes dos casos de teste
// para carregar o mapa default no registrador wall_map.
//
// Compilação (Icarus):
//   iverilog -g2012 -o sim_maze maze_env_tb.v maze_env.v
//   vvp sim_maze
// =============================================================================
// Objetivo da Simulação:
//   Garantir a corretude da lógica combinacional do módulo 'maze_env' sob as
//   diretrizes do algoritmo de Q-Learning. O testbench valida os seguintes cenários:
//
//     1. Movimentação Livre : Deslocamento em área limpa e cálculo do R_STEP (-1).
//     2. Limites do Mapa    : Colisões com as bordas (Y=0) e retorno do R_WALL (-5).
//     3. Paredes Internas   : Teste de colisão estática baseada no WALL_MAP.
//     4. Ação de Inércia    : Comportamento da ação STOP e retorno do R_STOP (-2).
//     5. Estado de Meta     : Transição para (31,31), sinal 'done' e R_GOAL (+120).
//
// Formato de Visualização:
//   Os resultados são exibidos decodificados em Coordenadas (Y, X) no console
//   com blocos de auto-verificação (SUCESSO/FALHA) para agilizar o debug.
// =============================================================================

`timescale 1ns / 1ps

module maze_env_tb;

    reg [9:0] state;
    reg [2:0] action;

    wire [9:0] next_state;
    wire signed [7:0] reward;
    wire done;

    // Interface síncrona da porta de programação (inativa neste TB: o alvo
    // aqui é a lógica combinacional de transição sobre o WALL_MAP_DEFAULT;
    // a porta de programação em si é coberta por tb_prog_port.v)
    reg  clk   = 1'b0;
    reg  rst_n = 1'b0;
    wire map_ready;

    always #5 clk = ~clk;

    maze_env maze_env_inst (
        .state(state),
        .action(action),
        .next_state(next_state),
        .reward(reward),
        .done(done),
        .clk(clk),
        .rst_n(rst_n),
        .prog_we(1'b0),
        .prog_byte_addr(7'd0),
        .prog_byte_data(8'd0),
        .map_ready(map_ready)
    );

    localparam [2:0] UP    = 3'd0;
    localparam [2:0] DOWN  = 3'd1;
    localparam [2:0] LEFT  = 3'd2;
    localparam [2:0] RIGHT = 3'd3;
    localparam [2:0] STOP  = 3'd4;


    initial begin
        $timeformat(-9, 1, " ns", 12);
        
        $display("=========================================================");
        $display("   INICIANDO SIMULAÇÃO DO AMBIENTE LABIRINTO (MAZE_ENV)   ");
        $display("=========================================================");

        // Reset: carrega WALL_MAP_DEFAULT no registrador wall_map (o mapa é
        // dinâmico desde a adição da porta de programação — sem reset, o
        // wall_map fica em X e todo teste de colisão passa/falha ao acaso)
        rst_n = 1'b0;
        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        if (map_ready !== 1'b1)
            $display(" AVISO: map_ready != 1 após reset (esperado 1: mapa default é válido)");
        
        // ---------------------------------------------------------------------
        // CASO 1: Movimento Válido em Espaço Livre (Passo Padrão)
        // ---------------------------------------------------------------------
        // Iniciando em (Y=9, X=10) -> Estado: 10'b01010_01010 (0x14A)
        state  = {5'd9, 5'd10}; 
        action = RIGHT;
        #10;
        $display("[PASSO VÁLIDO] Pos Inicial: (9,10) | Ação: DIREITA");
        $display(" -> Próximo Estado: (Y=%0d, X=%0d) | Recompensa: %0d | Fim: %b", 
                  next_state[9:5], next_state[4:0], reward, done);
        if (next_state == {5'd9, 5'd11} && reward == -8'sd1) 
            $display(" STATUS: SUCESSO\n");
        else 
            $display(" STATUS: FALHA\n");

        // ---------------------------------------------------------------------
        // CASO 2: Colisão com a Borda Superior do Mapa (Y=0)
        // ---------------------------------------------------------------------
        state  = {5'd0, 5'd5}; 
        action = UP;
        #10;
        $display("[BORDA] Pos Inicial: (0,5) | Ação: CIMA (Borda Superior)");
        $display(" -> Próximo Estado: (Y=%0d, X=%0d) | Recompensa: %0d | Fim: %b", 
                  next_state[9:5], next_state[4:0], reward, done);
        if (next_state == {5'd0, 5'd5} && reward == -8'sd5) 
            $display(" STATUS: SUCESSO\n");
        else 
            $display(" STATUS: FALHA\n");

        // ---------------------------------------------------------------------
        // CASO 3: Colisão com Parede Interna (Baseado no WALL_MAP)
        // ---------------------------------------------------------------------
        // No WALL_MAP_DEFAULT atual, a posição (0,13) possui um bit '1'
        // (bloqueado) e (0,12) é livre. Vamos tentar mover para a DIREITA a
        // partir de (0,12). (O teste antigo usava (0,0)→(0,1), parede que só
        // existia no mapa default do sistema legado.)
        state  = {5'd0, 5'd12}; 
        action = RIGHT;
        #10;
        $display("[PAREDE INTERNA] Pos Inicial: (0,12) | Ação: DIREITA (Tentando ir para (0,13))");
        $display(" -> Próximo Estado: (Y=%0d, X=%0d) | Recompensa: %0d | Fim: %b", 
                  next_state[9:5], next_state[4:0], reward, done);
        if (next_state == {5'd0, 5'd12} && reward == -8'sd5) 
            $display(" STATUS: SUCESSO\n");
        else 
            $display(" STATUS: FALHA\n");

        // ---------------------------------------------------------------------
        // CASO 4: Penalidade por Inércia (Ação STOP)
        // ---------------------------------------------------------------------
        state  = {5'd15, 5'd15}; 
        action = STOP;
        #10;
        $display("[INÉRCIA] Pos Inicial: (15,15) | Ação: PARAR");
        $display(" -> Próximo Estado: (Y=%0d, X=%0d) | Recompensa: %0d | Fim: %b", 
                  next_state[9:5], next_state[4:0], reward, done);
        if (next_state == {5'd15, 5'd15} && reward == -8'sd2) 
            $display(" STATUS: SUCESSO\n");
        else 
            $display(" STATUS: FALHA\n");

        // ---------------------------------------------------------------------
        // CASO 5: Chegada ao Estado de Meta (Goal)
        // ---------------------------------------------------------------------
        // Estando em (31,30) e movendo para a DIREITA para alcançar (31,31)
        state  = {5'd31, 5'd30}; 
        action = RIGHT;
        #10;
        $display("[META] Pos Inicial: (31,30) | Ação: DIREITA -> Meta (31,31)");
        $display(" -> Próximo Estado: (Y=%0d, X=%0d) | Recompensa: %0d | Fim: %b", 
                  next_state[9:5], next_state[4:0], reward, done);
        if (next_state == 10'b11111_11111 && reward == 8'sd120 && done == 1'b1) 
            $display(" STATUS: SUCESSO\n");
        else 
            $display(" STATUS: FALHA\n");

        $display("=========================================================");
        $display("                FIM DA SIMULAÇÃO                         ");
        $display("=========================================================");
        $finish;
    end

endmodule