// =============================================================================
// Módulo      : maze_env
// Descrição   : Ambiente do Labirinto 32x32 para Q-Learning (Lógica Combinacional)
// =============================================================================
// Configuração do Ambiente:
//   - Mapa        : Grade 32x32 representada por um vetor de 1024 bits (wall_map).
//                   Bits em nível lógico '1' indicam paredes/obstáculos.
//                   O mapa pode ser carregado dinamicamente via porta de programação
//                   (prog_we + prog_byte_addr + prog_byte_data) ou usar o default
//                   via parâmetro WALL_MAP_DEFAULT.
//   - Estado      : 10 bits no total -> [9:5] representa a Linha (Y), [4:0] a Coluna (X).
//                   Posição Inicial: (0,0)  | Meta (Goal): (31,31).
//
// Porta de programação do mapa (síncrona, ativa quando prog_we = 1):
//   prog_byte_addr [6:0] : índice do byte a escrever (0..127, MSB = byte 127)
//   prog_byte_data [7:0] : valor do byte
//   Byte   0 cobre bits [7:0]       (células (0,0)..(0,7) — bit N = célula (N÷32, N mod 32))
//   Byte 127 cobre bits [1023:1016] (células (31,24)..(31,31))
//   Convenção idêntica à do generate_maze.py (hex string big-endian → índice 0 = LSB)
//
// Ações Possíveis (3 bits):
//   0 : CIMA     (UP)
//   1 : BAIXO    (DOWN)
//   2 : ESQUERDA (LEFT)
//   3 : DIREITA  (RIGHT)
//   4 : PARAR    (STOP)
//
// Estrutura de Recompensas (Rewards):
//   - Meta Atingida (R_GOAL) : +120
//   - Passo Válido (R_STEP)  :  -1
//   - Bater (R_WALL)         :  -5
//   - Ação Parar (R_STOP)    :  -2
// =============================================================================

module maze_env (
    // ── Interface de treinamento (combinacional) ───────────────────────────
    input  wire        [9:0] state,
    input  wire        [2:0] action,
    output reg         [9:0] next_state,
    output reg  signed [7:0] reward,
    output reg               done,

    // ── Porta de programação do mapa (síncrona) ───────────────────────────
    // Escreva os 128 bytes do mapa (byte a byte) antes de iniciar o treino.
    // Pulsar prog_we = 1 por 1 ciclo para cada byte.
    input  wire              clk,
    input  wire              rst_n,
    input  wire              prog_we,           // write enable: 1 ciclo por byte
    input  wire       [6:0]  prog_byte_addr,    // 0..127  (127 = bytes do MSB do mapa)
    input  wire       [7:0]  prog_byte_data,    // byte a escrever

    // ── Indica que o mapa está válido e pode começar o treino ─────────────
    output reg               map_ready          // 1 após 128 bytes recebidos
);

    // ── Parâmetro de mapa padrão (usado se nunca programado via UART) ──────
    parameter [1023:0] WALL_MAP_DEFAULT = 1024'h6070006C4A002608040CD0F018012041A1C9C1858100986731402E49052A14EE3300D611090475C53E6305082A5C080D8B1052C81072680045124801600D140990118884604050431314000080202829D201008CD0581000252980C006670100280B48080A1F0A45C0A2483281450418C002284240202B1C0A20822881002000;
    
    parameter signed [7:0] R_GOAL     = 8'sd120;
    parameter signed [7:0] R_STEP     = -8'sd1;
    parameter signed [7:0] R_WALL     = -8'sd5;
    parameter signed [7:0] R_STOP     = -8'sd2;
    parameter        [9:0] GOAL_STATE = 10'b11111_11111;

    // ── Registrador dinâmico do mapa de paredes ────────────────────────────
    // Inicializado com WALL_MAP_DEFAULT; sobrescrito byte a byte via prog_we.
    reg [1023:0] wall_map;

    // Contador de bytes recebidos (para sinalizar map_ready após 128)
    reg [7:0] prog_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wall_map <= WALL_MAP_DEFAULT;
            prog_cnt <= 8'd0;
            map_ready <= 1'b1;   // mapa default é válido desde o reset
        end else begin
            if (prog_we) begin
                // Cada byte escrito cobre 8 bits consecutivos do mapa.
                // prog_byte_addr=0 → bits [7:0], addr=127 → bits [1023:1016]
                // Equivalente à convenção do hex string no Python (LSB = addr 0).
                //
                // BUGFIX (índice truncado): a versão anterior usava
                //   wall_map[prog_byte_addr * 7'd8 +: 8]
                // A base de um part-select indexado é AUTODETERMINADA, e a
                // multiplicação de dois operandos de 7 bits produz resultado
                // de 7 bits → 127*8=1016 era truncado módulo 128. Efeito: os
                // 128 bytes davam 8 voltas sobre os bits [127:0] (linhas 0–3),
                // as linhas 4–31 ficavam com o WALL_MAP_DEFAULT, e as linhas
                // 0–3 terminavam com as linhas 28–31 do mapa enviado. Todo
                // upload via UART gerava um mapa "Frankenstein" quase sempre
                // insolúvel (timeout em 100% dos episódios), enquanto o
                // visualizador replayava no mapa correto — a causa raiz da
                // divergência HW × replay. A concatenação abaixo produz o
                // índice completo de 10 bits (addr*8) sem multiplicação.
                wall_map[{prog_byte_addr, 3'b000} +: 8] <= prog_byte_data;

                if (prog_cnt == 8'd127) begin
                    prog_cnt  <= 8'd0;
                    map_ready <= 1'b1;
                end else begin
                    prog_cnt  <= prog_cnt + 8'd1;
                    map_ready <= 1'b0;
                end
            end
        end
    end

    // ── Lógica combinacional do ambiente ──────────────────────────────────
    wire [4:0] row = state[9:5];
    wire [4:0] col = state[4:0];

    // Consulta mapa com índice derivado de (row, col)
    // is_blocked é função do wall_map registrado (não parâmetro)
    function is_blocked;
        input [4:0] r, c;
        begin
            is_blocked = wall_map[r * 32 + c];
        end
    endfunction

    reg [4:0] nrow, ncol;

    always @(*) begin
        nrow   = row;
        ncol   = col;
        reward = R_STEP;
        done   = 1'b0;

        case (action)
            3'd0: begin // CIMA
                if (row == 5'b00000 || is_blocked(row - 5'd1, col)) reward = R_WALL;
                else nrow = row - 5'd1;
            end
            3'd1: begin // BAIXO
                if (row == 5'b11111 || is_blocked(row + 5'd1, col)) reward = R_WALL;
                else nrow = row + 5'd1;
            end
            3'd2: begin // ESQUERDA
                if (col == 5'b00000 || is_blocked(row, col - 5'd1)) reward = R_WALL;
                else ncol = col - 5'd1;
            end
            3'd3: begin // DIREITA
                if (col == 5'b11111 || is_blocked(row, col + 5'd1)) reward = R_WALL;
                else ncol = col + 5'd1;
            end
            3'd4:    reward = R_STOP;
            default: reward = R_STEP;
        endcase

        next_state = {nrow, ncol};

        if (next_state == GOAL_STATE) begin
            reward = R_GOAL;
            done   = 1'b1;
        end
    end

endmodule