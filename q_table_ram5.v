// =============================================================================
// q_table_ram5.v — Q-Table em 5 bancos paralelos (1 por ação)
// =============================================================================
// Em vez de uma única RAM 5120x16 endereçada por (state*5+action), usamos
// 5 RAMs independentes de 1024x16, uma por ação, todas endereçadas por
// 'state' (10 bits). Isso permite ler Q(s,0..4) — ou Q(s',0..4) — em UM
// único ciclo (5 leituras paralelas), em vez de 5 ciclos sequenciais.
//
// Escrita: sempre em UM banco só (o da ação tomada), selecionado por
// 'wr_action'. Leitura: combinacional (assíncrona), igual à RAM original.
// =============================================================================

module q_table_ram5 (
    input  wire        clk,
    input  wire        we,
    input  wire [9:0]  addr,          // estado atual (0..1023) — leitura E escrita
    input  wire [9:0]  addr_n,        // proximo estado (0..1023) — 2a porta de leitura
    input  wire [2:0]  wr_action,     // qual banco recebe a escrita (0..4)
    input  wire signed [15:0] din,

    output wire signed [15:0] q0, q1, q2, q3, q4,      // Q(addr,   0..4)
    output wire signed [15:0] qn0, qn1, qn2, qn3, qn4  // Q(addr_n, 0..4)
);

    reg signed [15:0] mem0 [0:1023];
    reg signed [15:0] mem1 [0:1023];
    reg signed [15:0] mem2 [0:1023];
    reg signed [15:0] mem3 [0:1023];
    reg signed [15:0] mem4 [0:1023];

    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            mem0[i] = -16'sd16;
            mem1[i] = -16'sd16;
            mem2[i] = -16'sd16;
            mem3[i] = -16'sd16;
            mem4[i] = -16'sd16;
        end
    end

    // Escrita síncrona: só o banco da ação tomada é atualizado
    always @(posedge clk) begin
        if (we) begin
            case (wr_action)
                3'd0: mem0[addr] <= din;
                3'd1: mem1[addr] <= din;
                3'd2: mem2[addr] <= din;
                3'd3: mem3[addr] <= din;
                3'd4: mem4[addr] <= din;
                default: ; // ações 5..7 não existem (apenas 5 ações)
            endcase
        end
    end

    // Leitura assíncrona — todas as 5 ações disponíveis no mesmo ciclo
    assign q0 = mem0[addr];
    assign q1 = mem1[addr];
    assign q2 = mem2[addr];
    assign q3 = mem3[addr];
    assign q4 = mem4[addr];

    assign qn0 = mem0[addr_n];
    assign qn1 = mem1[addr_n];
    assign qn2 = mem2[addr_n];
    assign qn3 = mem3[addr_n];
    assign qn4 = mem4[addr_n];

endmodule