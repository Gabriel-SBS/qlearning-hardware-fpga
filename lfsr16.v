// =============================================================================
// lfsr16.v — Gerador de Números Pseudo-Aleatórios (PRNG) utilizando um
//            Registrador de Deslocamento com Realimentação Linear (LFSR) de 16 bits.
// =============================================================================
// Polinômio Primitivo : x^16 + x^14 + x^13 + x^11 + 1
// Período Máximo      : 65.535 ciclos (2^16 - 1, iterando por todos os estados não nulos)
// Semente (Seed)      : 16'h5A5A (escolhida para garantir boa distribuição inicial)
// 
// Aplicação (Exploração ε-greedy no Q-Learning):
//   - rand_out  (Bits [7:0])  : Define a ação aleatória. A seleção de 3 bits é avaliada
//                               diretamente no módulo seguinte (garantindo valor < 5).
//   - rand_high (Bits [15:8]) : Fornece um byte para comparação direta com a taxa
//                               de exploração (epsilon). Define se o agente irá 
//                               explorar (random) ou aproveitar (argmax).
// =============================================================================

module lfsr16 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       en,
    output wire [7:0] rand_out,
    output wire [7:0] rand_high   // bits [15:8] para maior entropia
);
    reg [15:0] lfsr_reg;

    wire feedback = lfsr_reg[15] ^ lfsr_reg[13] ^ lfsr_reg[12] ^ lfsr_reg[10];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lfsr_reg <= 16'h5A5A; // Seed com boa distribuição para exploração inicial
        else if (en)
            lfsr_reg <= {lfsr_reg[14:0], feedback};
    end

    assign rand_out  = lfsr_reg[7:0];
    assign rand_high = lfsr_reg[15:8];

endmodule