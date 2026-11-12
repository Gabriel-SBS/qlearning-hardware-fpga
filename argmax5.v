// =============================================================================
// argmax5.v — Seletor de Maior Q-Value entre 5 Ações (Combinacional)
// =============================================================================
// Entradas : cinco Q-values de 16 bits com sinal
// Saídas   : índice da ação com maior Q-value (0..4) e o valor máximo
//
// Em caso de empate, a ação de menor índice é preferida (prioridade estática).
// =============================================================================

module argmax5 (
    input  wire signed [15:0] q0, q1, q2, q3, q4,  // Q(s,0..4)
    output reg  [2:0]          best_action,           // ação ótima
    output reg  signed [15:0]  max_q                  // valor máximo
);

    always @(*) begin
        best_action = 3'd0;
        max_q       = q0;

        if (q1 > max_q) begin best_action = 3'd1; max_q = q1; end
        if (q2 > max_q) begin best_action = 3'd2; max_q = q2; end
        if (q3 > max_q) begin best_action = 3'd3; max_q = q3; end
        if (q4 > max_q) begin best_action = 3'd4; max_q = q4; end
    end

endmodule
