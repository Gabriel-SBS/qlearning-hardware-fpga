// =============================================================================
// ula_bellman.v — ULA Bellman com ponto fixo Q4 (escala ×16)
// =============================================================================
// Problema anterior: inteiros puros truncavam o sinal de recompensa a zero
// após ~30 passos da meta. Com gamma=0.9375 e caminho mínimo de 62 passos,
// o sinal nunca chegava ao estado inicial.
//
// Solução: representação Q4 (4 bits fracionários, escala ×16):
//   • Q-values armazenados como inteiro × 16
//   • Reward é escalado (×16) antes de entrar na equação
//   • Shifts para alpha e gamma funcionam identicamente
//   • Q máximo teórico: 120/(1-0.9375)×16 = 1920×16 = 30720 < 32767
//
// Parâmetros:
//   α = 0.5    -> shift direito de 1 bit
//   γ = 0.9375 -> x − (x >>> 4)
// =============================================================================

module ula_bellman (
    input  wire signed [15:0] q_old,       // Q(s,a) em Q4 (valor × 16)
    input  wire signed [7:0]  reward,      // recompensa R (inteiro, sem escala)
    input  wire signed [15:0] max_q_next,  // max_a' Q(s',a') em Q4
    output wire signed [15:0] q_new        // Q(s,a) atualizado em Q4
);

    // Escala reward para Q4: R_q4 = R × 16 = R << 4
    wire signed [15:0] reward_q4 = {{4{reward[7]}}, reward, 4'b0};

    // γ · max_q_next   onde γ = 0.9375 = 1 − 1/16
    wire signed [15:0] gamma_maxq = max_q_next - (max_q_next >>> 4);

    // TD-error = R_q4 + γ·maxQ_next − Q_old   (tudo em Q4)
    wire signed [15:0] td_error = reward_q4 + gamma_maxq - q_old;

    // Q_new = Q_old + α·td_error   onde α = 0.5 → shift direito 1 bit
    assign q_new = q_old + (td_error >>> 1);

endmodule