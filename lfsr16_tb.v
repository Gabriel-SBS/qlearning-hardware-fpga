// =============================================================================
// lfsr16_tb - Testbench de Validação para Gerador Aleatório LFSR de 16 bits
// =============================================================================
// Objetivo da Simulação:
//   Validar o comportamento dinâmico e a geração de pseudo-aleatoriedade do
//   módulo 'lfsr16'. O testbench analisa a divisão do registrador de 16 bits 
//   em duas saídas de 8 bits (rand_out e rand_high) sob as seguintes condições:
//
//     1. Inicialização/Reset : Aplicação de reset síncrono/assíncrono ativo em 
//                              nível lógico baixo (rst_n) para carregar o seed.
//     2. Habilitação (Enable): Verificação da retenção de estado quando 'en' 
//                              está em '0' e avanço da sequência quando em '1'.
//     3. Longo Curso (Loop)  : Execução de 500 ciclos de clock para observar a 
//                              evolução e a robustez da sequência pseudo-aleatória.
//     4. Reset em Operação   : Forçamento de um reset no final do ciclo (i=499) 
//                              para garantir a capacidade de reinicialização.
//
// Detalhes de Temporização:
//   - Período do Clock : 10 unidades de tempo (#5 em alto, #5 em baixo).
//   - Monitoramento    : Exibição inicial dos sinais via $display.
// =============================================================================

module lfsr16_tb;
    integer i;
    reg clk, rst, en;
    wire [7:0] rand_out, rand_high;
    
    lfsr16 dut(
        .clk(clk),
        .rst_n(rst),
        .en(en),
        .rand_out(rand_out),
        .rand_high(rand_high)
    );
    always begin
        #5; clk =~clk;
    end
    initial begin
        $display("En: %b| rand_out: %b | rand_high: %b", en, rand_out, rand_high);
        clk = 0;
        rst = 1;
        en = 0;
        
        #10; rst = 0;
        #5; rst = 1;
        #5; en = 1;
        #5;
        for (i = 0; i < 500; i = i +1) begin
            #10;
            if (i == 499)
                rst = 0;
        end
        #5;
        #20; 
    
        $finish;
    end
endmodule