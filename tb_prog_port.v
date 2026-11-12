// =============================================================================
// tb_prog_port.v — verifica se a porta de programação do maze_env escreve
// os 128 bytes nos bits corretos. Regressão do bug 9 do README: o índice
// prog_byte_addr*7'd8 era truncado a 7 bits (part-select autodeterminado),
// corrigido para {prog_byte_addr, 3'b000}. Escreve um padrão único por
// endereço e lê de volta os 128 bytes via hierarquia.
//
// Compilação (Icarus):
//   iverilog -g2012 -o sim_prog tb_prog_port.v maze_env.v
//   vvp sim_prog
// =============================================================================
`timescale 1ns/1ns
module tb_prog_port;
    reg clk=0, rst_n=0, prog_we=0;
    reg  [6:0] prog_byte_addr=0;
    reg  [7:0] prog_byte_data=0;
    wire       map_ready;
    reg  [9:0] state=0; reg [2:0] action=0;
    wire [9:0] next_state; wire signed [7:0] reward; wire done;

    maze_env dut (.state(state), .action(action), .next_state(next_state),
                  .reward(reward), .done(done),
                  .clk(clk), .rst_n(rst_n),
                  .prog_we(prog_we), .prog_byte_addr(prog_byte_addr),
                  .prog_byte_data(prog_byte_data), .map_ready(map_ready));

    always #10 clk = ~clk;

    reg [1023:0] esperado;
    integer i, erros;
    initial begin
        rst_n = 0; repeat(3) @(posedge clk); rst_n = 1; @(posedge clk);

        // Programa byte i = i (padrão único por endereço)
        for (i = 0; i < 128; i = i + 1) begin
            @(posedge clk);
            prog_we        <= 1'b1;
            prog_byte_addr <= i[6:0];
            prog_byte_data <= i[7:0];
        end
        @(posedge clk); prog_we <= 1'b0; @(posedge clk);

        for (i = 0; i < 128; i = i + 1)
            esperado[i*8 +: 8] = i[7:0];

        erros = 0;
        for (i = 0; i < 128; i = i + 1)
            if (dut.wall_map[i*8 +: 8] !== i[7:0]) begin
                if (erros < 8)
                    $display("byte %0d: esperado %02h, lido %02h",
                             i, i[7:0], dut.wall_map[i*8 +: 8]);
                erros = erros + 1;
            end

        if (dut.wall_map === esperado)
            $display("PROG PORT OK: 128 bytes escritos nos lugares certos");
        else
            $display("PROG PORT QUEBRADA: %0d/128 bytes em posicao errada (map_ready=%b)",
                     erros, map_ready);
        $finish;
    end
endmodule
