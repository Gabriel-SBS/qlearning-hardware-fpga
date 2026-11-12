// =============================================================================
// q_table_ram5_tb.v — Testbench Individual: q_table_ram5 (5 bancos paralelos)
// =============================================================================
// Substitui o antigo q_table_ram_tb.v (que validava a RAM v1 de banco único,
// 5120x16, addr = state*5 + action). O design atual usa q_table_ram5.v:
// 5 RAMs de 1024x16 (uma por ação), todas endereçadas por 'state', com DUAS
// portas de leitura assíncronas (addr / addr_n) e escrita síncrona em um
// único banco selecionado por 'wr_action'.
//
// Verifica:
//   1. Inicialização: todas as posições de TODOS os bancos começam em -16
//      (Q4 = -1.0), visíveis pelas duas portas de leitura
//   2. Leitura assíncrona (combinacional, latência 0) nas duas portas
//   3. Escrita síncrona (efeito só após a borda de subida do clk)
//   4. Seletividade de banco: a escrita atinge SOMENTE o banco de wr_action;
//      os outros 4 bancos no mesmo endereço permanecem intactos
//   5. we=0 não altera o conteúdo da memória
//   6. Independência das portas: addr_n lê um endereço diferente de addr no
//      mesmo ciclo, inclusive durante uma escrita via addr
//   7. Endereços extremos (0 e 1023) e valores extremos com sinal
//      (+32767 / -32768)
//   8. wr_action inválido (5..7) não escreve em banco algum
//
// Compilação (Icarus):
//   iverilog -g2012 -o sim_ram5 q_table_ram5_tb.v q_table_ram5.v
//   vvp sim_ram5
// =============================================================================
`timescale 1ns/1ns

module q_table_ram5_tb;

    reg         clk;
    reg         we;
    reg  [9:0]  addr;      // porta de leitura/escrita — Q(addr, 0..4)
    reg  [9:0]  addr_n;    // 2a porta de leitura      — Q(addr_n, 0..4)
    reg  [2:0]  wr_action; // banco que recebe a escrita
    reg  signed [15:0] din;

    wire signed [15:0] q0, q1, q2, q3, q4;
    wire signed [15:0] qn0, qn1, qn2, qn3, qn4;

    q_table_ram5 dut (
        .clk(clk), .we(we),
        .addr(addr), .addr_n(addr_n),
        .wr_action(wr_action), .din(din),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4),
        .qn0(qn0), .qn1(qn1), .qn2(qn2), .qn3(qn3), .qn4(qn4)
    );

    integer pass = 0;
    integer fail = 0;

    // Gera clock de 10ns de período
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Seleção de um Q-value da porta principal por índice de ação
    function signed [15:0] q_sel;
        input [2:0] a;
        begin
            case (a)
                3'd0: q_sel = q0;
                3'd1: q_sel = q1;
                3'd2: q_sel = q2;
                3'd3: q_sel = q3;
                default: q_sel = q4;
            endcase
        end
    endfunction

    task check_val;
        input signed [15:0] got;
        input signed [15:0] exp_val;
        input [511:0] test_id;
        begin
            if (got === exp_val) begin
                $display("  [PASS] %0s: %0d (esperado %0d)", test_id, got, exp_val);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: %0d (esperado %0d)", test_id, got, exp_val);
                fail = fail + 1;
            end
        end
    endtask

    integer k;

    initial begin
        $display("-----------------------------------------------");
        $display("  TB_Q_TABLE_RAM5 - Inicio dos testes");
        $display("-----------------------------------------------");

        we        = 1'b0;
        din       = 16'sd0;
        addr      = 10'd0;
        addr_n    = 10'd0;
        wr_action = 3'd0;

        // ---------------------------------------------------------------
        // Teste 1: Inicializacao -> todos os bancos em -16 (-1.0 em Q4),
        // nas duas portas de leitura e em enderecos extremos
        // ---------------------------------------------------------------
        addr   = 10'd0;
        addr_n = 10'd1023;
        #1;
        check_val(q0,  -16'sd16, "Teste 1a: init q0  @ addr 0");
        check_val(q2,  -16'sd16, "Teste 1b: init q2  @ addr 0");
        check_val(q4,  -16'sd16, "Teste 1c: init q4  @ addr 0");
        check_val(qn0, -16'sd16, "Teste 1d: init qn0 @ addr_n 1023");
        check_val(qn3, -16'sd16, "Teste 1e: init qn3 @ addr_n 1023");

        addr = 10'd512; // estado intermediario
        #1; check_val(q1, -16'sd16, "Teste 1f: init q1 @ addr 512");

        // ---------------------------------------------------------------
        // Teste 2: Escrita sincrona no banco 0, addr 0
        // ---------------------------------------------------------------
        @(negedge clk);
        addr      = 10'd0;
        din       = 16'sd1234;
        wr_action = 3'd0;
        we        = 1'b1;

        // Antes da borda de subida, q0 ainda deve refletir o valor antigo
        #1; check_val(q0, -16'sd16, "Teste 2a: q0 antes do clk (ainda valor antigo)");

        @(posedge clk); // efetua a escrita
        #1;
        check_val(q0, 16'sd1234, "Teste 2b: q0 apos escrita (leitura assincrona)");

        @(negedge clk);
        we = 1'b0;
        #1; check_val(q0, 16'sd1234, "Teste 2c: q0 permanece apos we=0");

        // ---------------------------------------------------------------
        // Teste 3: Seletividade de banco — os outros 4 bancos no MESMO
        // endereco continuam com o valor inicial
        // ---------------------------------------------------------------
        check_val(q1, -16'sd16, "Teste 3a: banco 1 intacto @ addr 0");
        check_val(q2, -16'sd16, "Teste 3b: banco 2 intacto @ addr 0");
        check_val(q3, -16'sd16, "Teste 3c: banco 3 intacto @ addr 0");
        check_val(q4, -16'sd16, "Teste 3d: banco 4 intacto @ addr 0");

        // ---------------------------------------------------------------
        // Teste 4: we=0 nao altera a memoria
        // ---------------------------------------------------------------
        @(negedge clk);
        din = 16'sd9999;
        we  = 1'b0;
        @(posedge clk);
        #1; check_val(q0, 16'sd1234, "Teste 4: escrita ignorada quando we=0");

        // ---------------------------------------------------------------
        // Teste 5: Escrita em cada um dos 5 bancos (addr 100), leitura
        // simultanea de todos no mesmo ciclo
        // ---------------------------------------------------------------
        addr = 10'd100;
        for (k = 0; k < 5; k = k + 1) begin
            @(negedge clk);
            wr_action = k[2:0];
            din       = 16'sd100 + k[15:0]; // 100,101,102,103,104
            we        = 1'b1;
            @(posedge clk);
        end
        @(negedge clk);
        we = 1'b0;
        #1;
        check_val(q0, 16'sd100, "Teste 5a: banco 0 = 100 @ addr 100");
        check_val(q1, 16'sd101, "Teste 5b: banco 1 = 101 @ addr 100");
        check_val(q2, 16'sd102, "Teste 5c: banco 2 = 102 @ addr 100");
        check_val(q3, 16'sd103, "Teste 5d: banco 3 = 103 @ addr 100");
        check_val(q4, 16'sd104, "Teste 5e: banco 4 = 104 @ addr 100");

        // ---------------------------------------------------------------
        // Teste 6: Independencia das portas — addr_n le outro endereco no
        // mesmo ciclo, inclusive durante uma escrita via addr
        // ---------------------------------------------------------------
        @(negedge clk);
        addr      = 10'd200;
        addr_n    = 10'd100;   // enxerga os valores do Teste 5
        din       = -16'sd500;
        wr_action = 3'd2;
        we        = 1'b1;
        #1;
        check_val(qn2, 16'sd102, "Teste 6a: qn2 le addr 100 durante escrita em addr 200");
        @(posedge clk);
        #1;
        check_val(q2,  -16'sd500, "Teste 6b: q2 = -500 @ addr 200 apos escrita");
        check_val(qn2, 16'sd102,  "Teste 6c: qn2 @ addr 100 inalterado");
        @(negedge clk);
        we = 1'b0;

        // ---------------------------------------------------------------
        // Teste 7: Enderecos e valores extremos
        // ---------------------------------------------------------------
        @(negedge clk);
        addr      = 10'd1023;
        din       = 16'sd32767;
        wr_action = 3'd4;
        we        = 1'b1;
        @(posedge clk);
        #1; check_val(q4, 16'sd32767, "Teste 7a: q4 = 32767 @ addr 1023");

        @(negedge clk);
        addr      = 10'd0;
        din       = -16'sd32768;
        wr_action = 3'd1;
        we        = 1'b1;
        @(posedge clk);
        #1; check_val(q1, -16'sd32768, "Teste 7b: q1 = -32768 @ addr 0");

        @(negedge clk);
        we   = 1'b0;
        addr = 10'd100;
        #1; check_val(q0, 16'sd100, "Teste 7c: addr 100 preservado apos escritas");

        // ---------------------------------------------------------------
        // Teste 8: wr_action invalido (5..7) nao escreve em banco algum
        // ---------------------------------------------------------------
        @(negedge clk);
        addr      = 10'd100;
        din       = 16'sd7777;
        wr_action = 3'd6;
        we        = 1'b1;
        @(posedge clk);
        @(negedge clk);
        we = 1'b0;
        #1;
        check_val(q0, 16'sd100, "Teste 8a: banco 0 intacto com wr_action=6");
        check_val(q4, 16'sd104, "Teste 8b: banco 4 intacto com wr_action=6");

        $display("-----------------------------------------------");
        $display("  RESULTADO: %0d PASS / %0d FAIL", pass, fail);
        $display("-----------------------------------------------");
        if (fail == 0)
            $display("q_table_ram5 funcionando.");
        else
            $display("Falhas no q_table_ram5!");
        $finish;
    end

endmodule
