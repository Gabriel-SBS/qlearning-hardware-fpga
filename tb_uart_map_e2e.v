// =============================================================================
// tb_uart_map_e2e.v — validação fim-a-fim do caminho que estava quebrado:
// 0xFF (soft reset) → 0xB0 + 128 bytes (mapa_top.npy) → verifica wall_map
// interno bit a bit → 0xA0 (start) → treina e conta metas via mon_valid/mon_goal.
// Regressão do bug 9 do README (índice truncado na porta de programação).
//
// Requer os dumps mapa_top_bytes.hex / mapa_top_expected.hex no diretório
// de execução (gerados a partir de mapa_top.npy).
//
// Compilação (Icarus):
//   iverilog -g2012 -o sim_e2e tb_uart_map_e2e.v qlearning_top_v2.v \
//            fsm_qlearning_v2.v q_table_ram5.v ula_bellman.v argmax5.v \
//            lfsr16.v maze_env.v uart_tx.v uart_rx.v
//   vvp sim_e2e
// =============================================================================
`timescale 1ns/1ns
module tb_uart_map_e2e;
    localparam CLKS_PER_BIT = 4;
    localparam BIT_NS       = CLKS_PER_BIT * 20;

    reg clk = 0, rst_n = 0, rx = 1;
    wire tx, training_done;
    wire [3:0] led_ep;

    qlearning_top_v2 #(.CLKS_PER_BIT(CLKS_PER_BIT)) dut (
        .clk(clk), .rst_n(rst_n),
        .i_Rx_Serial(rx), .o_Tx_Serial(tx),
        .led_ep(led_ep), .training_done_o(training_done)
    );

    always #10 clk = ~clk;   // 50 MHz

    task uart_send(input [7:0] b);
        integer k;
        begin
            rx = 0;  #BIT_NS;                       // start
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k];  #BIT_NS;                // dados LSB-first
            end
            rx = 1;  #BIT_NS;                       // stop
        end
    endtask

    reg [7:0]    mapa [0:127];
    reg [1023:0] esperado;
    integer i, metas, episodios, metas_ult500;

    // Conta fins de episódio direto na verdade interna da FSM
    always @(posedge clk) begin
        if (dut.fsm_inst.mon_valid) begin
            episodios = episodios + 1;
            if (dut.fsm_inst.mon_goal) begin
                metas = metas + 1;
                if (episodios > 1500) metas_ult500 = metas_ult500 + 1;
            end
        end
    end

    initial begin
        $readmemh("mapa_top_bytes.hex", mapa);
        $readmemh("mapa_top_expected.hex", esperado_mem);
        metas = 0; episodios = 0; metas_ult500 = 0;

        rst_n = 0; repeat (10) @(posedge clk); rst_n = 1; repeat (10) @(posedge clk);

        uart_send(8'hFF);  #(BIT_NS * 40);           // soft reset + folga
        uart_send(8'hB0);                            // header de mapa
        for (i = 0; i < 128; i = i + 1) uart_send(mapa[i]);
        repeat (20) @(posedge clk);

        // ── Checagem decisiva: o mapa interno é EXATAMENTE o enviado? ──
        if (dut.maze_inst.wall_map === esperado) begin
            $display("WALL_MAP OK: 1024 bits identicos ao mapa_top.npy enviado via UART");
        end else begin
            $display("WALL_MAP DIVERGENTE apos upload!");
            for (i = 0; i < 32; i = i + 1)
                if (dut.maze_inst.wall_map[i*32 +: 32] !== esperado[i*32 +: 32])
                    $display("  linha %0d difere: HW=%h ESP=%h",
                             i, dut.maze_inst.wall_map[i*32 +: 32], esperado[i*32 +: 32]);
            $finish;
        end
        if (dut.map_ready !== 1'b1) begin
            $display("ERRO: map_ready nao subiu"); $finish;
        end

        uart_send(8'hA0);                            // start

        // Treina ate 2000 episodios (ou training_done)
        wait (episodios >= 2000 || training_done);
        repeat (100) @(posedge clk);

        $display("=== E2E mapa_top.npy via UART ===");
        $display("Episodios: %0d | Metas: %0d (%0d%%) | Metas nos ultimos 500: %0d/500",
                 episodios, metas, (metas*100)/episodios, metas_ult500);
        if (metas_ult500 > 400)
            $display("RESULTADO: HW APRENDEU o mapa enviado — caminho UART->prog OK");
        else
            $display("RESULTADO: taxa baixa — investigar");
        $finish;
    end

    // $readmemh não escreve direto num vetor de 1024 bits; carrega via memória auxiliar
    reg [1023:0] esperado_mem [0:0];
    always @(*) esperado = esperado_mem[0];

    initial begin
        #300_000_000;
        $display("TIMEOUT do TB"); $finish;
    end
endmodule
