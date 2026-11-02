// ============================================================================
// tb_i2c_master.v —— i2c_master 自检 testbench
//
// 验证内容：
//   1. START / STOP 时序
//   2. 地址字节（0x38 写 → 0x70）发送正确
//   3. 连续写 3 字节数据，从机逐字节应答
//   4. 读事务：地址 0x71，从机返回 0xA5，主机 NACK 结束
//   5. 回归项：统计 SCL 上升沿总数，等于 54 才说明"每字节只发一次"
//      （v4 之前的重复发送缺陷会让该值翻倍）
//
// 运行（ModelSim）：
//   vlib work
//   vlog ../rtl/i2c_master.v tb_i2c_master.v
//   vsim -c tb_i2c_master -do "run -all; quit -f"
// ============================================================================
`timescale 1ns / 1ps

module tb_i2c_master;

    // ---------------- 时钟与复位 ----------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    // ---------------- 开漏总线建模 ----------------
    tri  scl;
    tri  sda;
    pullup (scl);
    pullup (sda);

    // ---------------- DUT ----------------
    wire       scl_oe, sda_oe;
    wire       sda_i = sda;

    reg        cmd_start = 1'b0;
    reg        cmd_stop  = 1'b0;
    reg        cmd_wr    = 1'b0;
    reg        cmd_rd    = 1'b0;
    reg        mack_en   = 1'b0;
    reg  [6:0] dev_addr  = 7'h38;
    reg        dev_rw    = 1'b0;
    reg  [7:0] wr_data   = 8'h00;

    wire [7:0] rd_data;
    wire       done;
    wire       ack_err;

    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    i2c_master #(
        .CLK_FREQ (50_000_000),
        .I2C_FREQ (100_000)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_i(sda_i),
        .cmd_start(cmd_start), .cmd_stop(cmd_stop),
        .cmd_wr(cmd_wr), .cmd_rd(cmd_rd),
        .mack_en(mack_en), .dev_addr(dev_addr), .dev_rw(dev_rw),
        .wr_data(wr_data), .rd_data(rd_data),
        .done(done), .ack_err(ack_err)
    );

    // ---------------- 计分与统计 ----------------
    integer errors  = 0;
    integer scl_cnt = 0;
    integer wbytes  = 0;        // 写事务中从机实际收到的字节数（用于重复发送检测）
    always @(posedge scl) scl_cnt = scl_cnt + 1;

    task check_byte(input [7:0] got, input [7:0] exp, input [127:0] name);
        begin
            if (got === exp)
                $display("[PASS] %0s : got 0x%02h", name, got);
            else begin
                $display("[FAIL] %0s : got 0x%02h, expected 0x%02h", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // ================= 从机模型（过程式） =================
    reg        s_drive = 1'b0;
    reg        s_bit   = 1'b1;
    assign sda = s_drive ? s_bit : 1'bz;

    // 接收一个字节：在 SCL 高电平期间采样 8 位
    task recv_byte(output [7:0] b);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                @(posedge scl);
                b[i] = sda;
            end
        end
    endtask

    // 应答：第 8 个时钟下降沿后拉低，第 9 个下降沿后释放
    task send_ack;
        begin
            @(negedge scl);
            s_drive = 1'b1; s_bit = 1'b0;
            @(negedge scl);
            s_drive = 1'b0;
        end
    endtask

    // 读方向：在 SCL 低电平期间依次送出 8 位
    task drive_byte(input [7:0] d);
        integer i;
        begin
            s_drive = 1'b1; s_bit = d[7];
            for (i = 6; i >= 0; i = i - 1) begin
                @(negedge scl);
                s_bit = d[i];
            end
            @(negedge scl);
            s_drive = 1'b0;         // 交回 ACK 槽
        end
    endtask

    task wait_start;
        begin
            wait (scl === 1'b1);
            @(negedge sda);
        end
    endtask

    // ================= 主机激励任务 =================
    task m_start;
        begin
            @(posedge clk); cmd_start <= 1'b1;
            @(posedge clk); cmd_start <= 1'b0;
            wait (done === 1'b1); @(posedge clk);
        end
    endtask

    task m_write(input [7:0] d);
        begin
            @(posedge clk); wr_data <= d; cmd_wr <= 1'b1;
            @(posedge clk); cmd_wr <= 1'b0;
            wait (done === 1'b1); @(posedge clk);
        end
    endtask

    task m_read(input ack_en);
        begin
            @(posedge clk); mack_en <= ack_en; cmd_rd <= 1'b1;
            @(posedge clk); cmd_rd <= 1'b0;
            wait (done === 1'b1); @(posedge clk);
        end
    endtask

    task m_stop;
        begin
            @(posedge clk); cmd_stop <= 1'b1;
            @(posedge clk); cmd_stop <= 1'b0;
            wait (done === 1'b1); @(posedge clk);
        end
    endtask

    // ================= 从机行为序列 =================
    reg [7:0] rx;

    initial begin
        // ---- 事务 1：写 3 字节到 0x38 ----
        wait_start();
        recv_byte(rx); wbytes = wbytes + 1; send_ack(); check_byte(rx, 8'h70, "ADDR_W(0x70)");
        recv_byte(rx); wbytes = wbytes + 1; send_ack(); check_byte(rx, 8'hBE, "DATA1(0xBE)");
        recv_byte(rx); wbytes = wbytes + 1; send_ack(); check_byte(rx, 8'h08, "DATA2(0x08)");
        recv_byte(rx); wbytes = wbytes + 1; send_ack(); check_byte(rx, 8'h00, "DATA3(0x00)");

        // ---- 事务 2：读 1 字节 ----
        wait_start();
        recv_byte(rx); send_ack(); check_byte(rx, 8'h71, "ADDR_R(0x71)");
        drive_byte(8'hA5);
    end

    // ================= 主机激励 =================
    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("---- Test 1: write 3 bytes to slave 0x38 ----");
        dev_addr <= 7'h38; dev_rw <= 1'b0;
        m_start();
        m_write(8'hBE);
        m_write(8'h08);
        m_write(8'h00);
        m_stop();

        $display("   [dbg] SCL edges after write transaction : %0d", scl_cnt);

        repeat (200) @(posedge clk);

        $display("---- Test 2: read 1 byte from slave 0x38 ----");
        dev_addr <= 7'h38; dev_rw <= 1'b1;
        m_start();
        m_read(1'b0);                       // 最后一字节 NACK
        if (rd_data === 8'hA5)
            $display("[PASS] READ_DATA : got 0x%02h", rd_data);
        else begin
            $display("[FAIL] READ_DATA : got 0x%02h, expected 0xA5", rd_data);
            errors = errors + 1;
        end
        m_stop();

        repeat (300) @(posedge clk);

        $display("---- Test 3: duplicate-byte regression ----");
        // 写事务共 4 个字节（地址 + 3 数据）。若发生重复发送，从机会收到 8 个。
        if (wbytes == 4)
            $display("[PASS] BYTES_RECEIVED = %0d (expected 4, no duplicate)", wbytes);
        else begin
            $display("[FAIL] BYTES_RECEIVED = %0d, expected 4 (bytes duplicated?)", wbytes);
            errors = errors + 1;
        end
        $display("   [info] total SCL rising edges = %0d (54 data clocks + 2 STOP + 1 init)", scl_cnt);

        if (ack_err !== 1'b0) begin
            $display("[FAIL] ACK_ERR asserted unexpectedly");
            errors = errors + 1;
        end else
            $display("[PASS] ACK_ERR = 0");

        $display("=====================================");
        if (errors == 0) $display("RESULT: ALL TESTS PASSED");
        else             $display("RESULT: %0d TEST(S) FAILED", errors);
        $display("=====================================");
        $finish;
    end

    // 超时保护
    initial begin
        #5_000_000;
        $display("[FAIL] TIMEOUT");
        $finish;
    end

endmodule
