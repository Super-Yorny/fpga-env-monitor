// ============================================================================
// aht20_driver.v —— AHT20/AHT10 温湿度传感器驱动（I2C 地址 0x38）【正式版】
//
// 流程：上电等 100ms → 初始化(0xE1,0x08,0x00) → 每 1000ms 一轮：
//       触发测量(0xAC,0x33,0x00) → 等 200ms → 读满 7 字节 → 换算输出
//
// 字节布局：d0=状态  d1,d2,d3[7:4]=湿度20bit  d3[3:0],d4,d5=温度20bit  d6=CRC
//   湿度 %RH×10 = raw_h × 1000 >> 20
//   温度 ℃×10  = raw_t × 2000 >> 20 − 500   （12bit 补码，可负）
//
// ---------------------------------------------------------------------------
// 调试中确认的关键点（详见 调试记录.md）：
//   1. 初始化命令用 0xE1 0x08 0x00（AHT10 风格）。实测这块丝印 AHT20 的模块
//      只有这条命令能真正让芯片进入校准完成状态；发 AHT20 的 0xBE 时，
//      芯片永远卡在 busy，导致所有读地址都被 NAK。
//   2. 触发测量后必须等 ≥200ms 再读。手册标称 75~80ms 是典型值，实测 80ms
//      时芯片仍 busy 会 NAK 读地址，读回全 1。
//   3. 湿度换算必须用 32 位中间量。若直接写
//         humi_x10 <= (raw_h * 11'd1000) >> 20;
//      Verilog 会按 20 位计算乘积（左值 11 位、操作数 20 位 → 取较大者），
//      raw_h×1000 一旦超过 2^20 就溢出回绕，再右移 20 位恒等于 0，
//      表现为「湿度永远是 0%」。
//   4. wait_ms 必须 ≥16 位。若写成 reg [6:0]（最大 127）却与常量 999 比较，
//      条件永不成立，状态机会永久卡在 P_IDLE，读数永远停在上电初值。
// ============================================================================
module aht20_driver #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer MEAS_PERIOD_MS = 1000
)(
    input  wire        clk,
    input  wire        rst_n,

    // I2C 三态由顶层负责，本模块只给下拉使能与回读输入（开漏：只能拉低）
    output wire        scl_oe,
    output wire        sda_oe,
    input  wire        sda_i,

    output reg         dat_valid,
    output reg  [11:0] temp_x10,   // 温度×10，12bit 二进制补码
    output reg  [10:0] humi_x10    // 湿度×10，0~1000
);
    localparam [6:0] AHT_ADDR = 7'h38;

    localparam integer MS_CNT = CLK_FREQ / 1000;

    reg [15:0] ms_cnt;
    reg        tick_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    ms_cnt <= 16'd0;
        else if (ms_cnt >= MS_CNT-1)   ms_cnt <= 16'd0;
        else                           ms_cnt <= ms_cnt + 1'b1;
    end
    always @(posedge clk) tick_ms <= (ms_cnt == MS_CNT-1);

    localparam P_POR    = 4'd0,
               P_INIT   = 4'd1,
               P_IDLE   = 4'd2,
               P_TRIG   = 4'd3,
               P_WAIT   = 4'd4,
               P_READ   = 4'd5,
               P_CALC   = 4'd6;

    reg [3:0]  state;
    reg [15:0] wait_ms;      // 必须够宽，能数到 MEAS_PERIOD_MS(1000)
    reg [3:0]  seq;

    reg        c_start, c_stop, c_wr, c_rd, mack;
    reg [6:0]  addr;
    reg        rw;
    reg [7:0]  wdata;
    wire       done;
    wire       ack_err;
    wire [7:0] rdata;

    i2c_master #(.CLK_FREQ(CLK_FREQ), .I2C_FREQ(25_000)) u_i2c (
        .clk(clk), .rst_n(rst_n),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_i(sda_i),
        .cmd_start(c_start), .cmd_stop(c_stop), .cmd_wr(c_wr), .cmd_rd(c_rd),
        .mack_en(mack), .dev_addr(addr), .dev_rw(rw), .wr_data(wdata),
        .rd_data(rdata), .done(done), .ack_err(ack_err)
    );

    reg [7:0] d0, d1, d2, d3, d4, d5, d6;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= P_POR; wait_ms <= 16'd0; seq <= 4'd0;
            c_start <= 0; c_stop <= 0; c_wr <= 0; c_rd <= 0;
            mack <= 0; addr <= AHT_ADDR; rw <= 0; wdata <= 0;
            dat_valid <= 0; temp_x10 <= 12'd0; humi_x10 <= 11'd0;
            d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0; d4 <= 0; d5 <= 0; d6 <= 0;
        end else begin
            // 命令为单拍脉冲：done 之后自动清除
            if (done) begin c_start <= 0; c_wr <= 0; c_rd <= 0; c_stop <= 0; end
            dat_valid <= 1'b0;
            case (state)
            // ---- 上电稳定 100ms ----
            P_POR: if (tick_ms) begin
                wait_ms <= wait_ms + 1'b1;
                if (wait_ms == 16'd99) begin
                    wait_ms <= 16'd0; seq <= 4'd0; state <= P_INIT;
                end
            end
            // ---- 初始化：0xE1 0x08 0x00（同一事务内连续发送）----
            P_INIT: begin
                case (seq)
                0: begin addr <= AHT_ADDR; rw <= 1'b0; c_start <= 1'b1; seq <= 4'd1; end
                1: if (done) begin c_wr <= 1'b1; wdata <= 8'hE1; seq <= 4'd2; end
                2: if (done) begin c_wr <= 1'b1; wdata <= 8'h08; seq <= 4'd3; end
                3: if (done) begin c_wr <= 1'b1; wdata <= 8'h00; seq <= 4'd4; end
                4: if (done) begin c_stop <= 1'b1; seq <= 4'd5; end
                5: if (done) begin seq <= 4'd0; wait_ms <= 16'd0; state <= P_IDLE; end
                default: seq <= 4'd0;
                endcase
            end
            // ---- 周期等待 ----
            P_IDLE: if (tick_ms) begin
                wait_ms <= wait_ms + 1'b1;
                if (wait_ms >= (MEAS_PERIOD_MS - 1)) begin
                    wait_ms <= 16'd0; seq <= 4'd0; state <= P_TRIG;
                end
            end
            // ---- 触发测量：0xAC 0x33 0x00 ----
            P_TRIG: begin
                case (seq)
                0: begin rw <= 1'b0; c_start <= 1'b1; seq <= 4'd1; end
                1: if (done) begin c_wr <= 1'b1; wdata <= 8'hAC; seq <= 4'd2; end
                2: if (done) begin c_wr <= 1'b1; wdata <= 8'h33; seq <= 4'd3; end
                3: if (done) begin c_wr <= 1'b1; wdata <= 8'h00; seq <= 4'd4; end
                4: if (done) begin c_stop <= 1'b1; seq <= 4'd5; end
                5: if (done) begin wait_ms <= 16'd0; state <= P_WAIT; end
                default: seq <= 4'd0;
                endcase
            end
            // ---- 等待测量完成：200ms ----
            P_WAIT: if (tick_ms) begin
                wait_ms <= wait_ms + 1'b1;
                if (wait_ms == 16'd200) begin
                    wait_ms <= 16'd0; seq <= 4'd0; state <= P_READ;
                end
            end
            // ---- 读满 7 字节：d0 状态 / d1~d5 数据 / d6 CRC ----
            P_READ: begin
                case (seq)
                0:  begin rw <= 1'b1; c_start <= 1'b1; seq <= 4'd1; end
                1:  if (done) begin c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd2; end
                2:  if (done) begin d0 <= rdata; c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd3; end
                3:  if (done) begin d1 <= rdata; c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd4; end
                4:  if (done) begin d2 <= rdata; c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd5; end
                5:  if (done) begin d3 <= rdata; c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd6; end
                6:  if (done) begin d4 <= rdata; c_rd <= 1'b1; mack <= 1'b1; seq <= 4'd7; end
                7:  if (done) begin d5 <= rdata; c_rd <= 1'b1; mack <= 1'b0; seq <= 4'd8; end
                8:  if (done) begin d6 <= rdata; c_stop <= 1'b1; seq <= 4'd9; end
                9:  if (done) begin seq <= 4'd0; state <= P_CALC; end
                default: seq <= 4'd0;
                endcase
            end
            // ---- 换算输出 ----
            P_CALC: begin
                humi_x10  <= h_num[30:20];   // 32 位乘积，避免 20 位溢出
                temp_x10  <= t_calc;
                dat_valid <= 1'b1;
                state     <= P_IDLE;
                wait_ms   <= 16'd0;
            end
            default: state <= P_POR;
            endcase
        end
    end

    // 湿度原始 20bit：{d1, d2, d3[7:4]}；必须用 32 位中间量
    wire [19:0] raw_h = {d1, d2, d3[7:4]};
    wire [31:0] h_num = raw_h * 20'd1000;
    // 温度原始 20bit：{d3[3:0], d4, d5}
    wire [19:0] raw_t  = {d3[3:0], d4, d5};
    wire [31:0] t_num  = raw_t * 12'd2000;
    wire [11:0] t_calc = t_num[31:20] - 12'd500;

    // ack_err 正式版不再输出，保留端口避免综合告警
    wire unused_ack = ack_err;

endmodule
