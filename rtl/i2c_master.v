// ============================================================================
// i2c_master.v —— 通用 I2C 主机（100kHz，开漏输出，三态由顶层负责）
//
// ★ v7 关键改动（2026-10-25）★
// ---------------------------------------------------------------------------
// 【移除 inout 端口】原版在模块内部写：
//       assign sda = sda_oe ? 1'b0 : 1'bz;
//       wire sda_in = sda;
//   实测现象：顶层能正确读到 SDA 高电平（电平监测显示 D11），但本模块
//   内部的 sda_in 读出恒为 0 —— 所有 I2C 读操作得到 0x00，写操作却正常。
//   怀疑 PDS 对「非顶层模块内部读 inout 端口」没有保留输入通路。
//
//   改法：本模块不再持有 inout，改为三个纯端口：
//       output scl_oe    SCL 下拉使能（1=拉低，0=释放由上拉拉高）
//       output sda_oe    SDA 下拉使能（1=拉低，0=释放由上拉拉高）
//       input  sda_i     SDA 引脚回读电平（由顶层直接读顶层 inout 得到）
//   顶层负责：
//       assign xxx_sda    = xxx_sda_oe ? 1'b0 : 1'bz;
//       assign xxx_sda_i  = xxx_sda;
//
// 历史修复（保留）：
//   S_GAP 状态 —— 防止调用方电平保持式命令被重复锁存导致每字节发两遍。
//   相位计数器每次操作清零 —— 保证从 phase0 起步，时序确定。
// ============================================================================
module i2c_master #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer I2C_FREQ = 100_000
)(
    input  wire       clk,
    input  wire       rst_n,

    output reg        scl_oe,      // SCL 下拉使能
    output reg        sda_oe,      // SDA 下拉使能
    input  wire       sda_i,       // SDA 回读电平

    input  wire       cmd_start,
    input  wire       cmd_stop,
    input  wire       cmd_wr,
    input  wire       cmd_rd,
    input  wire       mack_en,
    input  wire [6:0] dev_addr,
    input  wire       dev_rw,
    input  wire [7:0] wr_data,

    output reg  [7:0] rd_data,
    output reg        done,
    output reg        ack_err
);
    // 一个 I2C 位 = 4 个相位；DIV = 每相位时钟数
    localparam integer DIV  = CLK_FREQ / (I2C_FREQ * 4);
    localparam integer DIVW = (DIV <= 2) ? 2 : $clog2(DIV);
    localparam [DIVW-1:0] DIV_M1 = DIV - 1;

    reg [DIVW-1:0] divcnt;
    reg [1:0]      phase;
    wire qtick = (divcnt == DIV_M1);

    localparam S_IDLE  = 3'd0,
               S_START = 3'd1,
               S_BIT   = 3'd2,
               S_ACK   = 3'd3,
               S_STOP  = 3'd4,
               S_PULSE = 3'd5,
               S_GAP   = 3'd6;

    reg [2:0] state;
    reg [7:0] shifter;
    reg [2:0] bitcnt;
    reg       cur_rw;
    reg       mack;
    reg       ack;
    reg       is_addr;   // 当前字节是否为「地址字节」（v9 新增）

    // 新操作开始：相位与分频器强制回到 0，保证从 phase0 起步
    wire new_op = (state == S_IDLE) &&
                  (cmd_start | cmd_wr | cmd_rd | cmd_stop);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      divcnt <= 0;
        else if (new_op) divcnt <= 0;
        else if (qtick)  divcnt <= 0;
        else             divcnt <= divcnt + 1'b1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      phase <= 2'd0;
        else if (new_op) phase <= 2'd0;
        else if (qtick)  phase <= phase + 2'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; ack_err <= 1'b0;
            scl_oe <= 1'b0; sda_oe <= 1'b0;
            bitcnt <= 3'd0; rd_data <= 8'h00; ack <= 1'b1;
            shifter <= 8'h00; cur_rw <= 1'b0; mack <= 1'b0; is_addr <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
            // ---------------- 空闲 ----------------
            S_IDLE: begin
                sda_oe <= 1'b0;
                if (cmd_start) begin
                    shifter <= {dev_addr, dev_rw};
                    cur_rw  <= dev_rw;
                    bitcnt  <= 3'd0;
                    is_addr <= 1'b1;              // 下一个字节是地址
                    ack_err <= 1'b0;              // v13：每次探测前清零，便于总线扫描
                    state   <= S_START;
                end
                else if (cmd_wr) begin
                    shifter <= wr_data;
                    cur_rw  <= 1'b0;
                    bitcnt  <= 3'd0;
                    is_addr <= 1'b0;
                    state   <= S_BIT;
                end
                else if (cmd_rd) begin
                    mack    <= mack_en;
                    cur_rw  <= 1'b1;
                    bitcnt  <= 3'd0;
                    is_addr <= 1'b0;
                    state   <= S_BIT;
                end
                else if (cmd_stop) begin
                    state <= S_STOP;
                end
            end

            // ---------------- START：SCL 高时 SDA 下降 ----------------
            S_START: if (qtick) case (phase)
                2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end  // SCL 高、SDA 高
                2'd1: sda_oe <= 1'b1;                            // SCL 高时拉低 SDA = START
                2'd2: scl_oe <= 1'b1;                            // SCL 拉低
                2'd3: state  <= S_BIT;
                default: ;
            endcase

            // ---------------- 数据位 ----------------
            // ★ v21 根因修复 ★
            //   原写法：if (cur_rw) sda_oe <= 1'b0;  → 读方向一律释放 SDA。
            //   但「地址字节」无论读写都必须由主机发送！cur_rw 只能控制"数据字节"。
            //   旧代码在读事务时把地址字节发成了 0xFF（全 1），从机自然 NAK。
            //   正解：只有「读方向 + 非地址字节」才释放 SDA。
            S_BIT: if (qtick) case (phase)
                2'd0: begin
                    if (cur_rw && !is_addr) sda_oe <= 1'b0;       // 读数据位：释放
                    else                    sda_oe <= ~shifter[7];// 地址字节 / 写数据位：驱动
                end
                2'd1: scl_oe <= 1'b0;                         // SCL 上升
                2'd2: begin                                   // SCL 高电平期间采样
                    if (cur_rw && !is_addr) rd_data <= {rd_data[6:0], sda_i};
                end
                2'd3: begin                                   // SCL 下降
                    scl_oe  <= 1'b1;
                    if (!cur_rw || is_addr) shifter <= {shifter[6:0], 1'b0};  // 地址字节也要移
                    if (bitcnt == 3'd7) state <= S_ACK;
                    else                bitcnt <= bitcnt + 3'd1;
                end
                default: ;
            endcase

            // ---------------- 第 9 位：ACK / NACK ----------------
            // ★ v9 修复 A：地址字节的应答位只能由从机拉低，主机必须释放。
            //   原写法 `if (cur_rw) sda_oe <= mack;` 在读事务的地址阶段也会拉低，
            //   造成「即使从机不在也读回 0」的假 ACK，NAK 诊断完全失效。
            // ★ v9 修复 B：ack 存的是「收到 ACK」的正面标志（ack=1 表示 SDA 低），
            //   所以报错条件必须是 !ack，原写法写成 ack，极性反了。
            S_ACK: if (qtick) case (phase)
                2'd0: begin
                    if (cur_rw && !is_addr) sda_oe <= mack;  // 读数据字节：主机回应答
                    else                    sda_oe <= 1'b0;  // 地址 / 写字节：释放
                end
                2'd1: scl_oe <= 1'b0;
                2'd2: begin
                    if (!cur_rw || is_addr) ack <= ~sda_i;   // SDA 低 → ack=1 → 收到 ACK
                end
                2'd3: begin
                    scl_oe  <= 1'b1;
                    sda_oe  <= 1'b0;
                    if ((!cur_rw || is_addr) && !ack) ack_err <= 1'b1;  // 真 NAK 才报错
                    state   <= S_PULSE;
                    is_addr <= 1'b0;
                end
                default: ;
            endcase

            // ---------------- STOP ----------------
            S_STOP: if (qtick) case (phase)
                2'd0: sda_oe <= 1'b1;          // SDA 低
                2'd1: scl_oe <= 1'b0;          // 释放 SCL（升高）
                2'd2: sda_oe <= 1'b0;          // SDA 升高 = STOP
                2'd3: state  <= S_PULSE;
                default: ;
            endcase

            // ---------------- 完成脉冲 ----------------
            S_PULSE: begin
                done  <= 1'b1;
                state <= S_GAP;                // 不再直接回 S_IDLE
            end

            // ---------------- 空转一拍 ----------------
            // 调用方在这一拍里撤销旧命令、装好下一条命令和数据。
            S_GAP: begin
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
