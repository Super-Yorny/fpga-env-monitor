// ============================================================================
// oled_ssd1306.v —— 0.96 寸 128x64 OLED 驱动（SSD1306，I2C 地址 0x3C，页寻址）
// 内置 21x8 字符屏（6x8 字库），后台持续扫描刷新，字符 RAM 可随时更新
// ============================================================================
module oled_ssd1306 #(
    parameter integer CLK_FREQ = 50_000_000
)(
    input  wire       clk,
    input  wire       rst_n,
    // v7：不再使用 inout，三态由顶层负责（见 i2c_master.v 头部说明）
    output wire       scl_oe,
    output wire       sda_oe,
    input  wire       sda_i,

    input  wire       wr_en,          // 写字符：一拍脉冲
    input  wire [7:0] wr_addr,        // {row[2:0], col[4:0]}，8bit 才能覆盖 8x21=168 字节
    input  wire [7:0] wr_char,        // ASCII
    output wire       ready,          // 初始化完成
    output wire       i2c_nak         // 诊断：出现过 NAK（从机无应答）则持续为 1
);
    localparam [6:0] OLED_ADDR = 7'h3C;
    localparam ROWS = 8, COLS = 21;   // 6px*21 = 126 列有效，尾部 2 列空白

    // ---------------- 字符 RAM ----------------
    reg [7:0] ram [0:ROWS*COLS-1];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            for (i = 0; i < ROWS*COLS; i = i + 1) ram[i] <= 8'h20;
        else if (wr_en)
            ram[wr_addr] <= wr_char;
    end

    // ---------------- I2C 主机（OLED 用 100kHz，杜邦线长时更稳）----------------
    // 说明：400kHz 对开漏+模块 10k 上拉+20cm 杜邦线的组合偏高，上升沿可能来不及，
    //       表现为"有波形但读不到 ACK"。降到 100kHz 后余量充足，整帧刷新约 95ms，
    //       仍在 100ms 的刷新节拍内，不影响观感。
    reg        c_start, c_stop, c_wr, c_rd, mack;
    reg [6:0]  addr;
    reg        rw;
    reg [7:0]  wdata;
    wire       done;
    wire       ack_err;
    wire [7:0] rdata;

    i2c_master #(.CLK_FREQ(CLK_FREQ), .I2C_FREQ(100_000)) u_i2c (
        .clk(clk), .rst_n(rst_n),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .sda_i(sda_i),
        .cmd_start(c_start), .cmd_stop(c_stop), .cmd_wr(c_wr), .cmd_rd(c_rd),
        .mack_en(mack), .dev_addr(addr), .dev_rw(rw), .wr_data(wdata),
        .rd_data(rdata), .done(done), .ack_err(ack_err)
    );

    // ---------------- 1ms 计时 ----------------
    localparam integer MS_CNT = CLK_FREQ / 1000;
    localparam integer MSW = $clog2(MS_CNT + 2);
    reg [MSW-1:0] ms_cnt;
    reg tick_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      ms_cnt <= 0;
        else if (ms_cnt >= MS_CNT-1) ms_cnt <= 0;
        else             ms_cnt <= ms_cnt + 1'b1;
    end
    always @(posedge clk) tick_ms <= (ms_cnt == MS_CNT-1);

    // ---------------- 初始化命令表（逐字节，每字节前加控制字 0x00）--------
    // v3：双兼容序列，同时适配 SSD1306 与 SH1106（淘宝 0.96" 常以 SH1106 冒充 SSD1306）。
    //   两者 I2C 协议相同、都回 ACK，但开内部升压的命令不同：
    //     SSD1306 → 0x8D 0x14（电荷泵）；SH1106 → 0xAD 0x8B（DC-DC）
    //   同时发送两套：在对方芯片上会被当作未定义命令忽略，两边都能点亮。
    //   另注意：去掉了 0x20 0x02（SSD1306 寻址模式命令）——SH1106 无此命令，
    //   而 SSD1306 上电默认就是页寻址，去掉无影响。
    function [7:0] initcmd(input [4:0] idx);
        case (idx)
        5'd0:  initcmd = 8'hAE;  // 关显示
        5'd1:  initcmd = 8'hD5;  // 显示时钟分频
        5'd2:  initcmd = 8'h80;
        5'd3:  initcmd = 8'hA8;  // 复用率 64
        5'd4:  initcmd = 8'h3F;
        5'd5:  initcmd = 8'hD3;  // 显示偏移 0
        5'd6:  initcmd = 8'h00;
        5'd7:  initcmd = 8'h40;  // 起始行 0
        5'd8:  initcmd = 8'h8D;  // SSD1306 电荷泵使能
        5'd9:  initcmd = 8'h14;
        5'd10: initcmd = 8'hAD;  // SH1106 内部 DC-DC 使能
        5'd11: initcmd = 8'h8B;
        5'd12: initcmd = 8'hA1;  // 段重映射
        5'd13: initcmd = 8'hC8;  // COM 扫描方向
        5'd14: initcmd = 8'hDA;  // COM 引脚
        5'd15: initcmd = 8'h12;
        5'd16: initcmd = 8'h81;  // 对比度
        5'd17: initcmd = 8'hCF;
        5'd18: initcmd = 8'hD9;  // 预充电
        5'd19: initcmd = 8'hF1;
        5'd20: initcmd = 8'hDB;  // VCOMH
        5'd21: initcmd = 8'h40;
        5'd22: initcmd = 8'hA4;  // 恢复 RAM 内容
        5'd23: initcmd = 8'hA6;  // 正常显示
        5'd24: initcmd = 8'hAF;  // 开显示
        default: initcmd = 8'hAF;
        endcase
    endfunction
    localparam INIT_N = 25;

    // ---------------- 主状态机 ----------------
    localparam S_POR  = 4'd0,
               S_INIT = 4'd1,
               S_PG1  = 4'd2,   // 设置页与列：00 B0|p 00 10
               S_PG2  = 4'd3,   // 数据流：40 + 128 字节
               S_NXTP = 4'd4;

    reg [3:0]  state;
    reg [3:0]  st;
    reg [4:0]  idx;      // init 命令序号
    reg [2:0]  page;
    reg [4:0]  charidx;  // 0..21 (21 有效 + 尾空白)
    reg [2:0]  charcol;  // 0..5
    reg [1:0]  tail;
    reg [6:0]  wait_ms;

    // 字库（6x8，列编码，bit0 = 顶部像素；col 5 为字间距）
    function [7:0] font(input [7:0] code, input [2:0] col);
        if (col == 3'd5) font = 8'h00;
        else case (code)
            8'h20: font = 8'h00;
            8'h25: font = (col==0)?8'h23:(col==1)?8'h13:(col==2)?8'h08:(col==3)?8'h64:8'h62; // %
            8'h2D: font = 8'h08;                    // -
            8'h2E: font = (col==1)?8'h60:(col==2)?8'h60:8'h00;                                // .
            8'h2F: font = (col==0)?8'h20:(col==1)?8'h10:(col==2)?8'h08:(col==3)?8'h04:8'h02;  // /
            8'h30: font = (col==0)?8'h3E:(col==1)?8'h51:(col==2)?8'h49:(col==3)?8'h45:8'h3E;  // 0
            8'h31: font = (col==1)?8'h42:(col==2)?8'h7F:8'h00;                                // 1
            8'h32: font = (col==0)?8'h42:(col==1)?8'h61:(col==2)?8'h51:(col==3)?8'h49:8'h46;  // 2
            8'h33: font = (col==0)?8'h21:(col==1)?8'h41:(col==2)?8'h45:(col==3)?8'h4B:8'h31;  // 3
            8'h34: font = (col==0)?8'h18:(col==1)?8'h14:(col==2)?8'h12:(col==3)?8'h7F:8'h10;  // 4
            8'h35: font = (col==0)?8'h27:(col==1)?8'h45:(col==2)?8'h45:(col==3)?8'h45:8'h39;  // 5
            8'h36: font = (col==0)?8'h3C:(col==1)?8'h4A:(col==2)?8'h49:(col==3)?8'h49:8'h30;  // 6
            8'h37: font = (col==0)?8'h01:(col==1)?8'h71:(col==2)?8'h09:(col==3)?8'h05:8'h03;  // 7
            8'h38: font = (col==0)?8'h36:(col==1)?8'h49:(col==2)?8'h49:(col==3)?8'h49:8'h36;  // 8
            8'h39: font = (col==0)?8'h06:(col==1)?8'h49:(col==2)?8'h49:(col==3)?8'h29:8'h1E;  // 9
            8'h3A: font = (col==1)?8'h36:(col==2)?8'h36:8'h00;                                // :
            8'h41: font = (col==0)?8'h7C:(col==1)?8'h12:(col==2)?8'h11:(col==3)?8'h12:8'h7C;  // A
            8'h42: font = (col==0)?8'h7F:(col==1)?8'h49:(col==2)?8'h49:(col==3)?8'h49:8'h36;  // B
            8'h43: font = (col==0)?8'h3E:(col==1)?8'h41:(col==2)?8'h41:(col==3)?8'h41:8'h22;  // C
            8'h44: font = (col==0)?8'h7F:(col==1)?8'h41:(col==2)?8'h41:(col==3)?8'h22:8'h1C;  // D
            8'h45: font = (col==0)?8'h7F:(col==1)?8'h49:(col==2)?8'h49:(col==3)?8'h49:8'h41;  // E
            8'h46: font = (col==0)?8'h7F:(col==1)?8'h09:(col==2)?8'h09:(col==3)?8'h09:8'h01;  // F
            8'h47: font = (col==0)?8'h3E:(col==1)?8'h41:(col==2)?8'h49:(col==3)?8'h49:8'h7A;  // G
            8'h48: font = (col==0)?8'h7F:(col==1)?8'h08:(col==2)?8'h08:(col==3)?8'h08:8'h7F;  // H
            8'h49: font = (col==1)?8'h41:(col==2)?8'h7F:8'h00;                                // I
            8'h4B: font = (col==0)?8'h7F:(col==1)?8'h08:(col==2)?8'h14:(col==3)?8'h22:8'h41;  // K
            8'h4C: font = (col==0)?8'h7F:8'h40;                                               // L
            8'h4D: font = (col==0)?8'h7F:(col==1)?8'h02:(col==2)?8'h0C:(col==3)?8'h02:8'h7F;  // M
            8'h4E: font = (col==0)?8'h7F:(col==1)?8'h04:(col==2)?8'h08:(col==3)?8'h10:8'h7F;  // N
            8'h4F: font = (col==0)?8'h3E:(col==1)?8'h41:(col==2)?8'h41:(col==3)?8'h41:8'h3E;  // O
            8'h50: font = (col==0)?8'h7F:(col==1)?8'h09:(col==2)?8'h09:(col==3)?8'h09:8'h06;  // P
            8'h52: font = (col==0)?8'h7F:(col==1)?8'h09:(col==2)?8'h19:(col==3)?8'h29:8'h46;  // R
            8'h53: font = (col==0)?8'h46:(col==1)?8'h49:(col==2)?8'h49:(col==3)?8'h49:8'h31;  // S
            8'h54: font = (col==2)?8'h7F:8'h01;                                               // T
            8'h55: font = (col==0)?8'h3F:(col==1)?8'h40:(col==2)?8'h40:(col==3)?8'h40:8'h3F;  // U
            8'h56: font = (col==0)?8'h1F:(col==1)?8'h20:(col==2)?8'h40:(col==3)?8'h20:8'h1F;  // V
            8'h57: font = (col==0)?8'h3F:(col==1)?8'h40:(col==2)?8'h38:(col==3)?8'h40:8'h3F;  // W
            8'h59: font = (col==0)?8'h07:(col==1)?8'h08:(col==2)?8'h70:(col==3)?8'h08:8'h07;  // Y
            default: font = 8'h00;
        endcase
    endfunction

    // 当前发送的数据字节：21 个字符 ×6 列 = 126，再加 2 列空白
    wire [7:0] fnt  = font(ram[page*21 + charidx], charcol);
    wire       tail_mode = (charidx == 5'd21);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_POR; st <= 0; idx <= 0; page <= 0;
            charidx <= 0; charcol <= 0; tail <= 0; wait_ms <= 0;
            c_start <= 0; c_stop <= 0; c_wr <= 0; c_rd <= 0;
            mack <= 0; addr <= OLED_ADDR; rw <= 0; wdata <= 0;
        end else begin
            if (done) begin c_start <= 0; c_wr <= 0; c_stop <= 0; end
            case (state)
            // ---- 上电稳定 100ms ----
            S_POR: if (tick_ms) begin
                wait_ms <= wait_ms + 1'b1;
                if (wait_ms == 7'd99) begin
                    wait_ms <= 0; st <= 0; idx <= 0; state <= S_INIT;
                end
            end
            // ---- 初始化：一个事务内连续发送 00 + 全部命令字节 ----
            // ⚠️⚠️ 必须放在同一个 I2C 事务里连续发送！
            //   SSD1306 有一批「命令 + 参数」的多字节命令：
            //     0x20 0x02(寻址模式) 0x81 0x7F(对比度) 0xA8 0x3F(复用率)
            //     0xD3 0x00(显示偏移) 0xD5 0x80(时钟) 0xD9 0xF1(预充电)
            //     0xDA 0x12(COM)      0xDB 0x40(VCOMH) 0x8D 0x14(电荷泵)
            //   参数字节必须紧跟命令字节、处于同一个控制字 0x00 之后。
            //   早期版本把每个字节拆成独立事务（START 78 00 cmd STOP），
            //   参数字节会被 SSD1306 当成「新命令」而丢弃 —— 其中
            //   0x8D 0x14（电荷泵使能）一旦丢失，片内电荷泵保持关闭，
            //   面板得不到 ~7.5V 驱动电压，**屏幕完全黑屏**，
            //   但 I2C 通信看起来一切正常（有 ACK、地址对）。
            S_INIT: begin
                case (st)
                0: begin c_start <= 1'b1; st <= 1; end
                1: if (done) begin c_wr <= 1'b1; wdata <= 8'h00; st <= 2; end
                2: if (done) begin c_wr <= 1'b1; wdata <= initcmd(idx); st <= 3; end
                3: if (done) begin
                       if (idx == INIT_N-1) begin
                           c_stop <= 1'b1; st <= 4;                  // 最后一条：发 STOP
                       end else begin
                           idx   <= idx + 1'b1;
                           c_wr  <= 1'b1; wdata <= initcmd(idx + 1'b1); // 继续流式发送
                       end
                   end
                4: if (done) begin
                       st <= 0; idx <= 0; page <= 0; state <= S_PG1;
                   end
                default: st <= 0;
                endcase
            end
            // ---- 设置页地址与起始列 ----
            S_PG1: begin
                case (st)
                0: begin c_start <= 1'b1; st <= 1; end
                1: if (done) begin c_wr <= 1'b1; wdata <= 8'h00; st <= 2; end
                // ★ 修 bug：原写法 {3'b1011, page} 只有 6 位，零扩展后变成
                //   0x2C+page，不是合法的"设置页地址"命令(0xB0|page)，
                //   SSD1306 直接忽略 → 8 页数据全写进第 0 页。
                2: if (done) begin c_wr <= 1'b1; wdata <= 8'hB0 | page; st <= 3; end
                3: if (done) begin c_wr <= 1'b1; wdata <= 8'h00; st <= 4; end
                4: if (done) begin c_wr <= 1'b1; wdata <= 8'h10; st <= 5; end
                5: if (done) begin c_stop <= 1'b1; st <= 6; end
                6: if (done) begin
                       st <= 0; charidx <= 0; charcol <= 0; tail <= 0;
                       state <= S_PG2;
                   end
                default: st <= 0;
                endcase
            end
            // ---- 数据流：START 78W 40 + 128 字节 STOP ----
            S_PG2: begin
                case (st)
                0: begin c_start <= 1'b1; st <= 1; end
                1: if (done) begin c_wr <= 1'b1; wdata <= 8'h40; st <= 2; end
                2: if (done) begin
                       if (tail_mode) begin
                           c_wr <= 1'b1; wdata <= 8'h00;       // 行尾空白
                           if (tail == 2'd1) begin st <= 3; end
                           else tail <= tail + 2'd1;
                       end else begin
                           c_wr <= 1'b1; wdata <= fnt;
                           if (charcol == 3'd5) begin          // 含第6列字间距
                               charcol <= 0;
                               charidx <= charidx + 1'b1;
                           end else
                               charcol <= charcol + 1'b1;
                       end
                   end
                3: if (done) begin c_stop <= 1'b1; st <= 4; end
                4: if (done) begin st <= 0; state <= S_NXTP; end
                default: st <= 0;
                endcase
            end
            // ---- 下一页 ----
            S_NXTP: begin
                page <= page + 1'b1;
                charidx <= 0; charcol <= 0; tail <= 0; st <= 0;
                state <= S_PG1;
            end
            default: state <= S_POR;
            endcase
        end
    end

    assign ready = (state != S_POR) && (state != S_INIT);

    // ---------------- 诊断：锁存 NAK ----------------
    // SSD1306 对地址字节、命令字节、数据字节都应回 ACK。
    // 只要出现过一次 NAK，就说明总线上没有器件应答：
    //   接线断 / 没共地 / 模块无上拉 / I2C 地址不是 0x3C。
    reg nak_latch;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)       nak_latch <= 1'b0;
        else if (ack_err) nak_latch <= 1'b1;
    end
    assign i2c_nak = nak_latch;

endmodule
