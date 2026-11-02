// ============================================================================
// top_env_monitor.v —— 顶层：盘古 MES50HP 多点环境监测与超限告警【正式版】
//
//   50MHz → 2 路 AHT20 各占一条独立 I2C 总线并行采集 → 换算 → OLED 显示
//   LED1 = 1Hz 心跳；LED2 = 温度超限常亮
//
// ---------------------------------------------------------------------------
// 【引脚分配】J8 40pin 扩展口（手册表 3-12）
//   AHT20 #1  SDA → Pin15 (Y12)      SCL → Pin16 (W12)
//   OLED      SDA → Pin17 (U12)      SCL → Pin18 (T12)
//   AHT20 #2  SDA → Pin19 (U10)      SCL → Pin20 (T10)
//   所有器件  VIN → Pin39/40 (A3V3)  GND → Pin37/38
//
//   注意：AHT20 用的是 Pin15/16（原本给 OLED 的那一对），这是调试期间
//   做「引脚互换 A/B 测试」时改的，之后一直沿用并验证通过。改接线时
//   必须与 constraints/env_monitor.fdc 保持一致，否则对应的 I2C 不会动。
// ---------------------------------------------------------------------------
// 【I2C 三态】开漏总线，FPGA 只能拉低、不能输出高（高电平靠外部上拉）。
//   三态控制统一放在顶层：
//       assign aht1_sda   = aht1_sda_oe ? 1'b0 : 1'bz;
//       assign aht1_sda_i = aht1_sda;              // 回读引脚真实电平
//   子模块只收发 scl_oe / sda_oe / sda_i 三个纯信号。
//   ⚠️ 不要改回"在子模块内部读 inout"的写法，调试中证实会读不到从机拉低。
// ============================================================================
module top_env_monitor (
    input  wire sys_clk,       // 50MHz，球 P20（核心板 U9 晶振）

    inout  wire aht1_scl,      // AHT20 #1 SCL  → J8 Pin16 / W12
    inout  wire aht1_sda,      // AHT20 #1 SDA  → J8 Pin15 / Y12
    inout  wire aht2_scl,      // AHT20 #2 SCL  → J8 Pin20 / T10
    inout  wire aht2_sda,      // AHT20 #2 SDA  → J8 Pin19 / U10

    inout  wire oled_scl,      // OLED SCL      → J8 Pin18 / T12
    inout  wire oled_sda,      // OLED SDA      → J8 Pin17 / U12

    output wire led_hb,        // LED1 球 B2：1Hz 心跳，闪=工程在跑
    output wire led_alarm      // LED2 球 A2：温度超限常亮
);
    // ---------------- 复位（上电计数器，约 21ms 后释放）----------------
    // por_cnt 数到 0xF_FFFF 后**停住**（饱和），rst_n 永久保持 1。
    // ⚠️ 若改成不停的回绕计数，会每 20.97ms 复位整个设计一次，
    //    心跳永远数不到 0.5s、OLED 初始化永远走不完。
    reg [19:0] por_cnt = 20'd0;
    always @(posedge sys_clk) begin
        if (por_cnt != 20'hF_FFFF)
            por_cnt <= por_cnt + 1'b1;
    end
    wire rst_n = (por_cnt == 20'hF_FFFF);

    // ================= I2C 三态控制 =================
    wire aht1_scl_oe, aht1_sda_oe, aht1_sda_i;
    wire aht2_scl_oe, aht2_sda_oe, aht2_sda_i;
    wire oled_scl_oe, oled_sda_oe, oled_sda_i;

    assign aht1_scl   = aht1_scl_oe ? 1'b0 : 1'bz;
    assign aht1_sda   = aht1_sda_oe ? 1'b0 : 1'bz;
    assign aht1_sda_i = aht1_sda;

    assign aht2_scl   = aht2_scl_oe ? 1'b0 : 1'bz;
    assign aht2_sda   = aht2_sda_oe ? 1'b0 : 1'bz;
    assign aht2_sda_i = aht2_sda;

    assign oled_scl   = oled_scl_oe ? 1'b0 : 1'bz;
    assign oled_sda   = oled_sda_oe ? 1'b0 : 1'bz;
    assign oled_sda_i = oled_sda;

    // ---------------- 两路 AHT20 并行采集 ----------------
    wire [11:0] t1_x10, t2_x10;
    wire [10:0] h1_x10, h2_x10;

    aht20_driver #(
        .CLK_FREQ(50_000_000),
        .MEAS_PERIOD_MS(1000)
    ) u_ch1 (
        .clk(sys_clk), .rst_n(rst_n),
        .scl_oe(aht1_scl_oe), .sda_oe(aht1_sda_oe), .sda_i(aht1_sda_i),
        .dat_valid(), .temp_x10(t1_x10), .humi_x10(h1_x10)
    );

    aht20_driver #(
        .CLK_FREQ(50_000_000),
        .MEAS_PERIOD_MS(1000)
    ) u_ch2 (
        .clk(sys_clk), .rst_n(rst_n),
        .scl_oe(aht2_scl_oe), .sda_oe(aht2_sda_oe), .sda_i(aht2_sda_i),
        .dat_valid(), .temp_x10(t2_x10), .humi_x10(h2_x10)
    );

    // ---------------- OLED ----------------
    wire       oled_ready;
    wire       i2c_nak;        // 诊断用，正式版不显示
    reg        wr_en;
    reg  [7:0] wr_addr;        // {row[2:0], col[4:0]}，行基址 = 行号 × 21
    reg  [7:0] wr_char;

    oled_ssd1306 #(.CLK_FREQ(50_000_000)) u_oled (
        .clk(sys_clk), .rst_n(rst_n),
        .scl_oe(oled_scl_oe), .sda_oe(oled_sda_oe), .sda_i(oled_sda_i),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_char(wr_char),
        .ready(oled_ready),
        .i2c_nak(i2c_nak)
    );
    wire unused_nak = i2c_nak;

    // ---------------- 绝对值与 BCD 换算 ----------------
    wire [11:0] t1_mag = t1_x10[11] ? (~t1_x10 + 1'b1) : t1_x10;
    wire [11:0] t2_mag = t2_x10[11] ? (~t2_x10 + 1'b1) : t2_x10;
    wire [11:0] h1_u   = {1'b0, h1_x10};
    wire [11:0] h2_u   = {1'b0, h2_x10};

    reg [3:0] t1_h, t1_t, t1_o;
    reg [3:0] h1_h, h1_t, h1_o;
    reg [3:0] t2_h, t2_t, t2_o;
    reg [3:0] h2_h, h2_t, h2_o;

    always @* begin
        t1_h = t1_mag / 12'd100;
        t1_t = (t1_mag % 12'd100) / 12'd10;
        t1_o = t1_mag % 12'd10;
        h1_h = h1_u / 12'd100;
        h1_t = (h1_u % 12'd100) / 12'd10;
        h1_o = h1_u % 12'd10;

        t2_h = t2_mag / 12'd100;
        t2_t = (t2_mag % 12'd100) / 12'd10;
        t2_o = t2_mag % 12'd10;
        h2_h = h2_u / 12'd100;
        h2_t = (h2_u % 12'd100) / 12'd10;
        h2_o = h2_u % 12'd10;
    end

    function [7:0] digit(input [3:0] d);
        digit = 8'h30 + {4'd0, d};
    endfunction

    // ---------------- 超限告警（带 0.5℃ 回差）----------------
    localparam [11:0] TH_HI = 12'd300;   // 30.0 ℃ 触发
    localparam [11:0] TH_LO = 12'd295;   // 29.5 ℃ 解除

    reg alarm;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            alarm <= 1'b0;
        else if (!alarm && (t1_mag >= TH_HI || t2_mag >= TH_HI))
            alarm <= 1'b1;
        else if (alarm && t1_mag < TH_LO && t2_mag < TH_LO)
            alarm <= 1'b0;
    end

    // ---------------- 板载 LED ----------------
    // 底板 LED 经 MMBT3904 驱动，IO 输出 1 → 三极管导通 → LED 亮（高电平点亮）
    localparam integer T500MS = 25_000_000;   // 50MHz × 0.5s
    reg [24:0] hb_cnt;
    reg        hb;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt <= 25'd0; hb <= 1'b0;
        end else if (hb_cnt >= T500MS - 1) begin
            hb_cnt <= 25'd0; hb <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led_hb    = hb;
    assign led_alarm = alarm;

    // ---------------- 行基址（= 行号 × 21）----------------
    localparam [7:0] B0 = 8'd0;     // row0
    localparam [7:0] B2 = 8'd42;    // row2
    localparam [7:0] B4 = 8'd84;    // row4
    localparam [7:0] B6 = 8'd126;   // row6

    // ---------------- 100ms 刷新节拍 ----------------
    localparam integer T100MS = 5_000_000;
    reg [22:0] tick_cnt;
    reg        tick100;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 23'd0; tick100 <= 1'b0;
        end else if (tick_cnt >= T100MS - 1) begin
            tick_cnt <= 23'd0; tick100 <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1; tick100 <= 1'b0;
        end
    end

    task putc(input [7:0] a, input [7:0] c);
    begin
        wr_en   <= 1'b1;
        wr_addr <= a;
        wr_char <= c;
    end
    endtask

    reg [5:0] init_step;
    reg       inited;
    reg [4:0] ustep;
    reg       upd_run;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_en <= 1'b0; wr_addr <= 8'd0; wr_char <= 8'd0;
            init_step <= 6'd0; inited <= 1'b0;
            ustep <= 5'd0; upd_run <= 1'b0;
        end else begin
            wr_en <= 1'b0;

            // ---------- 上电静态画面 ----------
            if (oled_ready && !inited) begin
                case (init_step)
                // row0: "MULTI ENV MONITOR"
                6'd0:  putc(B0 + 8'd0,  "M");
                6'd1:  putc(B0 + 8'd1,  "U");
                6'd2:  putc(B0 + 8'd2,  "L");
                6'd3:  putc(B0 + 8'd3,  "T");
                6'd4:  putc(B0 + 8'd4,  "I");
                6'd5:  putc(B0 + 8'd5,  " ");
                6'd6:  putc(B0 + 8'd6,  "E");
                6'd7:  putc(B0 + 8'd7,  "N");
                6'd8:  putc(B0 + 8'd8,  "V");
                6'd9:  putc(B0 + 8'd9,  " ");
                6'd10: putc(B0 + 8'd10, "M");
                6'd11: putc(B0 + 8'd11, "O");
                6'd12: putc(B0 + 8'd12, "N");
                6'd13: putc(B0 + 8'd13, "I");
                6'd14: putc(B0 + 8'd14, "T");
                6'd15: putc(B0 + 8'd15, "O");
                6'd16: putc(B0 + 8'd16, "R");
                // row2 框架: "CH1 T:  . C  H:  . %"
                6'd17: putc(B2 + 8'd0,  "C");
                6'd18: putc(B2 + 8'd1,  "H");
                6'd19: putc(B2 + 8'd2,  "1");
                6'd20: putc(B2 + 8'd3,  " ");
                6'd21: putc(B2 + 8'd4,  "T");
                6'd22: putc(B2 + 8'd5,  ":");
                6'd23: putc(B2 + 8'd11, "C");
                6'd24: putc(B2 + 8'd12, " ");
                6'd25: putc(B2 + 8'd13, "H");
                6'd26: putc(B2 + 8'd14, ":");
                6'd27: putc(B2 + 8'd19, "%");
                // row4 框架: "CH2 ..."
                6'd28: putc(B4 + 8'd0,  "C");
                6'd29: putc(B4 + 8'd1,  "H");
                6'd30: putc(B4 + 8'd2,  "2");
                6'd31: putc(B4 + 8'd3,  " ");
                6'd32: putc(B4 + 8'd4,  "T");
                6'd33: putc(B4 + 8'd5,  ":");
                6'd34: putc(B4 + 8'd11, "C");
                6'd35: putc(B4 + 8'd12, " ");
                6'd36: putc(B4 + 8'd13, "H");
                6'd37: putc(B4 + 8'd14, ":");
                6'd38: putc(B4 + 8'd19, "%");
                // row6: "STATUS:NORMAL"
                6'd39: putc(B6 + 8'd0,  "S");
                6'd40: putc(B6 + 8'd1,  "T");
                6'd41: putc(B6 + 8'd2,  "A");
                6'd42: putc(B6 + 8'd3,  "T");
                6'd43: putc(B6 + 8'd4,  "U");
                6'd44: putc(B6 + 8'd5,  "S");
                6'd45: putc(B6 + 8'd6,  ":");
                6'd46: putc(B6 + 8'd7,  " ");
                6'd47: putc(B6 + 8'd8,  "N");
                6'd48: putc(B6 + 8'd9,  "O");
                6'd49: putc(B6 + 8'd10, "R");
                6'd50: putc(B6 + 8'd11, "M");
                6'd51: putc(B6 + 8'd12, "A");
                6'd52: putc(B6 + 8'd13, "L");
                default: wr_en <= 1'b0;
                endcase
                if (init_step <= 6'd52) init_step <= init_step + 1'b1;
                else                    inited    <= 1'b1;
            end
            // ---------- 周期刷新（每 100ms，24 拍）----------
            else if (inited && tick100 && !upd_run) begin
                upd_run <= 1'b1;
                ustep   <= 5'd0;
            end
            else if (upd_run) begin
                wr_en <= 1'b1;
                case (ustep)
                // CH1 温度
                5'd0:  begin wr_addr <= B2 + 8'd6;  wr_char <= t1_x10[11] ? "-" : " "; end
                5'd1:  begin wr_addr <= B2 + 8'd7;  wr_char <= digit(t1_h); end
                5'd2:  begin wr_addr <= B2 + 8'd8;  wr_char <= digit(t1_t); end
                5'd3:  begin wr_addr <= B2 + 8'd9;  wr_char <= "."; end
                5'd4:  begin wr_addr <= B2 + 8'd10; wr_char <= digit(t1_o); end
                // CH1 湿度
                5'd5:  begin wr_addr <= B2 + 8'd15; wr_char <= digit(h1_h); end
                5'd6:  begin wr_addr <= B2 + 8'd16; wr_char <= digit(h1_t); end
                5'd7:  begin wr_addr <= B2 + 8'd17; wr_char <= "."; end
                5'd8:  begin wr_addr <= B2 + 8'd18; wr_char <= digit(h1_o); end
                // CH2 温度
                5'd9:  begin wr_addr <= B4 + 8'd6;  wr_char <= t2_x10[11] ? "-" : " "; end
                5'd10: begin wr_addr <= B4 + 8'd7;  wr_char <= digit(t2_h); end
                5'd11: begin wr_addr <= B4 + 8'd8;  wr_char <= digit(t2_t); end
                5'd12: begin wr_addr <= B4 + 8'd9;  wr_char <= "."; end
                5'd13: begin wr_addr <= B4 + 8'd10; wr_char <= digit(t2_o); end
                // CH2 湿度
                5'd14: begin wr_addr <= B4 + 8'd15; wr_char <= digit(h2_h); end
                5'd15: begin wr_addr <= B4 + 8'd16; wr_char <= digit(h2_t); end
                5'd16: begin wr_addr <= B4 + 8'd17; wr_char <= "."; end
                5'd17: begin wr_addr <= B4 + 8'd18; wr_char <= digit(h2_o); end
                // 状态行
                5'd18: begin wr_addr <= B6 + 8'd8;  wr_char <= alarm ? "A" : "N"; end
                5'd19: begin wr_addr <= B6 + 8'd9;  wr_char <= alarm ? "L" : "O"; end
                5'd20: begin wr_addr <= B6 + 8'd10; wr_char <= alarm ? "A" : "R"; end
                5'd21: begin wr_addr <= B6 + 8'd11; wr_char <= alarm ? "R" : "M"; end
                5'd22: begin wr_addr <= B6 + 8'd12; wr_char <= alarm ? "M" : "A"; end
                5'd23: begin wr_addr <= B6 + 8'd13; wr_char <= alarm ? " " : "L"; end
                default: wr_en <= 1'b0;
                endcase
                if (ustep == 5'd23) begin
                    upd_run <= 1'b0;
                    ustep   <= 5'd0;
                end else begin
                    ustep <= ustep + 1'b1;
                end
            end
        end
    end
endmodule
