# ============================================================================
# run_gui.do —— ModelSim 图形界面仿真（用于观察波形 / 截图）
#
# 用法（在仓库根目录执行）：
#   vsim -do tb/run_gui.do
#
# 执行后会自动：编译 → 加载 → 添加波形 → 跑到结束。
# 波形窗口会停留在 $finish 时刻，可用鼠标缩放到任意字节查看。
#
# 截图建议：
#   1. 选中 scl / sda 两条波形
#   2. 用 Zoom Full 后拖选一个字节（约 90us 宽）
#   3. 按 Zoom Selected，使 START + 地址字节 + ACK 充满画面
#   4. File -> Export -> Image，保存为 PNG
# ============================================================================

vlib work
vlog rtl/i2c_master.v tb/tb_i2c_master.v
vsim work.tb_i2c_master
do tb/wave.do
run -all
