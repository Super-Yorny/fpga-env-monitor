# ============================================================================
# run_sim.do —— ModelSim 一键仿真脚本
#
# 用法（在仓库根目录执行）：
#   vsim -c -do tb/run_sim.do
#
# 或在 ModelSim 命令行中：
#   cd <repo_root>
#   do tb/run_sim.do
# ============================================================================

vlib work
vlog rtl/i2c_master.v tb/tb_i2c_master.v
vsim -c work.tb_i2c_master
run -all
quit -f
