# ============================================================================
# wave.do —— 在 ModelSim GUI 中查看波形
#
# 用法（GUI 模式下加载设计后）：
#   do tb/wave.do
# ============================================================================

onerror {resume}
quietly WaveActivateNextPane {} 0

add wave -noupdate -divider {总线}
add wave -noupdate -format Logic -radix Binary /tb_i2c_master/scl
add wave -noupdate -format Logic -radix Binary /tb_i2c_master/sda

add wave -noupdate -divider {DUT 内部}
add wave -noupdate -format Literal -radix unsigned /tb_i2c_master/u_dut/state
add wave -noupdate -format Literal -radix unsigned /tb_i2c_master/u_dut/phase
add wave -noupdate -format Logic /tb_i2c_master/u_dut/scl_oe
add wave -noupdate -format Logic /tb_i2c_master/u_dut/sda_oe
add wave -noupdate -format Logic /tb_i2c_master/u_dut/is_addr
add wave -noupdate -format Literal -radix hex /tb_i2c_master/u_dut/shifter
add wave -noupdate -format Literal -radix hex /tb_i2c_master/u_dut/rd_data
add wave -noupdate -format Logic /tb_i2c_master/u_dut/done
add wave -noupdate -format Logic /tb_i2c_master/u_dut/ack_err

add wave -noupdate -divider {从机模型}
add wave -noupdate -format Logic /tb_i2c_master/s_drive
add wave -noupdate -format Logic /tb_i2c_master/s_bit
add wave -noupdate -format Literal -radix hex /tb_i2c_master/rx

configure wave -timelineunits us
TreeUpdate [SetDefaultTree]
