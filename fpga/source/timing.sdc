# Preliminary constraints. External camera and RGMII timing still require board
# timing verification. Do not interpret an internal STA pass as board signoff.
create_clock -name ref50 -period 20.000 [get_ports {clk50M}]
create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]
create_generated_clock -name net125 -source [get_ports {clk50M}] -multiply_by 5 -divide_by 2 [get_pins {clocks/net_pll/u_pll/PLLA_inst/CLKOUT1}]
create_generated_clock -name camera24 -source [get_ports {clk50M}] -multiply_by 12 -divide_by 25 [get_pins {clocks/camera_pll/CLKOUT1}]
create_generated_clock -name rgmii_tx_clock -source [get_pins {clocks/net_pll/u_pll/PLLA_inst/CLKOUT1}] -divide_by 1 [get_ports {rgmii_gtxc}]
# Gray pointers must reach their first synchronizer within one fast clock period.
# The destination synchronizer's metastability is handled structurally, not by
# declaring the whole camera/network clock relationship false.
set_max_delay 7.000 -to [get_pins {packets/wg_meta_*/D packets/rg_meta_*/D}]
set_false_path -hold -to [get_pins {packets/wg_meta_*/D packets/rg_meta_*/D}]
# Descriptor is stable before Gray commit is observed and until slot release.
set_max_delay 7.000 -from [get_pins {packets/descriptor*/Q}]
set_false_path -hold -from [get_pins {packets/descriptor*/Q}]
set_false_path -to [get_pins {cfg_sync_0_s0/D link_cam_0_s0/D link_net_0_s0/D}]
# Reset and UART asynchronous entry points; downstream state remains timed.
set_false_path -from [get_ports {reset_n tof_uart_rx imu_uart_rx}]
# Only reset synchronizer async assertion inputs are exempt from recovery and
# removal. Their last stage releases reset in the destination clock domain;
# downstream functional registers retain normal recovery/removal analysis.
set_false_path -to [get_pins {phy_reset/sync_*/CLEAR cap_reset/sync_*/CLEAR clocks/net_rst/sync_*/CLEAR clocks/cam_rst/sync_*/CLEAR clocks/sys_rst/sync_*/CLEAR packets/wr_release_*/CLEAR packets/rd_release_sync_*/CLEAR}]
# External I/O min/max delays intentionally not guessed from generic RTL.
# Before hardware release: validate matched object counts and add measured/
# datasheet DVP delays plus RGMII DDR output requirements including PHY TX delay.
report_timing -setup -max_paths 100
report_timing -hold -max_paths 100
report_timing -recovery -max_paths 100
report_timing -removal -max_paths 100
report_timing -setup -to [get_pins {packets/wg_meta_*/D packets/rg_meta_*/D}] -max_paths 6
report_timing -setup -from [get_pins {packets/descriptor*/Q}] -max_paths 260

# Asynchronous UART/encoder pads only. Internal paths remain timed.
set_false_path -from [get_ports {control_rx enc1_a enc1_b enc2_a enc2_b}]

set_false_path -from [get_ports {ir_rx}]
set_false_path -from [get_ports {bt_rx}]
