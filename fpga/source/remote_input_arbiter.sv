`default_nettype none
// A live Bluetooth session owns motion. All STOP sources remain active.
// Switching source always stops, and the IR decoder is inhibited while BT
// owns motion so old NEC repeats cannot take over after a disconnect.
module remote_input_arbiter(
 input wire clk,rst_n,bt_online,
 input wire [2:0] ir_action,bt_action,
 input wire ir_auto,bt_auto,ir_stop,bt_stop,ir_released,bt_released,
 output wire [2:0] manual_action,
 output wire auto_start,stop_request,released,ir_block,source_changed
);
 reg was_online;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)was_online<=0;else was_online<=bt_online;
 assign source_changed=was_online!=bt_online;
 assign stop_request=ir_stop || bt_stop || source_changed;
 assign ir_block=bt_online || source_changed || bt_stop;
 assign manual_action=stop_request?3'd0:(bt_online?bt_action:ir_action);
 assign auto_start=!stop_request && (bt_online?bt_auto:ir_auto);
 assign released=bt_online?bt_released:ir_released;
endmodule
`default_nettype wire
