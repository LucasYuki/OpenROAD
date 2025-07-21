# aes sky130hd 23539 insts
source "helpers.tcl"
source "flow_helpers.tcl"
source "sky130hd/sky130hd.vars"

# change the pdn config file
#set pdn_cfg "upf/mdp_aes.pdn.tcl"

lappend extra_lef "sky130hd/power_switch.lef"

set design "mpd_top"
set top_module "mpd_top"
set synth_verilog "upf/mpd_aes.v"
set sdc_file "aes_sky130hd.sdc"
set die_area {0 0 2800 2800}
set core_area {30 30 2770 2770}

set global_place_density uniform 
set global_place_overflow 0.05
set global_routing_layer_adjustments {{met1 0.10} {met2 0.10} {met3 0.15} {met4 0.25} {met5 0.25}}

set upf_file "upf/mpd_aes.upf"

include -echo "flow.tcl"
