####################################
# global connections
####################################
add_global_connection -net {VDD} -inst_pattern {.*} -pin_pattern {^VDD$} -power
add_global_connection -net {VDD} -inst_pattern {.*} -pin_pattern {^VDDPE$}
add_global_connection -net {VDD} -inst_pattern {.*} -pin_pattern {^VDDCE$}
add_global_connection -net {VDD} -inst_pattern {.*} -pin_pattern {VPWR}
add_global_connection -net {VDD} -inst_pattern {.*} -pin_pattern {VPB}
add_global_connection -net {VSS} -inst_pattern {.*} -pin_pattern {^VSS$} -ground
add_global_connection -net {VSS} -inst_pattern {.*} -pin_pattern {^VSSE$}
add_global_connection -net {VSS} -inst_pattern {.*} -pin_pattern {VGND}
add_global_connection -net {VSS} -inst_pattern {.*} -pin_pattern {VNB}
global_connect
####################################
# voltage domains
####################################
set_voltage_domain -name {CORE} -power {VDD} -ground {VSS}
####################################
# standard cell grid
####################################
define_pdn_grid -name {grid} -voltage_domains {CORE}
add_pdn_stripe -grid {grid} -layer {met1} -width {0.48} -pitch {5.44} -offset {0} -followpins
add_pdn_stripe -grid {grid} -layer {met4} -width {1.600} -pitch {27.140} -offset {13.570}
add_pdn_stripe -grid {grid} -layer {met5} -width {1.600} -pitch {27.200} -offset {13.600}
add_pdn_connect -grid {grid} -layers {met1 met4}
add_pdn_connect -grid {grid} -layers {met4 met5}
####################################
# macro grids
####################################
####################################
# grid for: CORE_macro_grid_1
####################################
define_pdn_grid -name {CORE_macro_grid_1} -voltage_domains {CORE} -macro -orient {R0 R180 MX MY} -halo {2.0 2.0 2.0 2.0} -default -grid_over_boundary
add_pdn_connect -grid {CORE_macro_grid_1} -layers {met4 met5}
####################################
# grid for: CORE_macro_grid_2
####################################
define_pdn_grid -name "grid" -voltage_domains {"CORE" "PD_AES_1" "PD_AES_2"} \
  -power_control_network DAISY
add_pdn_stripe -grid "grid" -followpins -layer met1 -width 0.48 -extend_to_core_ring
add_pdn_stripe -grid "grid" -layer met4 -width 1.6 -spacing 1.6 -pitch 55.2 -offset 13.570 -extend_to_core_ring
add_pdn_stripe -grid "grid" -layer met5 -width 1.6 -spacing 1.6 -pitch 27.2 -offset 13.600 -extend_to_core_ring
add_pdn_ring -grid "grid" -layers {met4 met5} -widths 2.0 -spacings 2.0 -core_offsets 2
add_pdn_connect -grid "grid" -layers {met1 met4}
add_pdn_connect -grid "grid" -layers {met4 met5}


#####################################
## standard cell grid
#####################################
#define_pdn_grid -name {grid} -voltage_domains {"CORE"}
##"PD_AES_1" "PD_AES_2"}
#add_pdn_stripe -grid {grid} -layer {met1} -width {0.48} -pitch {5.44} -offset {0} -followpins
#add_pdn_stripe -grid {grid} -layer {met4} -width {1.600} -pitch {27.140} -offset {13.570}
#add_pdn_stripe -grid {grid} -layer {met5} -width {1.600} -pitch {27.200} -offset {13.600}
#add_pdn_connect -grid {grid} -layers {met1 met4}
#add_pdn_connect -grid {grid} -layers {met4 met5}
#
#####################################
## PD_AES_1 grids
#####################################
#define_pdn_grid -name "aes1_grid" -voltage_domains "PD_AES_1" \
#  -power_control_network DAISY
#add_pdn_stripe -grid "aes1_grid" -followpins -layer met1 -width 0.48 -extend_to_core_ring
#add_pdn_stripe -grid "aes1_grid" -layer met4 -width 1.6 -spacing 1.6 -pitch 55.2 -offset 13.570 -extend_to_core_ring
#add_pdn_stripe -grid "aes1_grid" -layer met5 -width 1.6 -spacing 1.6 -pitch 27.2 -offset 13.600 -extend_to_core_ring
#add_pdn_ring -grid "aes1_grid" -layers {met4 met5} -widths 2.0 -spacings 2.0 -core_offsets 2
#add_pdn_connect -grid "aes1_grid" -layers {met1 met4}
#add_pdn_connect -grid "aes1_grid" -layers {met4 met5}
#
#####################################
## PD_AES_2 grids
#####################################
#define_pdn_grid -name "aes2_grid" -voltage_domains "PD_AES_2" \
#  -power_control_network DAISY
#add_pdn_stripe -grid "aes2_grid" -followpins -layer met1 -width 0.48 -extend_to_core_ring
#add_pdn_stripe -grid "aes2_grid" -layer met4 -width 1.6 -spacing 1.6 -pitch 55.2 -offset 13.570 -extend_to_core_ring
#add_pdn_stripe -grid "aes2_grid" -layer met5 -width 1.6 -spacing 1.6 -pitch 27.2 -offset 13.600 -extend_to_core_ring
#add_pdn_ring -grid "aes2_grid" -layers {met4 met5} -widths 2.0 -spacings 2.0 -core_offsets 2
#add_pdn_connect -grid "aes2_grid" -layers {met1 met4}
#add_pdn_connect -grid "aes2_grid" -layers {met4 met5}
