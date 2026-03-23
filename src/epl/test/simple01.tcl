source helpers.tcl
set test_name simple01
read_lef ./Nangate45/Nangate45.lef
read_def ./designs/simple01.def

set_debug_level EPL place 1
set_debug_level EPL initEPlace 1
epl::eplace_debug -draw_bins
epl::eplace_place -density 0.8 -i 200 -density_penalty 0.1

set def_file [make_result_file $test_name.def]
write_def $def_file
diff_file $def_file $test_name.defok
