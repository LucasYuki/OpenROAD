source ../helpers.tcl
set test_name simple01-random
read_lef ../Nangate45/Nangate45.lef
read_lef ./newblue1_macro.lef
read_def ./newblue1.finish.def

global_placement -skip_initial_place -density 0.9
detailed_placement

#set def_file [make_result_file $test_name.def]
#write_def $def_file
#diff_file $def_file $test_name.defok
