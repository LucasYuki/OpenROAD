source helpers.tcl
set test_name simple01
read_lef ./Nangate45/Nangate45.lef
read_def ./designs/simple01.def

set_debug_level EPL grid 2
epl::eplace_place -density 0.8

set def_file [make_result_file $test_name.def]
write_def $def_file
diff_file $def_file $test_name.defok
