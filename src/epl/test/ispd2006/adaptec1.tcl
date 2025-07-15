source ../helpers.tcl
set test_name simple01-random
read_lef ../Nangate45/Nangate45.lef
read_lef ./adaptec1_macro.lef
read_def ./adaptec1.finish.def

epl::eplace_simulated_anealing -wait_iterations 10000 -alpha 0.99995 -initial_T 1e6 -density 0.6 -print_period 1000 -swap_chance 0.2
#detailed_placement

#set def_file [make_result_file $test_name.def]
#write_def $def_file
#diff_file $def_file $test_name.defok
