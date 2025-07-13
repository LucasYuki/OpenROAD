source helpers.tcl
set test_name simple01-random
read_lef ./Nangate45/Nangate45.lef
read_def ./designs/simple01.def

epl::eplace_simulated_anealing -wait_iterations 10000 -alpha 0.9999995 -initial_T 1e3 -density 1.2
# fazer a densidade começar a alta e ir baixando
#epl::eplace_simulated_anealing -wait_iterations 10000 -alpha 0.999995 -initial_T 1e3 -simple 
detailed_placement

#set def_file [make_result_file $test_name.def]
#write_def $def_file
#diff_file $def_file $test_name.defok
