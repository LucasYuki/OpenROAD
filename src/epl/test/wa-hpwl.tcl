source helpers.tcl
set test_name medium01-tcl
read_lef ./Nangate45/Nangate45.lef
read_def ./designs/$test_name.def

epl::calcualte_WaHPWL -gamma 1.0
