# Copyright (c) 2021, The Regents of the University of California
# All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


# Put helper functions in a separate namespace so they are not visible
# too users in the global namespace.
namespace eval epl {

sta::define_cmd_args "eplace_random_placement" {}
proc eplace_random_placement { args } {
  epl::eplace_random_placement_cmd
}

sta::define_cmd_args "eplace_simulated_anealing" {\
    [-wait_iterations number]\
    [-initial_T temperature]\
    [-alpha alpha]\
    [-simple] \
}
proc eplace_simulated_anealing { args } {
  sta::parse_key_args "global_placement" args \
    keys {-wait_iterations -initial_T -alpha} flags {-simple}

  set wait_iterations 5
  if { [info exists keys(-wait_iterations)]} {
    set wait_iterations $keys(-wait_iterations)
    sta::check_positive_integer "-wait_iterations" $wait_iterations
  }
  
  set initial_T 1000
  if { [info exists keys(-initial_T)]} {
    set initial_T $keys(-initial_T)
    sta::check_positive_float "-initial_T" $initial_T
  }

  set alpha 0.99
  if { [info exists keys(-alpha)]} {
    set alpha $keys(-alpha)
    if { $alpha <= 0.0 || $alpha >= 1.0 } {
      utl::error EPL 4 "alpha must be >0 and <1."
    } 
  }

  if { [info exists flags(-simple)]} {
    epl::eplace_simulated_annealing_simple_cmd $wait_iterations $initial_T $alpha
  }
}

sta::define_cmd_args "test_epl" {}
proc test_epl { args } {
  puts "EPL working"
}


}

