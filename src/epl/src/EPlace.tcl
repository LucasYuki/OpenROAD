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


sta::define_cmd_args "eplace_place" { \
    [-density] \
}

proc eplace_place { args } {
  sta::parse_key_args "global_placement" args \
    keys {-density} \
    flags {}
  
  # density settings
  set target_density 0
  set uniform_mode 1

  if { [info exists keys(-density)] } {
    set target_density $keys(-density)
  }
  if { $target_density == "uniform" } {
    set uniform_mode 1
  } else {
    set uniform_mode 0
    sta::check_positive_float "-density" $target_density
    if { $target_density > 1.0 } {
      utl::error EPL 10 "Target density must be in \[0, 1\]."
    }
  }

  epl::eplace_place_cmd $target_density $uniform_mode
}


sta::define_cmd_args "eplace_random_placement" {}
proc eplace_random_placement { args } {
  epl::eplace_random_placement_cmd
}


sta::define_cmd_args "test_epl" {}
proc test_epl { args } {
  puts "EPL working"
}

}

