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
    [-density target_density] \
    [-iterations max_iterations] \
    [-density_penalty density_penalty]
}

proc eplace_place { args } {
  sta::parse_key_args "global_placement" args \
    keys {-density -iterations -density_penalty} \
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

  set iterations 100
  if { [info exists keys(-iterations)] } {
    set iterations $keys(-iterations)
  }

  set density_penalty 10
  if { [info exists keys(-density_penalty)] } {
    set density_penalty $keys(-density_penalty)
  }

  epl::eplace_place_cmd $target_density $uniform_mode $density_penalty $iterations
}

sta::define_cmd_args "eplace_debug" { \
    [-draw_bins] \
    [-disable_wirelength] \
    [-disable_density]
}

proc eplace_debug { args } {
  sta::parse_key_args "global_placement" args \
    keys {} \
    flags {-draw_bins \
      -disable_wirelength \
      -disable_density}

  set draw_bins [info exists flags(-draw_bins)]
  set disable_wirelength [info exists flags(-disable_wirelength)]
  set disable_density [info exists flags(-disable_density)]
  if { $disable_wirelength && $disable_density } {
    utl::error EPL 14 "Cannot disable wirelength and density at the same time"
  }

  eplace_debug_cmd $draw_bins $disable_wirelength $disable_density
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

