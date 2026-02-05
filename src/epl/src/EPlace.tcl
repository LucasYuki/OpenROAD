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

sta::define_cmd_args "eplace_place" {}
proc eplace_random_placement { args } {
  epl::eplace_place_cmd
}

sta::define_cmd_args "test_epl" {}
proc test_epl { args } {
  puts "EPL working"
}

}

