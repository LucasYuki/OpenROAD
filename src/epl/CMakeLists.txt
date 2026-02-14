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

include("openroad")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

project(epl)

swig_lib(NAME      epl
         NAMESPACE epl
         I_FILE    src/EPlace.i
         SWIG_INCLUDES ${ODB_HOME}/src/swig/common
                       ${ODB_HOME}/src/swig/tcl
                       ${ODB_HOME}/include
         SCRIPTS   src/EPlace.tcl
)

target_sources(epl
  PRIVATE
    src/MakeEPlace.cpp
    src/EPlace.cpp
    src/EDensity.cpp
    src/WAwirelength.cpp
    src/Grid.cpp
)

messages(TARGET epl)

target_include_directories(epl
  PUBLIC
    include
  PRIVATE
    src
)

target_link_libraries(epl
    OpenSTA
    odb
    gpl
    gui
    Eigen3::Eigen
    Threads::Threads
    OpenMP::OpenMP_CXX
)

#if (Python3_FOUND AND BUILD_PYTHON)
#  swig_lib(NAME          epl_py
#           NAMESPACE     epl
#           LANGUAGE      python
#           I_FILE        src/EPlace-py.i
#           SWIG_INCLUDES ${PROJECT_SOURCE_DIR}/include/epl
#           SCRIPTS       ${CMAKE_CURRENT_BINARY_DIR}/epl_py.py
#  )
#
#  target_include_directories(epl_py
#    PUBLIC
#      include
#      
#  )
#
#  target_link_libraries(epl_py
#    PUBLIC
#      utl_lib
#      Eigen3::Eigen
#      gpl
#      odb
#      OpenSTA
#  )
#
#endif()

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()
