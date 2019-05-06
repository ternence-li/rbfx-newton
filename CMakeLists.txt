#
# Copyright (c) 2017-2019 Rokas Kupstys.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

# Set CMake minimum version and CMake policy required by UrhoCommon module
cmake_minimum_required (VERSION 3.4.3)

# Set project name
project (NewtonPhysics)

# Dependencies
add_subdirectory(ThirdParty/newton-dynamics)

# Library
file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/Source/NewtonPhysics/*.h"
        "${PROJECT_SOURCE_DIR}/Source/NewtonPhysics/*.cpp"
        )

add_library(NewtonPhysics SHARED ${all_SRCS})
target_compile_definitions(NewtonPhysics PRIVATE URHONEWTON_EXPORTS)
target_include_directories(NewtonPhysics PUBLIC 
Source/NewtonPhysics
)



target_link_libraries(NewtonPhysics PUBLIC 

    dAnimation
    dContainers
    dCustomJoints
    dgCore
    dgNewtonAvx
    dgNewtonAvx2
    dgNewtonSse4.2
    #dgNewtonSse
    dgPhysics
    dNewton
    dMath
    newton

)


include(CheckIncludeFile)
check_include_file("valgrind/valgrind.h" HAVE_VALGRIND)
if (HAVE_VALGRIND)
    target_compile_definitions(NewtonPhysics PRIVATE -DHAVE_VALGRIND=${HAVE_VALGRIND})
endif ()


target_link_libraries(NewtonPhysics PUBLIC Urho3D)



if (URHO3D_SAMPLES)
	add_subdirectory(Samples/NewtonPhysicsTests)
endif ()

