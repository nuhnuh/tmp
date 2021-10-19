cmake_minimum_required(VERSION 3.10)

project(pybin11_snippet CXX)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen3
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  message(STATUS "Eigen3 found")
else()
  message(FATAL_ERROR "Eigen3 not found :/")
endif()
message(STATUS "EIGEN3_VERSION: ${Eigen3_VERSION}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

# mylib
set(Sources src/mylib.cpp)
#add_library(mylib STATIC ${Sources})
add_library(mylib SHARED ${Sources})
target_include_directories(mylib PUBLIC src)

# pybin11 (TIP: sudo apt install python3-pybind11)
find_package(pybind11 REQUIRED)
if(pybind11_FOUND)
  message(STATUS "pybind11 found")
else()
  message(FATAL_ERROR "pybind11 not found :/")
endif()
message(STATUS "pybind11_VERSION: ${pybind11_VERSION}")

## dbg: show all variables
#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#message("${Blink}DBG INFO:")
#foreach (_variableName ${_variableNames})
#    message("  ${_variableName}=${${_variableName}}")
#endforeach()
#message("${ColourReset}")

# pymylib
pybind11_add_module(pymylib src/pymylib.cpp)
target_include_directories(pymylib PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(pymylib PRIVATE mylib)
