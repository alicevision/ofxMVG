cmake_minimum_required(VERSION 2.8) 

project(openMVG_ofx)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
endif ()

# Check that submodule have been initialized and updated
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/openfx/include)
  message(FATAL_ERROR
    "\n submodule(s) are missing, please update your repository:\n"
    "  > git submodule update -i\n")
endif()

find_package(OpenMVG REQUIRED)
find_package(Ceres)
find_package(OpenCV)

# Add openfx subdirectory
add_subdirectory("${PROJECT_SOURCE_DIR}/openfx")

# Add Localizer plugin
add_subdirectory("${PROJECT_SOURCE_DIR}/localizer")

