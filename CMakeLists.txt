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
find_package(Boost)
#find_package(CCTag)

#IF(NOT CCTAG_FOUND)
#  MESSAGE(STATUS "CCTAG package not found, CCTAG will be disabled")
#ELSE(NOT CCTAG_FOUND)
#  MESSAGE(STATUS "CCTAG package found")
#  MESSAGE(STATUS "CCTAG library directory ${CCTAG_LIBRARY_DIR}")
#  MESSAGE(STATUS "CCTAG library names     ${CCTAG_LIBRARIES_NAMES}")
#  MESSAGE(STATUS "CCTAG include directory ${CCTAG_INCLUDE_DIR}")
  ADD_DEFINITIONS(-DHAVE_CCTAG)
#ENDIF(NOT CCTAG_FOUND)


#if(CMAKE_COMPILER_IS_GNUCXX)
#  include(AddCompilerFlag)
#
#  # This flag is useful as not returning from a non-void function is an error with MSVC
#  AddCompilerFlag("-Werror=return-type")
#  AddCompilerFlag("-Werror=switch")
#  AddCompilerFlag("-Werror=return-local-addr")
#endif()

# Add openfx subdirectory
add_subdirectory("${PROJECT_SOURCE_DIR}/openfx")

# Add plugin source 
add_subdirectory("${PROJECT_SOURCE_DIR}/src")

