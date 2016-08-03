# Build openMVG plugin
create_ofx_plugin(mvg .)

target_link_libraries(mvg
  PUBLIC
    ${OPENMVG_LIBRARIES}
    ${Boost_LIBRARIES}
  )
target_include_directories(mvg
  PUBLIC
    ${OPENMVG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
if(CMAKE_COMPILER_IS_GNUCXX)
  set_property(TARGET mvg PROPERTY LINK_FLAGS "-Wl,--no-undefined") 
endif()
