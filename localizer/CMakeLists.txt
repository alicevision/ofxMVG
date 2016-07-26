# Build Localizer plugin
create_ofx_plugin(mvgLocalizer src/)

target_link_libraries(mvgLocalizer
  PUBLIC
    ${OPENMVG_LIBRARIES}
    ${Boost_LIBRARIES}
  )
target_include_directories(mvgLocalizer
  PUBLIC
    ${OPENMVG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
if(CMAKE_COMPILER_IS_GNUCXX)
  set_property(TARGET mvgLocalizer PROPERTY LINK_FLAGS "-Wl,--no-undefined") 
endif()
