# Build Localizer plugin
create_ofx_plugin(mvgLocalizer src/)

target_link_libraries(mvgLocalizer
  PUBLIC
    ${OPENMVG_LIBRARIES})
target_include_directories(mvgLocalizer
  PUBLIC
    ${OPENMVG_INCLUDE_DIRS})
