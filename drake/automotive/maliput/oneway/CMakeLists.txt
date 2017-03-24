add_library_with_exports(LIB_NAME drakeMaliputOneway SOURCE_FILES
  branch_point.cc
  junction.cc
  lane.cc
  road_geometry.cc
  segment.cc)

target_link_libraries(drakeMaliputOneway
  drakeCommon
)

drake_install_libraries(drakeMaliputOneway)

drake_install_headers(
  branch_point.h
  junction.h
  lane.h
  road_geometry.h
  segment.h)

add_executable(oneway_to_urdf oneway_to_urdf.cc)
target_link_libraries(oneway_to_urdf
    drakeAutomotive
    drakeCommon
    drakeMaliputOneway
    spruce)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
