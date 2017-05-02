if(ignition-rndf0_FOUND)
	drake_add_cc_test(ignition_rndf_test)
	target_include_directories(ignition_rndf_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(ignition_rndf_test "${IGNITION-RNDF_LIBRARIES}")

	drake_add_cc_test(builder_test)
	target_include_directories(builder_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(builder_test
		drakeMaliputRNDF
		"${IGNITION-RNDF_LIBRARIES}")

	drake_add_cc_test(lane_test)
	target_include_directories(lane_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(lane_test
		drakeMaliputRNDF
		"${IGNITION-RNDF_LIBRARIES}")

	drake_add_cc_test(segment_test)
	target_include_directories(segment_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(segment_test
		drakeMaliputRNDF
		"${IGNITION-RNDF_LIBRARIES}")

	drake_add_cc_test(spline_helper_test)
	target_include_directories(spline_helper_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(spline_helper_test
		drakeMaliputRNDF
		"${IGNITION-RNDF_LIBRARIES}")

	drake_add_cc_test(spline_lane_test)
	target_include_directories(spline_lane_test PRIVATE
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(spline_lane_test
		drakeMaliputRNDF
		"${IGNITION-RNDF_LIBRARIES}")

	add_executable(rndf_load rndf_load.cc)
  	target_include_directories(rndf_load PRIVATE
    	"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	target_link_libraries(rndf_load
		drakeMaliputRNDF
		"${IGNITION-RNDF_INCLUDE_DIRS}"
	)
	if(PYTHON_EXECUTABLE)
	  drake_add_test(
	    NAME rndf_load_test
	    COMMAND ${PYTHON_EXECUTABLE}
	    ${CMAKE_CURRENT_SOURCE_DIR}/rndf_load_test.py
	    $<TARGET_FILE:rndf_load>)
	endif()
endif()
