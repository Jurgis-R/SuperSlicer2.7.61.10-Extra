get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_api_print.cpp
	)
    
target_link_libraries(${_TEST_NAME}_tests test_common test_common_data libslic3r polyholes_plugin)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

if (WIN32)
    prusaslicer_copy_dlls(${_TEST_NAME}_tests)
    get_property(_is_multi GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if (_is_multi)
        set(_api_test_runtime_dir "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
    else()
        set(_api_test_runtime_dir "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    add_custom_target(${_TEST_NAME}_tests_runtime
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Slic3r> ${_api_test_runtime_dir}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:polyholes_plugin> ${_api_test_runtime_dir}
        DEPENDS Slic3r polyholes_plugin
        COMMENT "Copy plugin host and polyholes runtimes to api test directory"
        VERBATIM
    )
    add_dependencies(${_TEST_NAME}_tests ${_TEST_NAME}_tests_runtime)
    set_property(TARGET ${_TEST_NAME}_tests_runtime PROPERTY FOLDER "tests")
endif()

# catch_discover_tests(${_TEST_NAME}_tests TEST_PREFIX "${_TEST_NAME}: ")
add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests ${CATCH_EXTRA_ARGS})
