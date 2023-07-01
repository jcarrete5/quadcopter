include(QuadcopterWarnings)

add_executable(
    flight-control
    main.cpp
)
if(PROJECT_IS_TOP_LEVEL)
    enable_warnings(flight-control)
endif()
target_link_libraries(
    flight-control
    PRIVATE
        Boost::boost
        util
        driver
)

add_subdirectory(util)
add_subdirectory(driver)
