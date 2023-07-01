include(QuadcopterWarnings)

add_library(
    driver
    i2c-device.cpp
)
if(PROJECT_IS_TOP_LEVEL)
    enable_warnings(driver)
endif()
target_include_directories(
    driver
    PUBLIC public
    PRIVATE .
)
target_link_libraries(
    driver
    PRIVATE util
)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    add_subdirectory(platform/linux)
endif()

if(BUILD_TESTING)
    include(GoogleTest)

    add_executable(
        test-i2c-device
        i2c-device.test.cpp
    )
    target_link_libraries(
        test-i2c-device
        PRIVATE
            driver
            GTest::gtest_main
    )
    gtest_discover_tests(test-i2c-device)
endif()
