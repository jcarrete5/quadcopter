include(FetchContent)

FetchContent_MakeAvailable(googletest)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

include(GoogleTest)

add_executable(test-i2c-device)
target_sources(
    test-i2c-device
    PRIVATE
        i2c-device.test.cpp
)
target_link_libraries(
    test-i2c-device
    PRIVATE
        driver
        GTest::gtest_main
)
gtest_discover_tests(test-i2c-device)

if (CMAKE_SYSTEM_NAME STREQUAL Linux)
    add_dependencies(test-i2c-device i2c-stub-plain-kernel-module)
endif()
