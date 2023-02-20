include(FetchContent)

FetchContent_MakeAvailable(googletest)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

include(GoogleTest)

add_executable(i2c-device-test
    i2c-device-test.cpp)
target_link_libraries(i2c-device-test
    PRIVATE driver
    PRIVATE GTest::gtest_main)
gtest_discover_tests(i2c-device-test)
