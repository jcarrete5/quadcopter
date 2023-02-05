include(FetchContent)

FetchContent_MakeAvailable(googletest)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

include(GoogleTest)

add_executable(flight-controller-test
    flight-controller-hello-test.cpp)
target_link_libraries(flight-controller-test
    GTest::gtest_main)

gtest_discover_tests(flight-controller-test)
