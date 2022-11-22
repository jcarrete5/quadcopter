include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG 58d77fa8070e8cec2dc1ed015d66b454c8d78850 #release-1.12.1
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE) # For Windows: Prevent overriding the parent project's compiler/linker settings
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(flight-controller-test 
    flight-controller-hello-test.cpp
)
target_link_libraries(flight-controller-test
    GTest::gtest_main
)

gtest_discover_tests(flight-controller-test)
