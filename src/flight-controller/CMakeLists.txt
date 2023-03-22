include(dependencies)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

declare_flight_controller_dependencies()
find_package(Boost 1.74.0 REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(util)
add_subdirectory(driver)

add_executable(flight-control)
target_sources(
    flight-control
    PRIVATE main.cpp
)
target_link_libraries(
    flight-control
    PRIVATE
        Boost::boost
        util
        driver
)
