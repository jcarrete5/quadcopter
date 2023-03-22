include(posix)

add_library(util
    defer.cpp)
target_include_directories(util
    PUBLIC public
    PRIVATE .)

if(CMAKE_SYSTEM_NAME IN_LIST POSIX_COMPLIANT_SYSTEMS)
    add_subdirectory(platform/posix)
endif()
