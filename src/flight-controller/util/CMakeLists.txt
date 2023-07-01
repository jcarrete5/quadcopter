include(QuadcopterWarnings)

add_library(
    util
    defer.cpp
)
if(PROJECT_IS_TOP_LEVEL)
    enable_warnings(util)
endif()
target_include_directories(
    util
    PUBLIC public
    PRIVATE .
)

if(UNIX)
    add_subdirectory(platform/posix)
endif()
