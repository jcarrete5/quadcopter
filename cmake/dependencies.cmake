# Contains definitions for declaring and managing dependencies.

include(FetchContent)

# Declare FetchContent dependencies used for flight-controller
function(declare_flight_controller_dependencies)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG 58d77fa8070e8cec2dc1ed015d66b454c8d78850)  # release-1.12.1
endfunction()
