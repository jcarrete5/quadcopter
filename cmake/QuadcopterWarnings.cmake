# Add quadcopter warning compile options to the specified target.
function(enable_warnings target_name)
    target_compile_options(
        ${target_name}
        PRIVATE ${QUADCOPTER_WARNINGS}
    )
endfunction()
