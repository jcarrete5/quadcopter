# Adapted from https://musteresel.github.io/posts/2020/02/cmake-template-linux-kernel-module.html

# Find kernel module build directory
if(NOT DEFINED CACHE{LINUX_KERNEL_DIRECTORY})
    execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE UNAME_R
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(
        LINUX_KERNEL_DIRECTORY "/lib/modules/${UNAME_R}/build"
        CACHE PATH "Path to the kernel build directory (used to build kernel modules)"
    )
endif()
find_file(
    KERNEL_MAKEFILE
    NAMES Makefile
    PATHS "${LINUX_KERNEL_DIRECTORY}"
    DOC "Kernel module Makefile used to build kernel modules out-of-source"
    NO_DEFAULT_PATH
)
if(NOT KERNEL_MAKEFILE)
    message(FATAL_ERROR "There is no Makefile in the Linux kernel directory (${LINUX_KERNEL_DIRECTORY})!")
endif()


set(MODULE_NAME i2c-stub-plain)
configure_file(Kbuild.in Kbuild @ONLY)
add_custom_command(
    OUTPUT ${MODULE_NAME}.c
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/${MODULE_NAME}.c ${MODULE_NAME}.c
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/${MODULE_NAME}.c
    VERBATIM
)

add_custom_command(
    OUTPUT ${MODULE_NAME}.ko
    COMMAND make -C "${LINUX_KERNEL_DIRECTORY}" M=${CMAKE_CURRENT_BINARY_DIR} modules
    DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}.c
        ${CMAKE_CURRENT_BINARY_DIR}/Kbuild
    BYPRODUCTS
        ${MODULE_NAME}.mod
        ${MODULE_NAME}.mod.c
        ${MODULE_NAME}.mod.o
        ${MODULE_NAME}.o
        Module.symvers
        modules.order
    VERBATIM
)
add_custom_target(
    i2c-stub-plain-kernel-module
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}.ko
)
