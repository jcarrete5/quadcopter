#!/bin/bash

BUILD_TYPE=${1:-Debug}
REPO_ROOT=$(git rev-parse --show-toplevel);
cd ${REPO_ROOT}
CMAKE_OPTIONS="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

BUILD_DIR="build-${BUILD_TYPE}"
mkdir ${BUILD_DIR} 2>/dev/null

if cmake -B ${BUILD_DIR} -S . ${CMAKE_OPTIONS}; then
    echo "CMake configuration finished: ${BUILD_DIR}"
else
    echo "CMake configuration failed"
    exit $?
fi

if cmake --build ${BUILD_DIR}; then
    echo "Build finished: ${BUILD_DIR}"
    cp ${BUILD_DIR}/compile_commands.json .
else
    echo "Build failed"
    exit $?
fi

if cmake --install ${BUILD_DIR} --prefix ${BUILD_DIR}; then
    echo "Installed to ${BUILD_DIR}"
else
    echo "Install to ${BUILD_DIR} failed"
fi
