# dependencies.cmake
include(FetchContent)

set(WEBGPU_TARGET_DIR "${CMAKE_SOURCE_DIR}/external/wgpu")
set(WEBGPU_SO_FILE "${WEBGPU_TARGET_DIR}/lib/libwgpu_native.so")

if(NOT EXISTS "${WEBGPU_SO_FILE}" OR NOT EXISTS "${WEBGPU_TARGET_DIR}/include")
    message(STATUS "WebGPU binary not found locally. Downloading...")

    FetchContent_Declare(
        webgpu_binaries
        URL      "https://github.com/gfx-rs/wgpu-native/releases/download/v22.1.0.5/wgpu-linux-x86_64-release.zip"
        # URL_HASH SHA256=put_the_hash_here  
    )

    FetchContent_MakeAvailable(webgpu_binaries)

    file(COPY 
        "${webgpu_binaries_SOURCE_DIR}/lib/libwgpu_native.so" 
        DESTINATION "${WEBGPU_TARGET_DIR}/lib"
    )

    file(COPY 
        "${webgpu_binaries_SOURCE_DIR}/include/" 
        DESTINATION "${WEBGPU_TARGET_DIR}/include"
    )
    message(STATUS "WebGPU binary successfully installed to ${WEBGPU_TARGET_DIR}")
else()
    message(STATUS "WebGPU binary already exists at ${WEBGPU_SO_FILE}. Skipping download.")
endif()