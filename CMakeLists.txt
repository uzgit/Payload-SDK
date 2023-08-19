cmake_minimum_required(VERSION 2.8)
        
if (NOT USE_SYSTEM_ARCH)
    # select use platform 'LINUX' or 'RTOS' here, reset cache and reload cmake project
    set(USE_SYSTEM_ARCH LINUX)
endif ()

if (USE_SYSTEM_ARCH MATCHES RTOS)
    cmake_minimum_required(VERSION 3.15)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_AR arm-none-eabi-ar)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_OBJDUMP arm-none-eabi-objdump)
    set(SIZE arm-none-eabi-size)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif ()

project(entry)

# Disable in-source builds to prevent source tree corruption.
if (" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.")
endif ()

if (USE_SYSTEM_ARCH MATCHES LINUX)
    add_definitions(-DSYSTEM_ARCH_LINUX)
    add_subdirectory(samples/sample_c/platform/linux/manifold2)
    add_subdirectory(samples/sample_c++/platform/linux/manifold2)
    add_subdirectory(samples/sample_c++/platform/linux/generic_computational_payload)
    
    execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
    if (DEVICE_SYSTEM_ID MATCHES x86_64)
	message("Detected architecture: x86_64")
        set(LIBRARY_PATH psdk_lib/lib/x86_64-linux-gnu-gcc)
    elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
	message("Detected architecture: aarch64")
        set(LIBRARY_PATH psdk_lib/lib/aarch64-linux-gnu-gcc)
    else ()
        message(FATAL_ERROR "FATAL: Please confirm your platform.")
    endif ()

    file(READ /proc/device-tree/model RPI_MODEL)

    if("${RPI_MODEL}" MATCHES "Raspberry Pi")
	    message("Raspberry Pi detected - setting LIBRARY_PATH")
	set(LIBRARY_PATH psdk_lib/lib/arm-linux-gnueabihf-gcc) 
	    # Your Raspberry Pi-specific configurations here
	    #    else()
	    #        message("Not a Raspberry Pi**********************************************")
	    #	    # Your non-Raspberry Pi configurations here
    endif()
    message("LIBRARY_PATH: ${LIBRARY_PATH}")

    install(FILES ${LIBRARY_PATH}/libpayloadsdk.a
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
            )

    install(DIRECTORY psdk_lib/include
            DESTINATION "${CMAKE_INSTALL_PREFIX}"
            )
elseif (USE_SYSTEM_ARCH MATCHES RTOS)
    add_definitions(-DSYSTEM_ARCH_RTOS)
    add_subdirectory(samples/sample_c/platform/rtos_freertos/stm32f4_discovery/project/armgcc)
endif ()

add_custom_target(${PROJECT_NAME} ALL)
