cmake_minimum_required(VERSION 3.20)
project(SurfaceController
    VERSION 1.0.0
    DESCRIPTION "Deterministic Flight Surface Controller with FreeRTOS"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build the GoogleTest unit test suite" ON)
option(ENABLE_HEAP_TRACKING "Enable runtime heap interception hooks" ON)

include(cmake/CompilerWarnings.cmake)
include(FetchContent)

# -----------------------------------------------------------------------------
# FreeRTOS Configuration Target
# -----------------------------------------------------------------------------
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config" CACHE STRING "Path to FreeRTOSConfig.h" FORCE)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
)

# -----------------------------------------------------------------------------
# Fetch FreeRTOS-Kernel
# -----------------------------------------------------------------------------
if(NOT TARGET freertos_kernel)
    set(FREERTOS_PORT GCC_POSIX CACHE STRING "FreeRTOS port name")
    set(FREERTOS_HEAP 4 CACHE STRING "FreeRTOS heap implementation")

    FetchContent_Declare(
        freertos_kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG V10.5.1
    )
    FetchContent_MakeAvailable(freertos_kernel)
endif()

# -----------------------------------------------------------------------------
# Core Library: surface_controller_core
# -----------------------------------------------------------------------------
add_library(surface_controller_core STATIC
    src/os/MemoryTracker.cpp
    src/drivers/ImuDriver.cpp
    src/drivers/SimulatedImuDriver.cpp
    src/drivers/PwmActuatorDriver.cpp
    src/control/PidController.cpp
    src/control/FlightSurfaceController.cpp
    src/fsm/FlightStateMachine.cpp
    src/health/HealthMonitor.cpp
    src/health/HeartbeatTracker.cpp
    src/system/SystemTasks.cpp
)

target_include_directories(surface_controller_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(surface_controller_core
    PUBLIC
        freertos_kernel
        freertos_config
)

target_compile_options(surface_controller_core
    PRIVATE
        -fno-rtti
        -fno-exceptions
)

if(ENABLE_HEAP_TRACKING)
    target_compile_definitions(surface_controller_core
        PUBLIC
            SURFACE_CONTROLLER_ENABLE_HEAP_INTERCEPTION=1
    )
endif()

set_project_warnings(surface_controller_core)

# -----------------------------------------------------------------------------
# Simulation Executable: flight_controller_sim
# -----------------------------------------------------------------------------
add_executable(flight_controller_sim
    src/main.cpp
)

target_link_libraries(flight_controller_sim
    PRIVATE
        surface_controller_core
)

target_compile_options(flight_controller_sim
    PRIVATE
        -fno-rtti
        -fno-exceptions
)

set_project_warnings(flight_controller_sim)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
