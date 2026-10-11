FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
# Prevent overriding parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(unit_tests
    test_pid.cpp
    test_state_machine.cpp
    test_actuator_mixer.cpp
    test_health_monitor.cpp
    test_imu_driver.cpp
    test_math.cpp
    test_zero_allocation.cpp
    test_flight_controller.cpp
)

target_include_directories(unit_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(unit_tests
    PRIVATE
        surface_controller_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
