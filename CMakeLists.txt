cmake_minimum_required(VERSION 3.20)
project(KartRiderDemoPhysics C)

add_library(kart_dynamics STATIC
    src/kart_dynamics.c
    src/kart_simulation.c
    src/kart_demo_data.c
)
target_include_directories(kart_dynamics PUBLIC include)
target_compile_features(kart_dynamics PUBLIC c_std_11)

if(MSVC)
    target_compile_options(kart_dynamics PRIVATE /W4)
else()
    target_compile_options(kart_dynamics PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(kart_dynamics PUBLIC m)
endif()

enable_testing()
add_executable(kart_dynamics_tests tests/test_kart_dynamics.c)
target_link_libraries(kart_dynamics_tests PRIVATE kart_dynamics)
add_test(NAME kart_dynamics_tests COMMAND kart_dynamics_tests)

add_executable(kart_oracle_compare tests/compare_lateral_oracle.c)
target_link_libraries(kart_oracle_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_lateral_binary_differential
    COMMAND kart_oracle_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-lateral.csv
)

add_executable(kart_trajectory_compare tests/compare_trajectory_oracle.c)
target_link_libraries(kart_trajectory_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_trajectory_binary_differential
    COMMAND kart_trajectory_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-trajectory.csv
)

add_executable(kart_instant_boost_compare tests/compare_instant_boost_oracle.c)
target_link_libraries(kart_instant_boost_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_instant_boost_binary_differential
    COMMAND kart_instant_boost_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-instant-boost.csv
)

add_executable(kart_timed_boost_compare tests/compare_timed_boost_oracle.c)
target_link_libraries(kart_timed_boost_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_timed_boost_binary_differential
    COMMAND kart_timed_boost_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-timed-boost.csv
)

if(WIN32)
    add_executable(kart_topdown WIN32 demo/kart_topdown_win32.c)
    target_link_libraries(kart_topdown PRIVATE kart_dynamics user32 gdi32)

    add_executable(kart_3d WIN32 demo/kart_3d_win32.c)
    target_link_libraries(kart_3d PRIVATE kart_dynamics user32 gdi32)

    add_executable(kart_binary_oracle tools/oracle/kart_binary_oracle.c)
endif()

if(APPLE)
    enable_language(OBJC)

    add_executable(kart_topdown MACOSX_BUNDLE demo/kart_demo_macos.m)
    target_link_libraries(kart_topdown PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_topdown PRIVATE -fobjc-arc)
    set_target_properties(kart_topdown PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics Top Down"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.topdown"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")

    add_executable(kart_3d MACOSX_BUNDLE demo/kart_demo_macos.m)
    target_compile_definitions(kart_3d PRIVATE KART_DEMO_3D=1)
    target_link_libraries(kart_3d PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_3d PRIVATE -fobjc-arc)
    set_target_properties(kart_3d PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics 3D"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.3d"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")
endif()
