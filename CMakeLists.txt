cmake_minimum_required(VERSION 3.20)
project(KartRiderDemoPhysics C)

if(MSVC)
    # The track tables carry Korean names as UTF-8. Without this MSVC decodes
    # them with the system codepage, so the sources fail to parse on a
    # non-UTF-8 locale such as CP949. Must precede every target below.
    add_compile_options(/utf-8)
endif()

add_library(kart_dynamics STATIC
    src/kart_dynamics.c
    src/kart_input.c
    src/kart_simulation.c
    src/kart_demo_data.c
    src/kart_inflate.c
    src/kart_camera.c
    src/kart_engine_sound.c
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

add_executable(
    kart_track_collision_tests
    tests/test_track_collision.c
    src/kart_track_collision.c
    src/kart_track_scene.c)
target_link_libraries(kart_track_collision_tests PRIVATE kart_dynamics)
add_test(NAME kart_track_collision_tests COMMAND kart_track_collision_tests)

add_executable(kart_inflate_tests tests/test_inflate.c src/kart_track_scene.c)
target_link_libraries(kart_inflate_tests PRIVATE kart_dynamics)
add_test(NAME kart_inflate_tests COMMAND kart_inflate_tests)

add_executable(kart_camera_tests tests/test_camera_follow.c)
target_link_libraries(kart_camera_tests PRIVATE kart_dynamics)
add_test(NAME kart_camera_tests COMMAND kart_camera_tests)

add_executable(kart_sound_tests tests/test_engine_sound.c)
target_link_libraries(kart_sound_tests PRIVATE kart_dynamics)
add_test(NAME kart_sound_tests COMMAND kart_sound_tests)

if(WIN32)
    set(KART_MINIMAP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/analysis/track-assets/extracted")
    set(MINIMAP_DESERT_I01 "${KART_MINIMAP_ROOT}/track_desert_I01/xt_minimap.png")
    set(MINIMAP_DESERT_I02 "${KART_MINIMAP_ROOT}/track_desert_I02/xt_minimap.png")
    set(MINIMAP_DESERT_R01 "${KART_MINIMAP_ROOT}/track_desert_R01/xt_minimap.png")
    set(MINIMAP_FOREST_I01 "${KART_MINIMAP_ROOT}/track_forest_I01/xt_minimap.png")
    set(MINIMAP_FOREST_I02 "${KART_MINIMAP_ROOT}/track_forest_I02/xt_minimap.png")
    set(MINIMAP_FOREST_R02 "${KART_MINIMAP_ROOT}/track_forest_R02/xt_minimap.png")
    set(MINIMAP_ICE_I01 "${KART_MINIMAP_ROOT}/track_ice_I01/xt_minimap.png")
    set(MINIMAP_ICE_I02 "${KART_MINIMAP_ROOT}/track_ice_I02/xt_minimap.png")
    set(MINIMAP_ICE_R01 "${KART_MINIMAP_ROOT}/track_ice_R01/xt_minimap.png")
    set(MINIMAP_VILLAGE_I01 "${KART_MINIMAP_ROOT}/track_village_I01/xt_minimap.png")
    set(MINIMAP_VILLAGE_I02 "${KART_MINIMAP_ROOT}/track_village_I02/xt_minimap.png")
    set(MINIMAP_VILLAGE_R01 "${KART_MINIMAP_ROOT}/track_village_R01/xt_minimap.png")
    set(MINIMAP_VILLAGE_R03 "${KART_MINIMAP_ROOT}/track_village_R03/xt_minimap.png")
    set(KART_MINIMAP_FILES
        "${MINIMAP_DESERT_I01}" "${MINIMAP_DESERT_I02}" "${MINIMAP_DESERT_R01}"
        "${MINIMAP_FOREST_I01}" "${MINIMAP_FOREST_I02}" "${MINIMAP_FOREST_R02}"
        "${MINIMAP_ICE_I01}" "${MINIMAP_ICE_I02}" "${MINIMAP_ICE_R01}"
        "${MINIMAP_VILLAGE_I01}" "${MINIMAP_VILLAGE_I02}"
        "${MINIMAP_VILLAGE_R01}" "${MINIMAP_VILLAGE_R03}")
    set(KART_HAVE_ALL_MINIMAPS TRUE)
    foreach(KART_MINIMAP_FILE IN LISTS KART_MINIMAP_FILES)
        if(NOT EXISTS "${KART_MINIMAP_FILE}")
            set(KART_HAVE_ALL_MINIMAPS FALSE)
        endif()
    endforeach()
    # The original effect samples. Without them the demos build and run silent.
    set(KART_SOUND_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/analysis/sound-assets")
    set(KART_SOUND_MOTOR "${KART_SOUND_ROOT}/sound_fx_kart/motor.wav")
    set(KART_SOUND_DRIFT "${KART_SOUND_ROOT}/sound_fx_kart/drift.wav")
    set(KART_SOUND_CRASH "${KART_SOUND_ROOT}/sound_fx_kart/crash.wav")
    set(KART_SOUND_SHOCK "${KART_SOUND_ROOT}/sound_fx_kart/shock.wav")
    set(KART_SOUND_BOOSTER "${KART_SOUND_ROOT}/sound_fx_item/booster/booster.wav")
    set(KART_HAVE_ALL_SOUNDS TRUE)
    foreach(KART_SOUND_FILE IN ITEMS
            "${KART_SOUND_MOTOR}" "${KART_SOUND_DRIFT}" "${KART_SOUND_CRASH}"
            "${KART_SOUND_SHOCK}" "${KART_SOUND_BOOSTER}")
        if(NOT EXISTS "${KART_SOUND_FILE}")
            set(KART_HAVE_ALL_SOUNDS FALSE)
        endif()
    endforeach()
    set(KART_AUDIO_SOURCES demo/kart_audio_win32.c)
    if(KART_HAVE_ALL_SOUNDS)
        configure_file(
            demo/kart_sounds.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_sounds.rc
            @ONLY)
        list(APPEND KART_AUDIO_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/kart_sounds.rc)
    else()
        message(STATUS "Original demo sounds not found; the demos will run silent")
    endif()

    set(KART_MINIMAP_SOURCES demo/kart_minimap_win32.c)
    if(KART_HAVE_ALL_MINIMAPS)
        configure_file(
            demo/kart_minimaps.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_minimaps.rc
            @ONLY)
        list(APPEND KART_MINIMAP_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/kart_minimaps.rc)
    else()
        message(STATUS "Original minimap PNGs not found; bounds panels will use grid fallback")
    endif()

    # Both demos call the scene and collision queries unconditionally; only the
    # resource loading is compiled out when there are no embedded scenes.
    add_executable(kart_topdown WIN32
        demo/kart_topdown_win32.c
        src/kart_track_scene.c
        src/kart_track_collision.c
        ${KART_MINIMAP_SOURCES}
        ${KART_AUDIO_SOURCES})
    target_include_directories(kart_topdown PRIVATE include demo)
    if(KART_HAVE_ALL_MINIMAPS)
        target_compile_definitions(kart_topdown PRIVATE KART_EMBED_MINIMAPS=1)
    endif()
    if(KART_HAVE_ALL_SOUNDS)
        target_compile_definitions(kart_topdown PRIVATE KART_EMBED_SOUNDS=1)
    endif()
    # winmm provides waveOut.
    target_link_libraries(kart_topdown PRIVATE
        kart_dynamics user32 gdi32 winmm windowscodecs ole32)

    # Track codes in TRACKS[] order; the RCDATA ids in
    # demo/kart_track_scene_resources.h follow the same order.
    set(KART_TRACK_CODES
        desert_I01 desert_I02 desert_R01
        forest_I01 forest_I02 forest_R02
        ice_I01 ice_I02 ice_R01
        village_I01 village_I02 village_R01 village_R03)
    set(KART_SCENE_MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/analysis/track-assets/meshes")
    set(KART_SCENE_PACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/analysis/track-assets/packed")

    set(KART_HAVE_ALL_SCENES TRUE)
    foreach(KART_TRACK_CODE IN LISTS KART_TRACK_CODES)
        if(NOT EXISTS "${KART_SCENE_MESH_DIR}/track_${KART_TRACK_CODE}.ktrk")
            set(KART_HAVE_ALL_SCENES FALSE)
        endif()
    endforeach()

    # Compress the KTRK exports into KTKZ containers (about 30% of raw size), so
    # all 13 scenes fit in the executables without a 12 MB resource section.
    if(KART_HAVE_ALL_SCENES)
        execute_process(
            COMMAND powershell -NoProfile -ExecutionPolicy Bypass
                    -File "${CMAKE_CURRENT_SOURCE_DIR}/scripts/pack_track_scenes.ps1"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE KART_SCENE_PACK_RESULT
            OUTPUT_VARIABLE KART_SCENE_PACK_OUTPUT
            ERROR_VARIABLE KART_SCENE_PACK_OUTPUT)
        if(NOT KART_SCENE_PACK_RESULT EQUAL 0)
            message(FATAL_ERROR
                "Packing the track scenes failed:\n${KART_SCENE_PACK_OUTPUT}")
        endif()
        foreach(KART_TRACK_CODE IN LISTS KART_TRACK_CODES)
            if(NOT EXISTS "${KART_SCENE_PACK_DIR}/track_${KART_TRACK_CODE}.ktkz")
                set(KART_HAVE_ALL_SCENES FALSE)
            endif()
        endforeach()
    endif()

    set(KART_3D_SOURCES
        demo/kart_3d_win32.c
        src/kart_track_scene.c
        src/kart_track_collision.c
        ${KART_MINIMAP_SOURCES}
        ${KART_AUDIO_SOURCES})
    if(KART_HAVE_ALL_SCENES)
        set(KART_TRACK_SCENE_RC_ENTRIES "")
        foreach(KART_TRACK_CODE IN LISTS KART_TRACK_CODES)
            string(TOUPPER "${KART_TRACK_CODE}" KART_TRACK_CODE_UPPER)
            string(APPEND KART_TRACK_SCENE_RC_ENTRIES
                "IDR_TRACK_SCENE_${KART_TRACK_CODE_UPPER} RCDATA \"${KART_SCENE_PACK_DIR}/track_${KART_TRACK_CODE}.ktkz\"\n")
        endforeach()
        configure_file(
            demo/kart_track_scenes.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc
            @ONLY)
        list(APPEND KART_3D_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc)
        target_sources(kart_topdown PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc)
        target_compile_definitions(kart_topdown PRIVATE KART_EMBED_TRACK_SCENES=1)
    else()
        message(STATUS "KTRK scene files not found; the demos will use bounds/grid fallback")
    endif()
    add_executable(kart_3d WIN32 ${KART_3D_SOURCES})
    target_include_directories(kart_3d PRIVATE include demo)
    if(KART_HAVE_ALL_SCENES)
        target_compile_definitions(kart_3d PRIVATE KART_EMBED_TRACK_SCENES=1)
    endif()
    if(KART_HAVE_ALL_MINIMAPS)
        target_compile_definitions(kart_3d PRIVATE KART_EMBED_MINIMAPS=1)
    endif()
    if(KART_HAVE_ALL_SOUNDS)
        target_compile_definitions(kart_3d PRIVATE KART_EMBED_SOUNDS=1)
    endif()
    # msimg32 provides AlphaBlend for the translucent faces; winmm provides waveOut.
    target_link_libraries(kart_3d PRIVATE
        kart_dynamics user32 gdi32 msimg32 winmm windowscodecs ole32)

    # The mixer is Win32-only, so its cursor test lives here rather than with
    # the platform-independent suites above.
    add_executable(kart_audio_tests
        tests/test_audio_cursor.c
        demo/kart_audio_win32.c)
    target_include_directories(kart_audio_tests PRIVATE include demo)
    target_link_libraries(kart_audio_tests PRIVATE kart_dynamics winmm)
    add_test(NAME kart_audio_tests COMMAND kart_audio_tests)

    # tools/ holds local-only harnesses and is not tracked, so a fresh clone
    # must still configure.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/oracle/kart_binary_oracle.c")
        add_executable(kart_binary_oracle tools/oracle/kart_binary_oracle.c)
    endif()

    # End-to-end over the real exports, so it only exists once they are packed.
    if(KART_HAVE_ALL_SCENES)
        add_executable(kart_scene_asset_tests
            tests/test_track_scene_assets.c
            src/kart_track_scene.c
            src/kart_track_collision.c)
        target_link_libraries(kart_scene_asset_tests PRIVATE kart_dynamics)
        add_test(
            NAME kart_scene_asset_tests
            COMMAND kart_scene_asset_tests "${KART_SCENE_PACK_DIR}")
    endif()
endif()

if(APPLE)
    enable_language(OBJC)

    set(KART_MAC_MINIMAP_RESOURCES)
    foreach(KART_MAC_TRACK IN ITEMS
            desert_I01 desert_I02 desert_R01
            forest_I01 forest_I02 forest_R02
            ice_I01 ice_I02 ice_R01
            village_I01 village_I02 village_R01 village_R03)
        set(KART_MAC_MINIMAP_SOURCE
            "${CMAKE_CURRENT_SOURCE_DIR}/analysis/track-assets/extracted/track_${KART_MAC_TRACK}/xt_minimap.png")
        if(EXISTS "${KART_MAC_MINIMAP_SOURCE}")
            set(KART_MAC_MINIMAP_COPY
                "${CMAKE_CURRENT_BINARY_DIR}/minimap_${KART_MAC_TRACK}.png")
            configure_file(
                "${KART_MAC_MINIMAP_SOURCE}"
                "${KART_MAC_MINIMAP_COPY}"
                COPYONLY)
            list(APPEND KART_MAC_MINIMAP_RESOURCES "${KART_MAC_MINIMAP_COPY}")
        endif()
    endforeach()
    if(KART_MAC_MINIMAP_RESOURCES)
        set_source_files_properties(${KART_MAC_MINIMAP_RESOURCES} PROPERTIES
            MACOSX_PACKAGE_LOCATION Resources)
    endif()

    add_executable(kart_topdown MACOSX_BUNDLE
        demo/kart_demo_macos.m ${KART_MAC_MINIMAP_RESOURCES})
    target_link_libraries(kart_topdown PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_topdown PRIVATE -fobjc-arc)
    set_target_properties(kart_topdown PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics Top Down"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.topdown"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")

    add_executable(kart_3d MACOSX_BUNDLE
        demo/kart_demo_macos.m ${KART_MAC_MINIMAP_RESOURCES})
    target_compile_definitions(kart_3d PRIVATE KART_DEMO_3D=1)
    target_link_libraries(kart_3d PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_3d PRIVATE -fobjc-arc)
    set_target_properties(kart_3d PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics 3D"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.3d"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")
endif()
