cmake_minimum_required(VERSION 3.20)
project(KartRiderDemoPhysics C)

set(KART_RUNTIME_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Physics
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Gameplay
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Input
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Audio
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Rendering
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Runtime/Assets)
set(KART_WINDOWS_PLATFORM_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Scripts/Platform/Windows)

if(MSVC)
    # The track tables carry Korean names as UTF-8. Without this MSVC decodes
    # them with the system codepage, so the sources fail to parse on a
    # non-UTF-8 locale such as CP949. Must precede every target below.
    add_compile_options(/utf-8)
endif()

add_library(kart_dynamics STATIC
    Scripts/Runtime/Physics/kart_dynamics.c
    Scripts/Runtime/Input/kart_input.c
    Scripts/Runtime/Physics/kart_simulation.c
    Scripts/Runtime/Gameplay/kart_demo_data.c
    Scripts/Runtime/Assets/kart_inflate.c
    Scripts/Runtime/Rendering/kart_camera.c
    Scripts/Runtime/Audio/kart_engine_sound.c
    Scripts/Runtime/Gameplay/kart_countdown.c
)
target_include_directories(kart_dynamics PUBLIC ${KART_RUNTIME_INCLUDE_DIRS})
target_compile_features(kart_dynamics PUBLIC c_std_11)

if(MSVC)
    target_compile_options(kart_dynamics PRIVATE /W4)
else()
    target_compile_options(kart_dynamics PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(kart_dynamics PUBLIC m)
endif()

enable_testing()
add_executable(kart_dynamics_tests Scripts/Tests/test_kart_dynamics.c)
target_link_libraries(kart_dynamics_tests PRIVATE kart_dynamics)
add_test(NAME kart_dynamics_tests COMMAND kart_dynamics_tests)

add_executable(kart_oracle_compare Scripts/Tests/compare_lateral_oracle.c)
target_link_libraries(kart_oracle_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_lateral_binary_differential
    COMMAND kart_oracle_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-lateral.csv
)

add_executable(kart_trajectory_compare Scripts/Tests/compare_trajectory_oracle.c)
target_link_libraries(kart_trajectory_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_trajectory_binary_differential
    COMMAND kart_trajectory_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-trajectory.csv
)

add_executable(kart_instant_boost_compare Scripts/Tests/compare_instant_boost_oracle.c)
target_link_libraries(kart_instant_boost_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_instant_boost_binary_differential
    COMMAND kart_instant_boost_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-instant-boost.csv
)

add_executable(kart_timed_boost_compare Scripts/Tests/compare_timed_boost_oracle.c)
target_link_libraries(kart_timed_boost_compare PRIVATE kart_dynamics)
add_test(
    NAME kart_timed_boost_binary_differential
    COMMAND kart_timed_boost_compare ${CMAKE_CURRENT_SOURCE_DIR}/analysis/reports/oracle-timed-boost.csv
)

add_executable(
    kart_track_collision_tests
    Scripts/Tests/test_track_collision.c
    Scripts/Runtime/Physics/kart_track_collision.c
    Scripts/Runtime/Assets/kart_track_scene.c)
target_link_libraries(kart_track_collision_tests PRIVATE kart_dynamics)
add_test(NAME kart_track_collision_tests COMMAND kart_track_collision_tests)

# The course tables are generated from the track assets by
# DeveloperTools/AssetPipeline/derive_course_gates.py, so this needs no packed scene to run.
add_executable(
    kart_course_tests
    Scripts/Tests/test_course_progress.c
    Scripts/Runtime/Gameplay/kart_course.c
    Scripts/Runtime/Gameplay/kart_course_data.c
    Scripts/Runtime/Physics/kart_track_collision.c
    Scripts/Runtime/Assets/kart_track_scene.c)
target_link_libraries(kart_course_tests PRIVATE kart_dynamics)
add_test(NAME kart_course_tests COMMAND kart_course_tests)

add_executable(kart_inflate_tests Scripts/Tests/test_inflate.c Scripts/Runtime/Assets/kart_track_scene.c)
target_link_libraries(kart_inflate_tests PRIVATE kart_dynamics)
add_test(NAME kart_inflate_tests COMMAND kart_inflate_tests)

add_executable(kart_camera_tests Scripts/Tests/test_camera_follow.c)
target_link_libraries(kart_camera_tests PRIVATE kart_dynamics)
add_test(NAME kart_camera_tests COMMAND kart_camera_tests)

add_executable(kart_sound_tests Scripts/Tests/test_engine_sound.c)
target_link_libraries(kart_sound_tests PRIVATE kart_dynamics)
add_test(NAME kart_sound_tests COMMAND kart_sound_tests)

add_executable(kart_countdown_tests Scripts/Tests/test_countdown.c)
target_link_libraries(kart_countdown_tests PRIVATE kart_dynamics)
add_test(NAME kart_countdown_tests COMMAND kart_countdown_tests)

if(WIN32)
    set(KART_MINIMAP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/extracted")
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
    set(KART_SOUND_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Audio")
    set(KART_SOUND_DRIFT "${KART_SOUND_ROOT}/sound_fx_kart/drift.wav")
    set(KART_SOUND_CRASH "${KART_SOUND_ROOT}/sound_fx_kart/crash.wav")
    set(KART_SOUND_SHOCK "${KART_SOUND_ROOT}/sound_fx_kart/shock.wav")
    set(KART_ENGINE_SOUND_ROOT "${KART_SOUND_ROOT}/sound_fx_engine")
    set(KART_ENGINE_SOUND_FILES "")
    set(KART_ENGINE_SOUND_RC_ENTRIES "")
    set(KART_ENGINE_SOUND_RESOURCE_ID 600)
    macro(kart_add_engine_sound_preset folder engine booster)
        foreach(KART_ENGINE_SOUND_NAME IN ITEMS
                "${engine}" "${booster}" boosterDrift.wav boosterPlay.wav)
            set(KART_ENGINE_SOUND_FILE
                "${KART_ENGINE_SOUND_ROOT}/${folder}/${KART_ENGINE_SOUND_NAME}")
            list(APPEND KART_ENGINE_SOUND_FILES "${KART_ENGINE_SOUND_FILE}")
            string(APPEND KART_ENGINE_SOUND_RC_ENTRIES
                "${KART_ENGINE_SOUND_RESOURCE_ID} RCDATA \"${KART_ENGINE_SOUND_FILE}\"\n")
            math(EXPR KART_ENGINE_SOUND_RESOURCE_ID
                "${KART_ENGINE_SOUND_RESOURCE_ID} + 1")
        endforeach()
    endmacro()
    kart_add_engine_sound_preset(classic engine.wav booster.wav)
    kart_add_engine_sound_preset(sr engine.wav booster.wav)
    kart_add_engine_sound_preset(sr bikeEngine.wav bikeBooster.wav)
    kart_add_engine_sound_preset(z7 engine.wav booster.wav)
    kart_add_engine_sound_preset(z7 bikeEngine.wav bikeBooster.wav)
    kart_add_engine_sound_preset(ht engine.wav booster.wav)
    kart_add_engine_sound_preset(ht bikeEngine.wav bikeBooster.wav)
    kart_add_engine_sound_preset(jiu engine.wav booster.wav)
    kart_add_engine_sound_preset(jiu bikeEngine.wav bikeBooster.wav)
    kart_add_engine_sound_preset(x engine.wav booster.wav)
    kart_add_engine_sound_preset(x bikeEngine.wav bikeBooster.wav)
    kart_add_engine_sound_preset(v1 engine.wav booster.wav)
    kart_add_engine_sound_preset(v1 bikeEngine.wav bikeBooster.wav)
    set(KART_SOUND_COUNT_3 "${KART_SOUND_ROOT}/sound_fx_etc/count_3.wav")
    set(KART_SOUND_COUNT_2 "${KART_SOUND_ROOT}/sound_fx_etc/count_2.wav")
    set(KART_SOUND_COUNT_1 "${KART_SOUND_ROOT}/sound_fx_etc/count_1.wav")
    set(KART_SOUND_COUNT_GO "${KART_SOUND_ROOT}/sound_fx_etc/count_go.wav")
    set(KART_HAVE_ALL_SOUNDS TRUE)
    foreach(KART_SOUND_FILE IN ITEMS
            ${KART_ENGINE_SOUND_FILES}
            "${KART_SOUND_DRIFT}" "${KART_SOUND_CRASH}" "${KART_SOUND_SHOCK}"
            "${KART_SOUND_COUNT_3}" "${KART_SOUND_COUNT_2}"
            "${KART_SOUND_COUNT_1}" "${KART_SOUND_COUNT_GO}")
        if(NOT EXISTS "${KART_SOUND_FILE}")
            set(KART_HAVE_ALL_SOUNDS FALSE)
        endif()
    endforeach()
    set(KART_AUDIO_SOURCES Scripts/Platform/Windows/kart_audio_win32.c)
    if(KART_HAVE_ALL_SOUNDS)
        configure_file(
            Scripts/Platform/Windows/kart_sounds.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_sounds.rc
            @ONLY)
        list(APPEND KART_AUDIO_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/kart_sounds.rc)
    else()
        message(STATUS "Original demo sounds not found; the demos will run silent")
    endif()

    set(KART_MINIMAP_SOURCES Scripts/Platform/Windows/kart_minimap_win32.c)
    if(KART_HAVE_ALL_MINIMAPS)
        configure_file(
            Scripts/Platform/Windows/kart_minimaps.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_minimaps.rc
            @ONLY)
        list(APPEND KART_MINIMAP_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/kart_minimaps.rc)
    else()
        message(STATUS "Original minimap PNGs not found; bounds panels will use grid fallback")
    endif()

    # Track codes in TRACKS[] order; the RCDATA ids in
    # Scripts/Platform/Windows/kart_track_scene_resources.h follow the same order.
    set(KART_TRACK_CODES
        desert_I01 desert_I02 desert_R01
        forest_I01 forest_I02 forest_R02
        ice_I01 ice_I02 ice_R01
        village_I01 village_I02 village_R01 village_R03)
    set(KART_SCENE_MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/meshes")
    set(KART_SCENE_PACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/packed")

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
                    -File "${CMAKE_CURRENT_SOURCE_DIR}/DeveloperTools/AssetPipeline/pack_track_scenes.ps1"
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

    # Kart asset names in KARTS[] order; the RCDATA ids in
    # Scripts/Platform/Windows/kart_model_resources.h follow the same order. mine1 is in the archive
    # but is not a kart, so it is not listed here.
    set(KART_MODEL_NAMES
        practice1
        burst1 burst2 burst3 burst4 burst5
        cotten1 cotten2 cotten3 cotten4 cotten5
        marathon1 marathon2 marathon3 marathon4 marathon5
        saber1 saber2 saber3 saber4 saber5
        solid1 solid2 solid3 solid4 solid5)
    set(KART_MODEL_MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Models/Karts/meshes")
    set(KART_MODEL_PACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Models/Karts/packed")

    set(KART_HAVE_ALL_MODELS TRUE)
    foreach(KART_MODEL_NAME IN LISTS KART_MODEL_NAMES)
        if(NOT EXISTS "${KART_MODEL_MESH_DIR}/${KART_MODEL_NAME}.ktrk")
            set(KART_HAVE_ALL_MODELS FALSE)
        endif()
    endforeach()

    # Same KTKZ container as the track scenes, so the same packer and the same
    # loader handle both. All 26 models together are 130 KB packed.
    if(KART_HAVE_ALL_MODELS)
        execute_process(
            COMMAND powershell -NoProfile -ExecutionPolicy Bypass
                    -File "${CMAKE_CURRENT_SOURCE_DIR}/DeveloperTools/AssetPipeline/pack_track_scenes.ps1"
                    -InputDirectory "Assets/Models/Karts/meshes"
                    -OutputDirectory "Assets/Models/Karts/packed"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE KART_MODEL_PACK_RESULT
            OUTPUT_VARIABLE KART_MODEL_PACK_OUTPUT
            ERROR_VARIABLE KART_MODEL_PACK_OUTPUT)
        if(NOT KART_MODEL_PACK_RESULT EQUAL 0)
            message(FATAL_ERROR
                "Packing the kart models failed:\n${KART_MODEL_PACK_OUTPUT}")
        endif()
        foreach(KART_MODEL_NAME IN LISTS KART_MODEL_NAMES)
            if(NOT EXISTS "${KART_MODEL_PACK_DIR}/${KART_MODEL_NAME}.ktkz")
                set(KART_HAVE_ALL_MODELS FALSE)
            endif()
        endforeach()
    endif()

    # The skydome that ships beside each track.1s. Six of the thirteen do not
    # export (unsupported class stamp), so this list is only the ones that do
    # and Scripts/Platform/Windows/kart_skydome_resources.h maps them back onto TRACKS[].
    set(KART_SKYDOME_CODES
        ice_I01 ice_I02 ice_R01
        village_I01 village_I02 village_R01 village_R03)
    set(KART_SKYDOME_MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/skydome")
    set(KART_SKYDOME_PACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/packed-skydome")
    set(KART_HAVE_ALL_SKYDOMES TRUE)
    foreach(KART_SKYDOME_CODE IN LISTS KART_SKYDOME_CODES)
        if(NOT EXISTS "${KART_SKYDOME_MESH_DIR}/track_${KART_SKYDOME_CODE}.skydome.ktrk")
            set(KART_HAVE_ALL_SKYDOMES FALSE)
        endif()
    endforeach()
    if(KART_HAVE_ALL_SKYDOMES)
        execute_process(
            COMMAND powershell -NoProfile -ExecutionPolicy Bypass
                    -File "${CMAKE_CURRENT_SOURCE_DIR}/DeveloperTools/AssetPipeline/pack_track_scenes.ps1"
                    -InputDirectory "Assets/Tracks/skydome"
                    -OutputDirectory "Assets/Tracks/packed-skydome"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE KART_SKYDOME_PACK_RESULT
            OUTPUT_VARIABLE KART_SKYDOME_PACK_OUTPUT
            ERROR_VARIABLE KART_SKYDOME_PACK_OUTPUT)
        if(NOT KART_SKYDOME_PACK_RESULT EQUAL 0)
            message(FATAL_ERROR
                "Packing the skydomes failed:\n${KART_SKYDOME_PACK_OUTPUT}")
        endif()
        foreach(KART_SKYDOME_CODE IN LISTS KART_SKYDOME_CODES)
            if(NOT EXISTS "${KART_SKYDOME_PACK_DIR}/track_${KART_SKYDOME_CODE}.skydome.ktkz")
                set(KART_HAVE_ALL_SKYDOMES FALSE)
            endif()
        endforeach()
    endif()

    # The tracks' texture assets, decoded out of the theme archives and packed
    # into one shared KTEX table keyed by theme and texture name. The KTRK
    # meshes already name a texture and carry the asset's UVs, so this is the
    # only piece the textured renderer was missing.
    set(KART_TRACK_TEXTURE_PACK
        "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/packed/track_textures.ktxz")
    set(KART_HAVE_TRACK_TEXTURES FALSE)
    if(KART_HAVE_ALL_SCENES)
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            execute_process(
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/DeveloperTools/AssetPipeline/pack_track_textures.py"
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                RESULT_VARIABLE KART_TEXTURE_PACK_RESULT
                OUTPUT_VARIABLE KART_TEXTURE_PACK_OUTPUT
                ERROR_VARIABLE KART_TEXTURE_PACK_OUTPUT)
            if(NOT KART_TEXTURE_PACK_RESULT EQUAL 0)
                message(WARNING
                    "Packing the track textures failed; the demo will draw untextured:\n${KART_TEXTURE_PACK_OUTPUT}")
            endif()
        else()
            message(STATUS "Python not found; reusing any existing track texture pack")
        endif()
        if(EXISTS "${KART_TRACK_TEXTURE_PACK}")
            set(KART_HAVE_TRACK_TEXTURES TRUE)
        endif()
    endif()

    # One executable with both renderers behind the V key; it calls the scene
    # and collision queries unconditionally and only the resource loading is
    # compiled out when there are no embedded scenes.
    set(KART_DEMO_SOURCES
        Scripts/Platform/Windows/kart_win32.c
        Scripts/Runtime/Gameplay/kart_course.c
        Scripts/Runtime/Gameplay/kart_course_data.c
        Scripts/Runtime/Assets/kart_track_scene.c
        Scripts/Runtime/Rendering/kart_track_texture.c
        Scripts/Runtime/Physics/kart_track_collision.c
        ${KART_MINIMAP_SOURCES}
        ${KART_AUDIO_SOURCES})
    set(KART_SKIDMARK_TEXTURE
        "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Models/Karts/demo_effect/skidmark/skidmark.tga")
    if(EXISTS "${KART_SKIDMARK_TEXTURE}")
        configure_file(
            Scripts/Platform/Windows/kart_skidmark.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skidmark.rc
            @ONLY)
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_sounds.rc
            PROPERTIES OBJECT_DEPENDS "${KART_ENGINE_SOUND_FILES}")
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skidmark.rc
            PROPERTIES OBJECT_DEPENDS "${KART_SKIDMARK_TEXTURE}")
        list(APPEND KART_DEMO_SOURCES
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skidmark.rc)
    else()
        message(WARNING "Original skidmark.tga not found; skid marks will be untextured")
    endif()
    if(KART_HAVE_ALL_SCENES)
        set(KART_TRACK_SCENE_RC_ENTRIES "")
        foreach(KART_TRACK_CODE IN LISTS KART_TRACK_CODES)
            string(TOUPPER "${KART_TRACK_CODE}" KART_TRACK_CODE_UPPER)
            string(APPEND KART_TRACK_SCENE_RC_ENTRIES
                "IDR_TRACK_SCENE_${KART_TRACK_CODE_UPPER} RCDATA \"${KART_SCENE_PACK_DIR}/track_${KART_TRACK_CODE}.ktkz\"\n")
        endforeach()
        configure_file(
            Scripts/Platform/Windows/kart_track_scenes.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc
            @ONLY)
        # The .rc only names the packed files, so repacking leaves it untouched
        # and the resource object goes stale: the executable keeps whatever
        # scenes it was last linked with. Depending on the payloads makes a
        # repack rebuild the resource.
        set(KART_SCENE_PACK_FILES "")
        foreach(KART_TRACK_CODE IN LISTS KART_TRACK_CODES)
            list(APPEND KART_SCENE_PACK_FILES
                "${KART_SCENE_PACK_DIR}/track_${KART_TRACK_CODE}.ktkz")
        endforeach()
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc
            PROPERTIES OBJECT_DEPENDS "${KART_SCENE_PACK_FILES}")
        list(APPEND KART_DEMO_SOURCES
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_scenes.rc)
    else()
        message(STATUS "KTRK scene files not found; the demo will use bounds/grid fallback")
    endif()
    if(KART_HAVE_ALL_MODELS)
        set(KART_MODEL_RC_ENTRIES "")
        foreach(KART_MODEL_NAME IN LISTS KART_MODEL_NAMES)
            string(TOUPPER "${KART_MODEL_NAME}" KART_MODEL_NAME_UPPER)
            string(APPEND KART_MODEL_RC_ENTRIES
                "IDR_KART_MODEL_${KART_MODEL_NAME_UPPER} RCDATA \"${KART_MODEL_PACK_DIR}/${KART_MODEL_NAME}.ktkz\"\n")
        endforeach()
        configure_file(
            Scripts/Platform/Windows/kart_models.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_models.rc
            @ONLY)
        set(KART_MODEL_PACK_FILES "")
        foreach(KART_MODEL_NAME IN LISTS KART_MODEL_NAMES)
            list(APPEND KART_MODEL_PACK_FILES
                "${KART_MODEL_PACK_DIR}/${KART_MODEL_NAME}.ktkz")
        endforeach()
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_models.rc
            PROPERTIES OBJECT_DEPENDS "${KART_MODEL_PACK_FILES}")
        list(APPEND KART_DEMO_SOURCES
            ${CMAKE_CURRENT_BINARY_DIR}/kart_models.rc)
    else()
        message(STATUS "KTRK kart models not found; the demo will draw the box hull")
    endif()
    if(KART_HAVE_ALL_SKYDOMES)
        set(KART_SKYDOME_RC_ENTRIES "")
        set(KART_SKYDOME_PACK_FILES "")
        foreach(KART_SKYDOME_CODE IN LISTS KART_SKYDOME_CODES)
            string(TOUPPER "${KART_SKYDOME_CODE}" KART_SKYDOME_CODE_UPPER)
            string(APPEND KART_SKYDOME_RC_ENTRIES
                "IDR_SKYDOME_${KART_SKYDOME_CODE_UPPER} RCDATA \"${KART_SKYDOME_PACK_DIR}/track_${KART_SKYDOME_CODE}.skydome.ktkz\"\n")
            list(APPEND KART_SKYDOME_PACK_FILES
                "${KART_SKYDOME_PACK_DIR}/track_${KART_SKYDOME_CODE}.skydome.ktkz")
        endforeach()
        configure_file(
            Scripts/Platform/Windows/kart_skydomes.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skydomes.rc
            @ONLY)
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skydomes.rc
            PROPERTIES OBJECT_DEPENDS "${KART_SKYDOME_PACK_FILES}")
        list(APPEND KART_DEMO_SOURCES
            ${CMAKE_CURRENT_BINARY_DIR}/kart_skydomes.rc)
    else()
        message(STATUS "Skydome exports not found; the demo will draw an empty sky")
    endif()
    if(KART_HAVE_TRACK_TEXTURES)
        configure_file(
            Scripts/Platform/Windows/kart_track_textures.rc.in
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_textures.rc
            @ONLY)
        # As with the scenes: the .rc only names the pack, so a repack has to
        # invalidate the resource object or the executable keeps stale pixels.
        set_source_files_properties(
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_textures.rc
            PROPERTIES OBJECT_DEPENDS "${KART_TRACK_TEXTURE_PACK}")
        list(APPEND KART_DEMO_SOURCES
            ${CMAKE_CURRENT_BINARY_DIR}/kart_track_textures.rc)
    else()
        message(STATUS "Track texture pack not found; the demo will draw untextured")
    endif()
    add_executable(kart WIN32 ${KART_DEMO_SOURCES})
    target_include_directories(kart PRIVATE
        ${KART_RUNTIME_INCLUDE_DIRS}
        ${KART_WINDOWS_PLATFORM_DIR})
    if(KART_HAVE_ALL_SCENES)
        target_compile_definitions(kart PRIVATE KART_EMBED_TRACK_SCENES=1)
    endif()
    if(KART_HAVE_ALL_MODELS)
        target_compile_definitions(kart PRIVATE KART_EMBED_KART_MODELS=1)
    endif()
    if(KART_HAVE_TRACK_TEXTURES)
        target_compile_definitions(kart PRIVATE KART_EMBED_TRACK_TEXTURES=1)
    endif()
    if(KART_HAVE_ALL_SKYDOMES)
        target_compile_definitions(kart PRIVATE KART_EMBED_SKYDOMES=1)
    endif()
    if(KART_HAVE_ALL_MINIMAPS)
        target_compile_definitions(kart PRIVATE KART_EMBED_MINIMAPS=1)
    endif()
    if(KART_HAVE_ALL_SOUNDS)
        target_compile_definitions(kart PRIVATE KART_EMBED_SOUNDS=1)
    endif()
    # msimg32 provides AlphaBlend for the translucent faces; winmm provides waveOut.
    target_link_libraries(kart PRIVATE
        kart_dynamics user32 gdi32 msimg32 winmm windowscodecs ole32)

    # The mixer is Win32-only, so its cursor test lives here rather than with
    # the platform-independent suites above.
    add_executable(kart_audio_tests
        Scripts/Tests/test_audio_cursor.c
        Scripts/Platform/Windows/kart_audio_win32.c)
    target_include_directories(kart_audio_tests PRIVATE
        ${KART_RUNTIME_INCLUDE_DIRS}
        ${KART_WINDOWS_PLATFORM_DIR})
    target_link_libraries(kart_audio_tests PRIVATE kart_dynamics winmm)
    add_test(NAME kart_audio_tests COMMAND kart_audio_tests)

    # tools/ holds local-only harnesses and is not tracked, so a fresh clone
    # must still configure.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/oracle/kart_binary_oracle.c")
        add_executable(kart_binary_oracle tools/oracle/kart_binary_oracle.c)
    endif()

    if(KART_HAVE_ALL_MODELS)
        add_executable(kart_model_asset_tests
            Scripts/Tests/test_kart_model_assets.c
            Scripts/Runtime/Assets/kart_track_scene.c)
        target_include_directories(kart_model_asset_tests PRIVATE
            ${KART_RUNTIME_INCLUDE_DIRS}
            ${KART_WINDOWS_PLATFORM_DIR})
        target_link_libraries(kart_model_asset_tests PRIVATE kart_dynamics)
        add_test(
            NAME kart_model_asset_tests
            COMMAND kart_model_asset_tests "${KART_MODEL_PACK_DIR}")
    endif()

    # End-to-end over the real exports, so it only exists once they are packed.
    if(KART_HAVE_ALL_SCENES)
        add_executable(kart_scene_asset_tests
            Scripts/Tests/test_track_scene_assets.c
            Scripts/Runtime/Gameplay/kart_course.c
            Scripts/Runtime/Gameplay/kart_course_data.c
            Scripts/Runtime/Assets/kart_track_scene.c
            Scripts/Runtime/Physics/kart_track_collision.c)
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
            "${CMAKE_CURRENT_SOURCE_DIR}/Assets/Tracks/extracted/track_${KART_MAC_TRACK}/xt_minimap.png")
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
        Scripts/Platform/macOS/kart_demo_macos.m ${KART_MAC_MINIMAP_RESOURCES})
    target_link_libraries(kart_topdown PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_topdown PRIVATE -fobjc-arc)
    set_target_properties(kart_topdown PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics Top Down"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.topdown"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")

    add_executable(kart_3d MACOSX_BUNDLE
        Scripts/Platform/macOS/kart_demo_macos.m ${KART_MAC_MINIMAP_RESOURCES})
    target_compile_definitions(kart_3d PRIVATE KART_DEMO_3D=1)
    target_link_libraries(kart_3d PRIVATE kart_dynamics "-framework Cocoa")
    target_compile_options(kart_3d PRIVATE -fobjc-arc)
    set_target_properties(kart_3d PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Kart Physics 3D"
        MACOSX_BUNDLE_GUI_IDENTIFIER "local.kartriderdemo.physics.3d"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")
endif()
