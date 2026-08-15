cmake_minimum_required(VERSION 3.10)
project(grblHAL-glowforge C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP and friends.
add_compile_definitions(_GNU_SOURCE)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # The stepper producer thread runs core "ISR" code concurrently with the
    # protocol thread; keep type-punning-heavy embedded code out of
    # strict-aliasing territory.
    add_compile_options(-fno-strict-aliasing)
    # An implicitly declared function returning float reads the wrong
    # register on the hard-float target; make the whole class a hard error.
    add_compile_options(-Werror=implicit-function-declaration)
endif()

# Machine configuration: my_machine.h (-> boards/glowforge.h) is force-
# included into every translation unit, core sources included, so the
# core's #ifndef-guarded defaults in grbl/config.h pick up the measured
# Glowforge machine constants without touching the submodule.
add_compile_options(-include ${CMAKE_CURRENT_LIST_DIR}/src/my_machine.h)

# Capture the effective build-time flags for the -v/--version output.
string(TOUPPER "${CMAKE_BUILD_TYPE}" _drv_build_type)
if(_drv_build_type STREQUAL "")
    set(DRV_BUILD_TYPE "None")
else()
    set(DRV_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()
string(STRIP "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${_drv_build_type}}" DRV_C_FLAGS)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/src/build_info.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/build_info.h
    @ONLY
)

include(src/grbl/CMakeLists.txt)

set(DRIVER_SOURCES
    src/main.c
    src/driver.c
    src/stepper_stream.c
    src/glowforge_io.c
    src/glowforge_cooling.c
    src/glowforge_homing.c
    src/glowforge_laser.c
    src/glowforge_switches.c
    src/serial.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/platform_linux.c
)

add_executable(grblHAL_glowforge ${DRIVER_SOURCES})

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # The DRIVER's own sources are warning-free and held to that; the
    # core submodule is upstream code and is not held to -Werror here.
    set_source_files_properties(${DRIVER_SOURCES}
        tests/switch_map_test.c tests/laser_arm_test.c
        PROPERTIES COMPILE_OPTIONS "-Werror")
endif()

target_link_libraries(grblHAL_glowforge PRIVATE
    m
    grbl
    rt
    pthread
)

# Generated build_info.h lands in the build dir.
target_include_directories(grblHAL_glowforge PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS grblHAL_glowforge RUNTIME DESTINATION bin)

# Host unit test for the EV_SW switch-map decode (the pure mapping in
# src/glowforge_switch_map.h). Built everywhere, installed nowhere; CI
# runs it after the build.
add_executable(switch_map_test tests/switch_map_test.c)
# Headers only - the test exercises the pure mapping and must not link
# the core (whose entry points expect the full driver).
target_include_directories(switch_map_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

# Host unit test for the operator-arm coolant re-check (G-4): the arm
# must re-check gfcool_fire_ok() after the button wait. It includes the
# driver source directly and stubs the core, so - like switch_map_test -
# it must not link the core. Built everywhere, installed nowhere.
add_executable(laser_arm_test tests/laser_arm_test.c)
target_include_directories(laser_arm_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(laser_arm_test m)
