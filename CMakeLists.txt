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

add_executable(grblHAL_glowforge
    src/main.c
    src/driver.c
    src/stepper_stream.c
    src/glowforge_io.c
    src/glowforge_cooling.c
    src/glowforge_homing.c
    src/glowforge_laser.c
    src/serial.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/platform_linux.c
)

target_link_libraries(grblHAL_glowforge PRIVATE
    m
    grbl
    rt
    pthread
)

# Generated build_info.h lands in the build dir.
target_include_directories(grblHAL_glowforge PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS grblHAL_glowforge RUNTIME DESTINATION bin)
