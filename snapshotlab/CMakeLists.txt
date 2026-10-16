cmake_minimum_required(VERSION 3.22)
project(box3d_public_snapshot_experiment LANGUAGES C CXX)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Static library" FORCE)
set(BOX3D_PROFILE OFF CACHE BOOL "No profiler dependency" FORCE)
set(BOX3D_VALIDATE ON CACHE BOOL "Upstream validation" FORCE)
set(BOX3D_AVX2 OFF CACHE BOOL "Common four-lane backend" FORCE)
set(BOX3D_DISABLE_SIMD OFF CACHE BOOL "Common four-lane backend" FORCE)
set(BOX3D_DOUBLE_PRECISION OFF CACHE BOOL "Pinned float32 ABI" FORCE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

set(SNAPSHOTLAB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(SNAPSHOTLAB_UPSTREAM "${SNAPSHOTLAB_ROOT}/upstream")
add_subdirectory("${SNAPSHOTLAB_UPSTREAM}" upstream-build)

function(snapshotlab_contraction target)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        if(CMAKE_C_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
            target_compile_options("${target}" PRIVATE /clang:-ffp-contract=off)
        else()
            target_compile_options("${target}" PRIVATE -ffp-contract=off)
        endif()
    endif()
endfunction()

# The wire experiment changes only a generated serializer copy. The upstream
# checkout and original library remain available as the measured stock control.
set(generated "${CMAKE_BINARY_DIR}/snapshotlab-generated")
file(MAKE_DIRECTORY "${generated}")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/instrument_snapshot.py"
        --source "${SNAPSHOTLAB_UPSTREAM}/src/world_snapshot.c"
        --output "${generated}/world_snapshot.c"
    RESULT_VARIABLE generated_result OUTPUT_VARIABLE generated_manifest)
if(NOT generated_result EQUAL 0)
    message(FATAL_ERROR "Pinned snapshot serializer generation failed")
endif()
message(STATUS "${generated_manifest}")

get_target_property(source_dir box3d SOURCE_DIR)
get_target_property(original_sources box3d SOURCES)
set(wire_sources "")
set(replacement_count 0)
foreach(source IN LISTS original_sources)
    if(NOT IS_ABSOLUTE "${source}")
        set(source "${source_dir}/${source}")
    endif()
    get_filename_component(base "${source}" NAME)
    if(base STREQUAL "world_snapshot.c")
        set(source "${generated}/world_snapshot.c")
        math(EXPR replacement_count "${replacement_count} + 1")
    endif()
    list(APPEND wire_sources "${source}")
endforeach()
if(NOT replacement_count EQUAL 1)
    message(FATAL_ERROR "Expected exactly one snapshot serializer source")
endif()

add_library(snapshotlab-wire-library STATIC ${wire_sources})
foreach(property IN ITEMS INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_LIBRARIES
    INTERFACE_INCLUDE_DIRECTORIES INTERFACE_COMPILE_DEFINITIONS INTERFACE_LINK_LIBRARIES
    C_STANDARD C_STANDARD_REQUIRED C_EXTENSIONS MSVC_RUNTIME_LIBRARY)
    get_target_property(value box3d "${property}")
    if(NOT value STREQUAL "value-NOTFOUND")
        set_property(TARGET snapshotlab-wire-library PROPERTY "${property}" "${value}")
    endif()
endforeach()
target_include_directories(snapshotlab-wire-library PRIVATE "${source_dir}")
snapshotlab_contraction(snapshotlab-wire-library)

function(snapshotlab_executable name source library policy)
    add_executable("${name}" "${SNAPSHOTLAB_ROOT}/${source}")
    set_target_properties("${name}" PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_include_directories("${name}" PRIVATE "${SNAPSHOTLAB_UPSTREAM}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}" "${SNAPSHOTLAB_ROOT}")
    target_link_libraries("${name}" PRIVATE "${library}" Threads::Threads)
    target_compile_definitions("${name}" PRIVATE SNAPSHOTLAB_POLICY=${policy} BOX3D_DISABLE_AVX2)
    snapshotlab_contraction("${name}")
endfunction()

snapshotlab_executable(snapshotlab-transport-stock snapshotlab-transport.c box3d::box3d 0)
snapshotlab_executable(snapshotlab-transport-wire snapshotlab-transport.c snapshotlab-wire-library 1)
snapshotlab_executable(snapshotlab-cost snapshotlab-cost.c snapshotlab-wire-library 1)
