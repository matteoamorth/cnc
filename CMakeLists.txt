cmake_minimum_required(VERSION 3.10)
# NOTE: This project is mostly already configured.
#       Go to the "Targets" section to add new products
#   _____          _ _           _                        
#  |  __ \        | (_)         (_)                       
#  | |__) | __ ___| |_ _ __ ___  _ _ __   __ _ _ __ _   _ 
#  |  ___/ '__/ _ \ | | '_ ` _ \| | '_ \ / _` | '__| | | |
#  | |   | | |  __/ | | | | | | | | | | | (_| | |  | |_| |
#  |_|   |_|  \___|_|_|_| |_| |_|_|_| |_|\__,_|_|   \__, |
#                                                    __/ |
#                                                   |___/ 
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed! Create a build directory and run CMake from there.")
endif()
# This is dynamically updated in defines.h.in:
set(VERSION "1.1")
project(C-CNC VERSION ${VERSION} LANGUAGES C CXX)


#   ____        _ _     _            _   _   _                 
#  | __ ) _   _(_) | __| |  ___  ___| |_| |_(_)_ __   __ _ ___ 
#  |  _ \| | | | | |/ _` | / __|/ _ \ __| __| | '_ \ / _` / __|
#  | |_) | |_| | | | (_| | \__ \  __/ |_| |_| | | | | (_| \__ \
#  |____/ \__,_|_|_|\__,_| |___/\___|\__|\__|_|_| |_|\__, |___/
#                                                    |___/     
# Build type (default to Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type default to Debug" FORCE)
endif()
# Compiler flags
add_compile_options(
  -fPIC 
  -D_GNU_SOURCE 
  -Wno-backslash-newline-escape 
  -Wno-comment
)
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Debug mode, enabling all warnings")
  add_compile_options(-Wall -DDEBUG -Wno-comment)
endif()
# Language Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
# RPATH management
# This embeds the relative path of the dynamic library into the executable
# So that it can find the library across installations
# If this is not given, the executable only searches the library in its same 
# folder or in a system path (/usr/lib, /usr/local/lib)
if(APPLE)
  set(CMAKE_INSTALL_RPATH "@executable_path/../lib")
  if(EXISTS /usr/local)
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
  endif()
  # on M1 Macs, brew uses /opt/homebrew:
  if(EXISTS /opt/homebrew)
    include_directories(/opt/homebrew/include)
    link_directories(/opt/homebrew/lib)
  endif()
else()
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib;/usr/local/lib")
endif()
# FILES
# expect all sources in /src, except mains
set(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
file(GLOB LIB_SOURCES "${SOURCE_DIR}/[^_]*.c")
# generate defines.h
configure_file(
  ${SOURCE_DIR}/defines.h.in
  ${SOURCE_DIR}/defines.h
)

# cmake config option for enabling/disabling syslog in FSM
option(SYSLOG "Enable use of syslog facility" OFF)
if(SYSLOG)
  add_definitions(-DSYSLOG)
endif()

#   _____                    _       
#  |_   _|_ _ _ __ __ _  ___| |_ ___ 
#    | |/ _` | '__/ _` |/ _ \ __/ __|
#    | | (_| | | | (_| |  __/ |_\__ \
#    |_|\__,_|_|  \__, |\___|\__|___/
#                 |___/              
# Libraries
# All files in /src (except src/main) are compiled into a library
add_compile_options(-pthread)

add_library(c-cnc_lib SHARED ${LIB_SOURCES})
target_link_libraries(c-cnc_lib m mosquitto)
add_library(c-cnc_static STATIC ${LIB_SOURCES})

# Test executables
add_executable(point ${SOURCE_DIR}/point.c)
target_compile_definitions(point PUBLIC POINT_MAIN)
target_link_libraries(point m)

add_executable(machine ${LIB_SOURCES})
target_compile_definitions(machine PUBLIC MACHINE_MAIN)
target_link_libraries(machine m mosquitto)

add_executable(block ${LIB_SOURCES})
target_compile_definitions(block PUBLIC BLOCK_MAIN)
target_link_libraries(block m mosquitto)

add_executable(program ${LIB_SOURCES})
target_compile_definitions(program PUBLIC PROGRAM_MAIN)
target_link_libraries(program m mosquitto)

# Tuning axes PID
add_executable(tuning ${LIB_SOURCES})
target_compile_definitions(tuning PUBLIC AXIS_MAIN)
target_link_libraries(tuning c-cnc_static m mosquitto)

# main programs
add_executable(c-cnc ${SOURCE_DIR}/main/c-cnc.c)
target_link_libraries(c-cnc c-cnc_static m mosquitto)

add_executable(ccnc ${SOURCE_DIR}/main/ccnc.c)
target_link_libraries(ccnc c-cnc_lib m mosquitto)

add_executable(simulate ${SOURCE_DIR}/main/simulate.c)
target_link_libraries(simulate c-cnc_lib m mosquitto)


#   _____           _        _ _ 
#  |_   _|         | |      | | |
#    | |  _ __  ___| |_ __ _| | |
#    | | | '_ \/ __| __/ _` | | |
#   _| |_| | | \__ \ || (_| | | |
#  |_____|_| |_|___/\__\__,_|_|_|
# List of targets to install
list(APPEND TARGETS_LIST
  ccnc
  c-cnc c-cnc_static c-cnc_lib
)
# Destination directory
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/products)
install(TARGETS ${TARGETS_LIST}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static
)


#   _____         _   
#  |_   _|__  ___| |_ 
#    | |/ _ \/ __| __|
#    | |  __/\__ \ |_ 
#    |_|\___||___/\__|
include(CTest)
add_test(NAME point COMMAND point)
add_test(NAME block COMMAND block)
