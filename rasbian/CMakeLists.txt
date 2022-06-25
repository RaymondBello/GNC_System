# This file specifies how the project should be built, using CMake.
# If you are unfamiliar with CMake, don't worry about all the details.
# The sections you might want to edit are marked as such, and
# the comments should hopefully make most of it clear.
#
# For many purposes, you may not need to change anything about this file.

cmake_minimum_required(VERSION 3.14)

# Set project name, version and languages here. (change as needed)
# Version numbers are available by including "exampleConfig.h" in
# the source. See exampleConfig.h.in for some more details.
project(My6502 VERSION 1.0.0.0 LANGUAGES C CXX )
# project(My6502 VERSION 1.0.0.0 LANGUAGES C CXX OBJC OBJCXX)



# Options: Things you can set via commandline options to cmake (e.g. -DENABLE_LTO=[ON|OFF])
option(ENABLE_WARNINGS_SETTINGS "Allow target_set_warnings to add flags and defines.
                                 Set this to OFF if you want to provide your own warning parameters." ON)
option(ENABLE_LTO "Enable link time optimization" ON)
option(ENABLE_DOCTESTS "Include tests in the library. Setting this to OFF will remove all doctest related code.
                        Tests in tests/*.cpp will still be enabled." ON)

# Include stuff. No change needed.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
include(ConfigSafeGuards)
include(Colors)
include(CTest)
include(Doctest)
include(Documentation)
include(LTO)
include(Misc)
include(Warnings)

# Check for LTO support.
find_lto(CXX)

# --------------------------------------------------------------------------------
#                         Locate files (change as needed).
# --------------------------------------------------------------------------------
set(SOURCES          # All .cpp files in src/
    src/Bus.cpp
    src/R6502.cpp
)
set(TESTFILES        # All .cpp files in tests/
    tests/main.cpp
)
set(LIBRARY_NAME emulator)  # Default name for the library built from src/*.cpp (change if you wish)

# --------------------------------------------------------------------------------
#                            Build! (Change as needed)
# --------------------------------------------------------------------------------

SET(EXTRA_COMPILE_FLAGS " -x objective-c++ ")

# Compile all sources into a library.
add_library(${LIBRARY_NAME} OBJECT ${SOURCES})

# Lib needs its header files, and users of the library must also see these (PUBLIC). (No change needed)
target_include_directories(
    ${LIBRARY_NAME} PUBLIC 
        ${PROJECT_SOURCE_DIR}/include 
        ${PROJECT_SOURCE_DIR}/lib/olcPixelGameEngine
)

# There's also (probably) doctests within the library, so we need to see this as well.
target_link_libraries(${LIBRARY_NAME} PUBLIC doctest )

#########################################################
# FIND CARBON
#########################################################
FIND_LIBRARY(CARBON_LIBRARY Carbon)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${CARBON_LIBRARY})

#########################################################
# FIND PNG
#########################################################
find_package(PNG REQUIRED)
target_include_directories(${LIBRARY_NAME} PUBLIC ${PNG_INCLUDE_DIRS})
target_link_directories(${LIBRARY_NAME} PUBLIC ${PNG_LIBRARY_DIRS})
add_definitions(${PNG_DEFINITIONS})
if(NOT PNG_FOUND)
    message(ERROR " PNG not found!")
endif(NOT PNG_FOUND)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${PNG_LIBRARIES})

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
target_include_directories(${LIBRARY_NAME} PUBLIC  ${GLUT_INCLUDE_DIRS})
target_link_directories(${LIBRARY_NAME} PUBLIC ${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${GLUT_LIBRARIES})

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
target_include_directories(${LIBRARY_NAME} PUBLIC ${OpenGL_INCLUDE_DIRS})
target_link_directories(${LIBRARY_NAME} PUBLIC ${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${OPENGL_LIBRARIES})




# Set the compile options you want (change as needed).
target_set_warnings(${LIBRARY_NAME} ENABLE ALL DISABLE Annoying)
# target_compile_options(${LIBRARY_NAME} ... )  # For setting manually.

add_definitions(${EXTRA_COMPILE_FLAGS})



# Add an executable for the file app/main.cpp.
# If you add more executables, copy these lines accordingly.
add_executable(main app/main.cpp)   # Name of exec. and location of file.

target_link_libraries(main PRIVATE ${LIBRARY_NAME} )  # Link the executable to library (if it uses it).

target_compile_options(main PRIVATE  -Wall -Wno-unused-private-field -Wno-missing-braces -Wno-inconsistent-missing-override)
target_set_warnings(main ENABLE ALL  DISABLE Annoying) # Set warnings (if needed).
target_enable_lto(main optimized)  # enable link-time-optimization if available for non-debug configurations

# Set the properties you require, e.g. what C++ standard to use. Here applied to library and main (change as needed).
set_target_properties(
    ${LIBRARY_NAME} main
      PROPERTIES
        OBJCXX_STANDARD 17
        C_STANDARD 11
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

# Set up tests (see tests/CMakeLists.txt).
add_subdirectory(tests)
