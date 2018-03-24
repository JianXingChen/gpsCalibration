cmake_minimum_required( VERSION 2.8 )
 
# Create Project
project( DrawFeatureMap )

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

if (${CMAKE_BUILD_TYPE} EQUAL "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

     
# Find PCL Package
find_package( PCL 1.8.0 REQUIRED )
 
# Find VTK Package
find_package( VTK REQUIRED )

# Fine Opencv
find_package( OpenCV REQUIRED )

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
 
# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
 
# Load VTK Settings
include( ${VTK_USE_FILE} )
 
# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )
 
# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR}/include )



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Add Library
add_library(${PROJECT_NAME} SHARED
src/DrawFeatureMap/CellInfo.cpp)
 
# Additional Dependencies
target_link_libraries( ${PROJECT_NAME}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
${OpenCV_LIBS}
)

# Add Executable
add_executable( ${PROJECT_NAME}_tool src/main.cpp)

target_link_libraries( ${PROJECT_NAME}_tool ${PROJECT_NAME})


