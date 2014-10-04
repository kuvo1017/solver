#----------------------------------------------------------------------
#
# ADVENTURE MATES
# (MATES : Multi-Agent-based Traffic and Environment Simulator)
#
#----------------------------------------------------------------------

# cmake version
cmake_minimum_required(VERSION 2.6)

# start cmake
message(STATUS "*** cmake: ${CMAKE_CURRENT_SOURCE_DIR} ***")
set(CMAKE_VERBOSE_MAKEFILE TRUE)

#----------------------------------------------------------------------
# platform detection
if(NOT PLATFORM)
  set(PLATFORM ${CMAKE_SYSTEM_NAME})
endif(NOT PLATFORM)
string(TOLOWER ${PLATFORM} PLATFORM)

if(${PLATFORM} STREQUAL "windows")
  if(${CYGWIN})
    set(PLATFORM "cygwin")
  elseif(${MINGW})
    set(PLATFORM "mingw")
  endif(${CYGWIN})
endif(${PLATFORM} STREQUAL "windows")

message(STATUS "PLATFORM: ${PLATFORM}")

#----------------------------------------------------------------------
# libraries
set(LIBMATESUTIL "mates-util")
set(LIBMATES     "mates")
set(LIBMATESGL   "matesGL")

# make advmates-calc (default: true)
set(BUILD_MATES_CALC TRUE)
set(MATES_CALC "advmates-calc")

# make advmates-sim (default: true)
set(BUILD_MATES_SIM TRUE)
set(MATES_SIM "advmates-sim")
if(${PLATFORM} STREQUAL "novis")
  set(BUILD_MATES_SIM FALSE)
endif(${PLATFORM} STREQUAL "novis")

# using "advautogl-config" (defalut: false)
set(USE_AUTOGL_CONFIG FALSE)

# using OpenMP (default: true) (without Cygwin, MinGW)
set(USE_OPENMP FALSE)
if(${PLATFORM} STREQUAL "cygwin"
    OR ${PLATFORM} STREQUAL "mingw")
  set(USE_OPENMP FALSE)
endif(${PLATFORM} STREQUAL "cygwin"
  OR ${PLATFORM} STREQUAL "mingw")

#using accident
#set(USE_ACCIDENTS)
#----------------------------------------------------------------------
# compiler
set(CMAKE_C_COMPILER   gcc)
set(CMAKE_CXX_COMPILER g++)

# compile options
# build type = Debug (defualt)
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "build type is not specified, set to debug build")
  set(CMAKE_BUILD_TYPE "debug")
endif(NOT CMAKE_BUILD_TYPE)
string(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

if(${CMAKE_BUILD_TYPE} STREQUAL "winnative"
    AND NOT ${PLATFORM} STREQUAL "mingw")
  message(WARNING "winnative mode is only valid on mingw platform")
  set(CMAKE_BUILD_TYPE "debug")
endif(${CMAKE_BUILD_TYPE} STREQUAL "winnative"
  AND NOT ${PLATFORM} STREQUAL "mingw")

message(STATUS "configuring for ${CMAKE_BUILD_TYPE}...")

# options for release build
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wno-sign-compare")
set(CMAKE_C_FLAGS_RELEASE   "-O3 -Wno-sign-compare")
#if(${CMAKE_BUILD_TYPE} STREQUAL "release")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "release")

# options for windows native application
# #compile on MinGW
set(CMAKE_CXX_FLAGS_WINNATIVE "-O3 -Wno-sign-compare -mwindows -static-libgcc -static-libstdc++")
set(CMAKE_C_FLAGS_WINNATIVE   "-O3 -Wno-sign-compare -mwindows -static-libgcc")
#if(${CMAKE_BUILD_TYPE} STREQUAL "winnative")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "winnative")

# options for debug build
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wno-sign-compare")
set(CMAKE_C_FLAGS_DEBUG   "-g -Wall -Wno-sign-compare")
#if(${CMAKE_BUILD_TYPE} STREQUAL "debug")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "debug")

#----------------------------------------------------------------------
# MATES global settings
set(MATES_HEADER_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../include")
set(MATES_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib")

# AutoGL settings
set(AUTOGL_HEADER_DIR  "${MATES_LIBRARY_DIR}/autogl")
set(AUTOGL_LIBRARY_DIR "${MATES_LIBRARY_DIR}/autogl")

# for use advautogl-config
if(AUTOGL_CONFIG)
  execute_process(
    COMMAND
    sh ${AUTOGL_LIBRARY_DIR}/advautogl-config
    OUTPUT_VARIABLE
    AUTOGL_LIBS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(AUTOGL_CONFIG)

# MT19937 (Random number generator) settings
set(MT19937_HEADER_DIR  "${MATES_LIBRARY_DIR}/mt19937")
set(MT19937_LIBRARY_DIR "${MATES_LIBRARY_DIR}/mt19937")

#----------------------------------------------------------------------
# MATES simulation settings
option(EXCLUDE_VEHICLES "Define EXCLUDE_VEHICLES macro" OFF)
if(EXCLUDE_VEHICLES)
  add_definitions("-DEXCLUDE_VEHICLES")
endif(EXCLUDE_VEHICLES)

#----------------------------------------------------------------------
# find packages

# zlib
# The following variables will be defined:
#  ZLIB_FOUND
#  ZLIB_INCLUDE_DIRS
#  ZLIB_LIBRARIES
message(STATUS "Finding zlib ... ${ZLIB_FOUND}")
find_package(ZLIB)
if(ZLIB_FOUND)
  add_definitions("-DUSE_ZLIB")
  link_libraries(${ZLIB_LIBRARIES})
endif(ZLIB_FOUND)

if(NOT USE_AUTOGL_CONFIG AND BUILD_MATES_SIM)

  #********************************************************************
  # for PLATFORM LINUX
  #
  if(${PLATFORM} STREQUAL "linux")
    # OpenGL
    # The following variables will be defined:
    #   OPENGL_FOUND
    #   OPENGL_INCLUDE_DIR
    #   OPENGL_LIBRARIES
    message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
    find_package(OpenGL)
    if(NOT OPENGL_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT OPENGL_FOUND)

    # GTK2
    # The following variables will be defined:
    #   GTK2_FOUND
    #   GTK2_INCLUDE_DIRS
    #   GTK2_LIBRARIES
    message(STATUS "Finding GTK2 ... ${GTK2_FOUND}")
    find_package(GTK2)
    if(NOT GTK2_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT GTK2_FOUND)

  endif(${PLATFORM} STREQUAL "linux")

 if(${PLATFORM} STREQUAL "darwin")
    # OpenGL
    # The following variables will be defined:
    #   OPENGL_FOUND
    #   OPENGL_INCLUDE_DIR
    #   OPENGL_LIBRARIES
    message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
    find_package(OpenGL)
    if(NOT OPENGL_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT OPENGL_FOUND)

    # GTK2
    # The following variables will be defined:
    #   GTK2_FOUND
    #   GTK2_INCLUDE_DIRS
    #   GTK2_LIBRARIES
    message(STATUS "Finding GTK2 ... ${GTK2_FOUND}")
    find_package(GTK2)
    if(NOT GTK2_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT GTK2_FOUND)

   # X11
    # The following variables will be defined:
    #   OPENGL_FOUND
    #   OPENGL_INCLUDE_DIR
    #   OPENGL_LIBRARIES
    message(STATUS "Finding X11 ... ${X11_FOUND}")
    find_package(X11)
    if(NOT X11_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT X11_FOUND)

    # GLUT
    # The following variables will be defined:
    #   OPENGL_FOUND
    #   OPENGL_INCLUDE_DIR
    #   OPENGL_LIBRARIES
    message(STATUS "Finding GLUT ... ${GLUT_FOUND}")
    find_package(GLUT)
    if(NOT GLUT_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT GLUT_FOUND)

  endif(${PLATFORM} STREQUAL "darwin")
  #********************************************************************
  # for PLATFORM Unix
  #
  if(${PLATFORM} STREQUAL "unix")
    # OpenGL
    # The following variables will be defined:
    #   OPENGL_FOUND
    #   OPENGL_INCLUDE_DIR
    #   OPENGL_LIBRARIES
    message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
    find_package(OpenGL)
    if(NOT OPENGL_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT OPENGL_FOUND)

    # Motif
    # The folloing variables will be defined:
    #   MOTIF_FOUND
    #   MOTIF_INCLUDE_DIR
    #   MOTIF_LIBRARIES
    message(STATUS "Finding Motif ... ${MOTIF_FOUND}")
    find_package(Motif)
    if(NOT MOTIF_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT MOTIF_FOUND)
    
    # X11
    # The folloing variables will be defined:
    #   X11_FOUND
    #   X11_INCLUDE_DIR
    #   X11_LIBRARIES
    message(STATUS "Finding X11 ... ${X11_FOUND}")
    find_package(X11)
    if(NOT X11_FOUND)
      set(BUILD_MATES_SIM FALSE)
    endif(NOT X11_FOUND)

  endif(${PLATFORM} STREQUAL "unix")

  #********************************************************************
  # for PLATFORM Cygwin on Windows
  #
  if(${PLATFORM} STREQUAL "cygwin")
    set(CMAKE_LEGACY_CYGWIN_WIN32 TRUE)
    set(CMAKE_C_FLAGS_RELEASE
      "${CMAKE_C_FLAGS_RELEASE} -DUSE_OPENGL32")
    set(CMAKE_CXX_FLAGS_RELEASE
      "${CMAKE_CXX_FLAGS_RELEASE} -DUSE_OPENGL32")
    set(CMAKE_C_FLAGS_DEBUG
      "${CMAKE_C_FLAGS_DEBUG} -DUSE_OPENGL32")
    set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} -DUSE_OPENGL32")
    
  endif(${PLATFORM} STREQUAL "cygwin")

  #********************************************************************
  # for PLATFORM MinGW on Windows
  #
  if(${PLATFORM} STREQUAL "mingw")

    # OpenGL
    message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
    find_package(OpenGL)

    add_definitions("-DUSE_MINGW")
    
  endif(${PLATFORM} STREQUAL "mingw")

endif(NOT USE_AUTOGL_CONFIG AND BUILD_MATES_SIM)

# OpenMP
if(USE_OPENMP)
  set(CMAKE_C_FLAGS_RELEASE
    "${CMAKE_C_FLAGS_RELEASE} -fopenmp")
  set(CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp")
  set(CMAKE_C_FLAGS_DEBUG
    "${CMAKE_C_FLAGS_DEBUG} -fopenmp")
  set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -fopenmp")
endif(USE_OPENMP)

# include path
include_directories(${AUTOGL_HEADER_DIR})
include_directories(${MT19937_HEADER_DIR})

# library path
link_directories(${AUTOGL_LIBRARY_DIR})
link_directories(${MT19937_LIBRARY_DIR})

#----------------------------------------------------------------------
# source
set(MATES_UTIL_HDRS
  AmuLineSegment.h
  AmuConverter.h
  AmuMatrix.h
  AmuMatrix2D.h
  AmuMatrix3D.h
  AmuVector.h
  AmuInterval.h
  AmuPoint.h
  AmuStringOperator.h
  )

set(MATES_UTIL_SRCS
  AmuLineSegment.cpp
  AmuConverter.cpp
  AmuMatrix2D.cpp
  AmuMatrix3D.cpp
  AmuVector.cpp
  AmuInterval.cpp
  AmuPoint.cpp
  AmuStringOperator.cpp
  )

set(MATES_COMMON_SRCS
  AppMates.cpp
  Blinker.cpp
  Border.cpp
  Clocker.cpp
  Connector.cpp
  Detector.cpp
  DetectorIO.cpp
  DetectorUnit.cpp
  ErrorController.cpp
  OD.cpp
  GeneratingTable.cpp
  GenerateVehicleController.cpp
  GenerateVehicleIO.cpp
  GVInitializer.cpp
  GVManager.cpp
  FileManager.cpp
  Intersection.cpp
  IntersectionBuilder.cpp
  Lane.cpp
  LaneBundle.cpp
  LaneInSection.cpp
  LocalLaneRoute.cpp
  LocalLaneRouter.cpp
  NodeAStar.cpp
  ObjManager.cpp
  ODNode.cpp
  Random.cpp
  RelativeDirectionTable.cpp
  RoadEntity.cpp
  RoadMap.cpp
  RoadMapBuilder.cpp
  Route.cpp
  Router.cpp
  Section.cpp
  SectionBuilder.cpp
  Signal.cpp
  SignalAspect.cpp
  SignalColor.cpp
  SignalControlData.cpp
  SignalControlDataSet.cpp
  SignalIO.cpp
  Simulator.cpp
  SubIntersection.cpp
  SubSection.cpp
  TimeManager.cpp
  Tree.cpp
  Vehicle.cpp
  VehicleAccelDeterminer.cpp
  VehicleFamily.cpp
  VehicleFamilyIO.cpp
  VehicleFamilyManager.cpp
  VehicleIO.cpp
  VehicleLaneShifter.cpp
  VehicleRecognizer.cpp
  VehicleRunner.cpp
  VirtualLeader.cpp
  )

set(MATES_CALC_SRCS
  AppCalc.cpp
  )

set(MATES_SIM_SRCS
  AppSim.cpp
  Drawer.cpp
  GLColor.cpp
  Visualizer.cpp
  autogl_mates.c
  )

set(MATES_CALC_MAIN_SRC mainCalc.cpp)
set(MATES_SIM_MAIN_SRC  mainSim.cpp)

#----------------------------------------------------------------------
# build libmates.a libmatesGL.a
add_library(${LIBMATES} STATIC ${MATES_COMMON_SRCS})

if(BUILD_MATES_SIM)
  add_library(${LIBMATESGL}   STATIC ${MATES_SIM_SRCS})
endif(BUILD_MATES_SIM)

add_library(${LIBMATESUTIL} STATIC ${MATES_UTIL_SRCS})
add_custom_command(TARGET ${LIBMATESUTIL}
  POST_BUILD
  COMMAND cp -f  ${MATES_UTIL_HDRS} ${MATES_HEADER_DIR}
  COMMAND ln -fs ${CMAKE_CURRENT_SOURCE_DIR}/lib${LIBMATESUTIL}.a
  ${MATES_LIBRARY_DIR}/lib${LIBMATESUTIL}.a
  )

#----------------------------------------------------------------------
# build advmates-calc
if(BUILD_MATES_CALC)
  add_executable(${MATES_CALC}
    ${MATES_CALC_SRCS} ${MATES_CALC_MAIN_SRC})
  target_link_libraries(${MATES_CALC}
    ${LIBMATES} ${LIBMATESUTIL})
  target_link_libraries(${MATES_CALC}
    mt19937 m)
endif(BUILD_MATES_CALC)

#----------------------------------------------------------------------
# build advmates-sim
if(BUILD_MATES_SIM)
  add_executable(${MATES_SIM}
    ${MATES_SIM_MAIN_SRC})
  target_link_libraries(${MATES_SIM}
    ${LIBMATES} ${LIBMATESGL} ${LIBMATESUTIL})
  target_link_libraries(${MATES_SIM} mt19937 autogl m)

  if(USE_AUTOGL_CONFIG)
    target_link_libraries(${MATES_SIM}
      ${AUTOGL_LIBS}
      )

  else(USE_AUTOGL_CONFIG)
    if(${PLATFORM} STREQUAL "linux")
      target_link_libraries(${MATES_SIM}
        ${OPENGL_LIBRARIES}
        ${GTK2_LIBRARIES}
        )

    elseif(${PLATFORM} STREQUAL "darwin")
      target_link_libraries(${MATES_SIM}
        ${OPENGL_LIBRARIES}
        ${GTK2_LIBRARIES}
	${X11_LIBRARIES}
	${GLUT_LIBRARIES}
        )

    elseif(${PLATFORM} STREQUAL "unix")
      target_link_libraries(${MATES_SIM}
        ${OPENGL_LIBRARIES}
        ${MOTIF_LIBRARIES}
        ${X11_Xt_LIB}
        ${X11_Xmu_LIB}
        ${X11_Xext_LIB}
        ${X11_SM_LIB}
        ${X11_ICE_LIB}
        GLw
        )

    elseif(${PLATFORM} STREQUAL "cygwin")
      target_link_libraries(${MATES_SIM}
        glu32
        opengl32
        user32
        gdi32
        )

    elseif(${PLATFORM} STREQUAL "mingw")
      target_link_libraries(${MATES_SIM}
        glu32
        opengl32
        user32
        gdi32
        )

    endif(${PLATFORM} STREQUAL "linux")
  endif(USE_AUTOGL_CONFIG)
endif(BUILD_MATES_SIM)

#----------------------------------------------------------------------
# clean target
set(CLEAN_TARGETS
  ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
  ${MATES_LIBRARY_DIR}/lib${LIBMATESUTIL}.a
  )

set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${CLEAN_TARGETS}"
  )

#----------------------------------------------------------------------
# for flymake
get_directory_property(FM_INCLUDE_DIRECTORIES INCLUDE_DIRECTORIES)
foreach(FM_INCLUDE_DIRECTORY ${FM_INCLUDE_DIRECTORIES})
  set(FM_INCLUDES ${FM_INCLUDES} -I${FM_INCLUDE_DIRECTORY})
endforeach(FM_INCLUDE_DIRECTORY ${FM_INCLUDE_DIRECTORIES})

add_custom_target(check-syntax
  ${CMAKE_CXX_COMPILER} ${FM_INCLUDES}
  -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)
  )

#----------------------------------------------------------------------
# for doxygen
set(MATES_DOXYGEN_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../doc/doxygen")

find_program(DOT_EXECUTABLE dot)
message(STATUS "Finding dot ... ${DOT_EXECUTABLE}")
if(${DOT_EXECUTABLE} STREQUAL "DOT_EXECUTABLE-NOTFOUND")
  set(MATES_DOXYGEN_CONF "${MATES_DOXYGEN_DIR}/advmates-dox-min")
else(${DOT_EXECUTABLE} STREQUAL "DOT_EXECUTABLE-NOTFOUND")
  set(MATES_DOXYGEN_CONF "${MATES_DOXYGEN_DIR}/advmates-dox")
endif(${DOT_EXECUTABLE} STREQUAL "DOT_EXECUTABLE-NOTFOUND")

add_custom_target(dox
  COMMAND doxygen ${MATES_DOXYGEN_CONF}
  WORKING_DIRECTORY ${MATES_DOXYGEN_DIR}
  )
