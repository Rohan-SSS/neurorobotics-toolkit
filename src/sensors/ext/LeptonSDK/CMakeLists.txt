cmake_minimum_required(VERSION 3.0.0)
project(LeptonSDK VERSION 0.1.0)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Debug)
#ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(LEPTON_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(LEPTON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} ${LEPTON_SOURCE_DIR}/crc16fast.c ${LEPTON_SOURCE_DIR}/LEPTON_AGC.c ${LEPTON_SOURCE_DIR}/LEPTON_OEM.c ${LEPTON_SOURCE_DIR}/LEPTON_RAD.c ${LEPTON_SOURCE_DIR}/LEPTON_SDK.c ${LEPTON_SOURCE_DIR}/LEPTON_SYS.c ${LEPTON_SOURCE_DIR}/LEPTON_VID.c)

target_include_directories(${PROJECT_NAME} 
   PUBLIC ${LEPTON_INCLUDE_DIR}
)
