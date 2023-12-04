project(NMEA)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(ENV{LAPTOP_TEST} 1)

set(NMEA_DIR ${PROJECT_SOURCE_DIR})
set(SYSTEM_PACKAGES "/home/divanshu-g/Libraries")

set (NMEA_INCLUDE_DIR ${NMEA_DIR}/include)
set (NMEA_SOURCE_DIR ${NMEA_DIR}/src)
#set (SPDLOG_DIR ${SKYNET_DIR}/ThirdParty/spdlog)


add_library(${PROJECT_NAME} SHARED ${NMEA_SOURCE_DIR}/context.c ${NMEA_SOURCE_DIR}/generate.c ${NMEA_SOURCE_DIR}/generator.c ${NMEA_SOURCE_DIR}/gmath.c ${NMEA_SOURCE_DIR}/info.c ${NMEA_SOURCE_DIR}/parse.c ${NMEA_SOURCE_DIR}/parser.c ${NMEA_SOURCE_DIR}/sentence.c ${NMEA_SOURCE_DIR}/nmeatime.c ${NMEA_SOURCE_DIR}/tok.c)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib/")


find_package(GeographicLib REQUIRED)


target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_BINARY_DIR}"
PUBLIC ${NMEA_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} 
PUBLIC m
)
