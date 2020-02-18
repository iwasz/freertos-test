CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET (CMAKE_VERBOSE_MAKEFILE OFF)

SET(CMAKE_C_FLAGS "-std=gnu99 -Wall" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall" CACHE INTERNAL "cxx compiler flags")

PROJECT (unit-tests)

ADD_DEFINITIONS ("-DUNIT_TEST")

INCLUDE_DIRECTORIES("../../etl/include/" "etl-x86")

SET (CATCH2_INCLUDE "../../deps/Catch2/single_include/catch2")
INCLUDE_DIRECTORIES("../../deps/etl/include/" )
INCLUDE_DIRECTORIES("${CATCH2_INCLUDE}")
INCLUDE_DIRECTORIES("../../src")

ADD_EXECUTABLE(unit-tests
    "${CATCH2_INCLUDE}/catch.hpp"
    "00CatchInit.cc"
    "01LoggingTest.cc"
)
