cmake_minimum_required(VERSION 2.8.3)
project(oomact_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)



# enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas")

# enable C++11 support
if(CMAKE_VERSION VERSION_LESS "3.1")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

# This functions take TARGET_NAME PYTHON_MODULE_DIRECTORY sourceFile1 [sourceFile2 ...]
add_python_export_library(${PROJECT_NAME} python/oomact
  src/module.cpp
  src/Models.cpp
  src/Measurements.cpp
  src/Sensors.cpp
  src/Calibrators.cpp
)
target_link_libraries(${PROJECT_NAME})

cs_install()
cs_export()

catkin_python_setup()

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/oomact)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/tests.py
    DEPENDENCIES ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
endif()

