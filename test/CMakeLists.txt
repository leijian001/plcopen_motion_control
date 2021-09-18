# Find GTEST
find_package(GTest REQUIRED)
message(STATUS "GTEST_INCLUDE_DIRS: ${GTEST_INCLUDE_DIRS}")
message(STATUS "GTEST_BOTH_LIBRARIES: ${GTEST_BOTH_LIBRARIES}")
# Include gtest headers
include_directories(
  ${GTEST_INCLUDE_DIRS}
)

# Create scurve test executable
add_executable(offline_scurve_test offline_scurve_test.cpp)
target_link_libraries(offline_scurve_test ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${PYTHON_LIBRARIES} -lpthread)

# Create scurve test executable
add_executable(online_scurve_test online_scurve_test.cpp)
target_link_libraries(online_scurve_test ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${PYTHON_LIBRARIES} ruckig::ruckig -lpthread)

# Create planner test executable
add_executable(planner_test planner_test.cpp)
target_link_libraries(planner_test ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${PYTHON_LIBRARIES} ruckig::ruckig -lpthread)

# Create function block test executable
add_executable(function_block_test function_block_test.cpp)
target_link_libraries(function_block_test ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} ${PYTHON_LIBRARIES} ruckig::ruckig -lpthread)

# Install test executables
install(TARGETS offline_scurve_test online_scurve_test planner_test function_block_test
        RUNTIME DESTINATION ${INSTALL_BINDIR}
)

if(URANUS)
  # Find URANUS
  find_library(
    URANUS_LIBRARIES
    NAMES uranus
    HINTS ${URANUS_DIR}/build)
  if (NOT URANUS_LIBRARIES)
    message(ERROR " URANUS library not found under: ${URANUS_DIR}/build")
  endif()

  # Include uranus headers
  include_directories(
    ${URANUS_DIR}/install/include/motion
    ${URANUS_DIR}/install/include/fb
    ${URANUS_DIR}/install/include/misc
  )

  # Create uranus test executable
  add_executable(uranus_test uranus_test.cpp)
  target_link_libraries(uranus_test ${PROJECT_NAME} ${URANUS_LIBRARIES} ${GTEST_LIBRARIES})

  # Install uranus test executable
  install(TARGETS uranus_test
          RUNTIME DESTINATION ${INSTALL_BINDIR}
  )
endif(URANUS)