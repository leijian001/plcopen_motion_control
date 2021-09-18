# Add source files
aux_source_directory(algorithm SOURCE)
aux_source_directory(axis SOURCE)
aux_source_directory(fb SOURCE)
aux_source_directory(servo SOURCE)
aux_source_directory(tool SOURCE)

# Create "RTmotion" library
add_library(${PROJECT_NAME} SHARED ${SOURCE})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-variable)
if(PLOT)
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PYTHON_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ruckig::ruckig)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE ruckig::ruckig)
endif(PLOT)

# Create single axis demo executable
add_executable(single_axis_move_relative demo/RT_single_axis_move_relative.cpp)
target_link_libraries(single_axis_move_relative ${PROJECT_NAME} ruckig::ruckig -lpthread)

add_executable(axis_move_cyclic demo/single_axis_move_relative.cpp)
target_link_libraries(axis_move_cyclic ${PROJECT_NAME} ruckig::ruckig -lpthread)

add_executable(multi-axis demo/multi-axis.cpp)
target_link_libraries(multi-axis ${PROJECT_NAME} ruckig::ruckig -lpthread)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  TARGETS single_axis_move_relative axis_move_cyclic
  DESTINATION ${INSTALL_BINDIR}
)
