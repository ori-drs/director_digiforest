message(STATUS "to $USE_DRC " ${USE_DRC})
message(STATUS "to $USE_COLLECTIONS " ${USE_COLLECTIONS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(OpenGL REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(sources
  vtkDepthImageUtils.cxx
  vtkGridSource.cxx
  vtkFrameWidget.cxx
  vtkFrameWidgetRepresentation.cxx
  vtkInteractorStyleTerrain2.cxx
  vtkPickCenteredInteractorStyle.cxx
  )

# extra source files to compile but do not python wrap
set(EXTRA_SRCS
  )

#####

set(GLSL_RESOURCES_DIR
    edl_resources/Shaders
    )

set(GLSL_SRCS
  edl_compose
  edl_shade
  bilateral_filter
  depth_compose
  )

# -----------------------------------------------------------------------------
# Create custom commands to encode each glsl file into a C string literal
# in a header file
# -----------------------------------------------------------------------------
if(NOT TARGET vtkEncodeString)
  message(SEND_ERROR "The required vtkEncodeString executable was not found.\nThis may indicate an error in your VTK install or configuration.")
endif()

foreach(file ${GLSL_SRCS})
  set(src ${CMAKE_CURRENT_SOURCE_DIR}/${GLSL_RESOURCES_DIR}/${file}.glsl)
  set(res ${CMAKE_CURRENT_BINARY_DIR}/${file}.cxx)
  add_custom_command(
    OUTPUT ${res}
    DEPENDS ${src}
    COMMAND vtkEncodeString
    ARGS ${res} ${src} ${file}
    )
  set(EXTRA_SRCS ${EXTRA_SRCS} ${res})
endforeach(file)


#####

set(VTK_LIBRARIES
  vtkRenderingFreeTypeFontConfig
  vtkInteractionWidgets
  vtkInteractionStyle
  vtkRenderingFreeType
  vtkRenderingCore
  vtkRendering${VTK_RENDERING_BACKEND}
  vtkFiltersExtraction
  vtkFiltersCore
  vtkFiltersParallelDIY2
  vtkCommonExecutionModel
  vtkWrappingPythonCore
  vtkCommonExecutionModelPythonD
  vtkInteractionWidgets
  vtkCommonCore
  vtkWrappingPythonCore
  vtkCommonCorePythonD
  vtkInteractionWidgetsPythonD
  )


set(deps
  ${VTK_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )

set(pkg_deps)

find_package(catkin  REQUIRED)

list(APPEND deps
  ${catkin_LIBRARIES}
)

include_directories(${catkin_INCLUDE_DIRS})


if (USE_DRC_PLANE_SEG)
  find_package(PlaneSeg REQUIRED)
  find_package(PCL REQUIRED)

  list(APPEND sources
    vtkPlaneSegmentation.cxx
    vtkRobustNormalEstimator.cxx
    vtkSurfaceFitter.cxx
  )

  include_directories(${PLANE_SEG_INCLUDE_DIRS})
  include_directories(${PCL_INCLUDE_DIRS})
  list(APPEND deps ${PLANE_SEG_LIBRARIES} ${PCL_LIBRARIES})
endif()

set(library_name vtkDRCFilters)
add_library(${library_name} ${sources} ${EXTRA_SRCS})
target_link_libraries(${library_name} ${deps})

if(pkg_deps)
  use_pkg(${library_name} ${pkg_deps})
endif()

install(TARGETS ${library_name}
    EXPORT ${DD_TARGETS_NAME}
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

if(NOT VTK_WRAP_PYTHON)
  message(FATAL_ERROR "VTK was compiled without Python support. "
    "VTK_WRAP_PYTHON is OFF. You must use a version of VTK with Python. "
    "VTK_DIR is: ${VTK_DIR}")
endif()

set(VTK_LIBRARIES)

include_directories(${PYTHON_INCLUDE_DIR})
include(${CMAKE_DIRECTOR_SOURCE_DIR}/cmake/wrap-python.cmake)
wrap_python(${library_name} "${sources}")
set_target_properties(${library_name}Python
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${DD_INSTALL_PYTHON_DIR}/director")
set_target_properties(${library_name}Python
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${DD_INSTALL_PYTHON3_DIR}")
target_link_libraries(${library_name}Python ${PYTHON_LIBRARIES})
target_link_libraries(${library_name}PythonD ${PYTHON_LIBRARIES})
