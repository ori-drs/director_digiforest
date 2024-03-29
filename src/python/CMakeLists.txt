
set(python_files
  director/__init__.py
  director/actionhandlers.py
  director/affordanceupdater.py
  director/affordanceitems.py
  director/affordancemanager.py
  director/affordancepanel.py
  director/affordanceviewbehaviors.py
  director/applogic.py
  director/appsettings.py
  director/asynctaskqueue.py
  director/callbacks.py
  director/camerabookmarks.py
  director/cameracontrol.py
  director/cameracontrolpanel.py
  director/cameraview.py
  director/componentgraph.py
  director/consoleapp.py
  director/contextviewbehaviors.py
  director/debugpolydata.py
  director/drcargs.py
  director/frameupdater.py
  director/fieldcontainer.py
  director/filterUtils.py
  director/framevisualization.py
  director/geometryencoder.py
  director/imageview.py
  director/inputdialog.py
  director/ioutils.py
  director/mainwindowapp.py
  director/measurementpanel.py
  director/meshmanager.py
  director/objectcollection.py
  director/objectmodel.py
  director/opendatahandler.py
  director/outputconsole.py
  director/perceptionmeta.py
  director/pointpicker.py
  director/pointselector.py
  director/propertyanimation.py
  director/propertyset.py
  director/qtutils.py
  director/screengrabberpanel.py
  director/segmentation.py
  director/segmentationroutines.py
  director/shallowCopy.py
  director/simpletimer.py
  director/skybox.py
  director/startup.py
  director/tasklaunchpanel.py
  director/timercallback.py
  director/transformUtils.py
  director/uipanel.py
  director/utime.py
  director/uuidutil.py
  director/version.py.in
  director/viewbehaviors.py
  director/viewcolors.py
  director/viewframes.py
  director/vieweventfilter.py
  director/visualization.py
  director/vtkAll.py
  director/vtkNumpy.py

  director/tasks/__init__.py
  director/tasks/taskmanagerwidget.py
  director/tasks/taskuserpanel.py

  director/thirdparty/__init__.py
  director/thirdparty/qhull_2d.py
  director/thirdparty/min_bounding_rect.py
  director/thirdparty/NumericStringParser.py
  director/thirdparty/numpyjsoncoder.py
  director/thirdparty/transformations.py
  director/thirdparty/conversions.py
  director/thirdparty/geometry_msgs_mock.py
  director/thirdparty/naming.py
  director/thirdparty/pysdf.py
  director/thirdparty/toposort.py
  
  director/digiforest/coordinatesconverter.py
  director/digiforest/forestpayloadspanel.py
  director/digiforest/imageloader.py
  director/digiforest/objectpicker.py
  director/digiforest/posegraphloader.py
  director/digiforest/tilepicker.py
  director/digiforest/utils.py
  director/digiforest/heightmapper.py
  )


set(python_module_dir "${CATKIN_DEVEL_PREFIX}/${DD_INSTALL_PYTHON_DIR}")
# Copy python files
set(copied_python_files)
foreach(file ${python_files})
  set(src "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
  set(tgt "${python_module_dir}/${file}")
  get_filename_component(tgtDir ${tgt} PATH)
  get_filename_component(tgtExt ${tgt} EXT)
  get_filename_component(tgt_we ${tgt} NAME_WE)

  file(MAKE_DIRECTORY ${tgtDir})

  if(${tgtExt} STREQUAL ".py.in")
    set(tgt "${tgtDir}/${tgt_we}.py")
    configure_file(${src} ${tgt} @ONLY)
  else()
    add_custom_command(DEPENDS ${src}
                       COMMAND ${CMAKE_COMMAND} -E copy ${src} ${tgt}
                       OUTPUT ${tgt}
                       COMMENT "source copy")
  endif()


  set(copied_python_files ${copied_python_files} ${tgt})
  get_filename_component(installDirSuffix ${file} PATH)
  install(FILES ${tgt} ${tgtDir}/${tgt_we}.pyc DESTINATION ${DD_INSTALL_PYTHON_DIR}/${installDirSuffix} COMPONENT Runtime)
endforeach(file)


# Byte compile python files
set(compile_all_script "${CMAKE_CURRENT_BINARY_DIR}/compile_all.py")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/compile_all.py.in"
               ${compile_all_script}
               @ONLY IMMEDIATE)

add_custom_command(
  COMMAND ${PYTHON_EXECUTABLE}
  ARGS  "${compile_all_script}"
  DEPENDS ${copied_python_files}  ${compile_all_script}
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/compile_complete"
  )

add_custom_target(python_byte_compile ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/compile_complete")

add_subdirectory(extensions)
add_subdirectory(tests)
