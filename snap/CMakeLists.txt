project(snap)

set(PYTHON_FILES
  viewer.py
  tool.py
  spline.py
  qt.py
  math.py
  gl.py
  __init__.py
  )

foreach(file ${PYTHON_FILES})
  message(${file} ${CMAKE_CURRENT_BINARY_DIR}/${file})
  configure_file(${file} ${CMAKE_CURRENT_BINARY_DIR}/${file} COPYONLY)
endforeach()

