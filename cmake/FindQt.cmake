find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})
if(WIN32)
	set(QT_LIBRARIES)
	set(QT_MODULES QtGui4 QtCore4 QtXml4 QtOpenGL4)
		#message("QT_LIBRARY_DIR:${QT_LIBRARY_DIR}")
	foreach(QT_MODULE ${QT_MODULES})
		find_library( ${QT_MODULE}_LIBRARIES NAMES "${QT_MODULE}"
			PATHS
			${QT_LIBRARY_DIR}
			)
		#message("${QT_MODULE}_LIBRARIES:${${QT_MODULE}_LIBRARIES}")
		list(APPEND QT_LIBRARIES ${${QT_MODULE}_LIBRARIES})
	endforeach()
else()

	set(QT_LIBRARIES QtGui QtCore QtXml QtOpenGL)
endif()
