# G2O

IF(WIN32)
		message( STATUS "Looking for G2O or greater  - not found" )
		SET ( PIL_FOUND 0 )
ELSE(WIN32) # Linux

#	message("G2O path?=" ${PROJECT_SOURCE_DIR}/ThirdParty/g2o)
	FIND_PATH( G2O_PATH g2o/types/sim3/sim3.h
	${PROJECT_SOURCE_DIR}/ThirdParty/g2o
	/data/zhaoyong/Linux/Program/Apps/Thirdparty/orbslam/trunk/Thirdparty/g2o
	# installation selected by user
	$ENV{G2O_PATH}
	# system placed in /usr/local/include
	)

	if(G2O_PATH)
	    MESSAGE( STATUS "Found g2o at path " ${G2O_PATH})
	    set(G2O_INCLUDES ${G2O_PATH})
        
        set(G2O_MODULES2FIND g2o_core g2o_stuff g2o_types_sba g2o_types_sim3)
        
        foreach (G2O_MODULE_NAME ${G2O_MODULES2FIND})
		    FIND_LIBRARY(${G2O_MODULE_NAME}_LIBRARIES NAMES ${G2O_MODULE_NAME}
                    PATHS
			    	${G2O_PATH}/lib
			)
			
		    if(${G2O_MODULE_NAME}_LIBRARIES)
			    set(${G2O_MODULE_NAME}_INCLUDES ${G2O_INCLUDES})
			    set(${G2O_MODULE_NAME}_FOUND 1)
			    list(APPEND G2O_LIBRARIES ${${G2O_MODULE_NAME}_LIBRARIES})
		    else(${G2O_MODULE_NAME}_LIBRARIES)
			    message("Can't found module " ${G2O_MODULE_NAME})
		    endif(${G2O_MODULE_NAME}_LIBRARIES)
	    endforeach()
	
	endif(G2O_PATH)

	if( G2O_INCLUDES AND G2O_LIBRARIES)
		MESSAGE( STATUS "Looking for G2O or greater - found")
		SET ( G2O_FOUND 1 )
	else()
		message( STATUS "Looking for G2O or greater  - not found" )
		SET ( G2O_FOUND 0 )
	endif()
	
ENDIF(WIN32)
