#
# for SiftGPU
#

# Copyright (c) 2016 Yong Zhao, <zd5945@126.com>
# Copyright (c) 2016 Shuhui Bu, <bushuhui@gmail.com>


FIND_PATH(SiftGPU_INCLUDE_DIR SiftGPU/SiftGPU.h
	# installation selected by user
	$ENV{SIFTGPU_PATH}/src
    ${PROJECT_SOURCE_DIR}/ThirdParty/SiftGPU/src
    ${PROJECT_SOURCE_DIR}/ThirdParty/SiftGPU-V400/src
)

if(WIN32)
set(SIFTGPU_LIBS ${PROJECT_SOURCE_DIR}/ThirdParty/SiftGPU-V400/bin/libsiftgpu.so)
else()
set(SIFTGPU_LIBS ${PROJECT_SOURCE_DIR}/ThirdParty/SiftGPU-V400/bin/libsiftgpu.so)
endif()

if(SiftGPU_INCLUDE_DIR)
	set(SiftGPU_INCLUDES ${SiftGPU_INCLUDE_DIR})
  set(SIFTGPU_FOUND true)
endif(SiftGPU_INCLUDE_DIR)

