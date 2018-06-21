# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

project(lib_camera_gmsl_raw C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    main.cpp
)

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
    lodepng
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
#add_executable(${PROJECT_NAME} ${SOURCES})
cuda_add_library(${PROJECT_NAME} ${SOURCES} SHARED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

