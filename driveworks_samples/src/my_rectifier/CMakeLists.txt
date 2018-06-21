# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

project(my_rectifier C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    main.cpp
)

if(VIBRANTE)
    set(SOURCES
        ${SOURCES}
    )
endif()

set(LIBRARIES
    dw_samples_framework
    ${Driveworks_LIBRARIES}
    lodepng
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_add_sample(${PROJECT_NAME})
