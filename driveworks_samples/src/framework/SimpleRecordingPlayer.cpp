/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "SimpleRecordingPlayer.hpp"
#include <sstream>
#include <algorithm>

namespace dw_samples
{
namespace common
{

void SimpleRecordingPlayer::restart()
{
    m_isPendingCANMsgValid = false;
    if(m_canSensor)
        CHECK_DW_ERROR( dwSensor_reset(m_canSensor) );

    m_isPendingIMUMsgValid = false;
    if(m_imuSensor)
        CHECK_DW_ERROR( dwSensor_reset(m_imuSensor) );

    for(auto &data : m_cameras)
    {
        data.camera->resetCamera();
        data.pendingImage = nullptr;
    }
}

void SimpleRecordingPlayer::stepForward()
{
    dwStatus result;

    // Load from all sensors

    if(!m_isPendingCANMsgValid && m_canSensor != DW_NULL_HANDLE)
    {
        // Load CAN
        result = dwSensorCAN_readMessage(&m_pendingCANMsg, 100000, m_canSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingCANMsgValid = true;
            if(m_pendingCANMsg.timestamp_us == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: CAN msg has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "CAN reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read CAN frame: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    if(!m_isPendingIMUMsgValid && m_imuSensor != DW_NULL_HANDLE)
    {
        // Load IMU
        result = dwSensorIMU_readFrame(&m_pendingIMUMsg, 100000, m_imuSensor);
        if (result == DW_SUCCESS)
        {
            m_isPendingIMUMsgValid = true;
            if(m_pendingIMUMsg.timestamp_us == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: IMU msg has no timestamp. Playback sync will not work.");
        }
        else if (result == DW_END_OF_STREAM)
        {
            std::cout << "IMU reached end of stream." << std::endl;
            m_handler->handleEndOfStream();
            return;
        }
        else
        {
            std::stringstream ss;
            ss << "Terminating. Cannot read IMU frame: " << dwGetStatusName(result);
            throw std::runtime_error(ss.str());
        }
    }

    for(auto &data : m_cameras)
    {
        if(!data.pendingImage)
        {
            // Load image
            data.pendingImage = data.camera->readFrame();
            if(!data.pendingImage)
            {
                m_handler->handleEndOfStream();
                return;
            }

            if(GenericImage::getTimestamp(data.pendingImage) == 0 && !isSingleSensorPlayback())
                throw std::runtime_error("SimpleRecordingPlayer: Camera image has no timestamp. Playback sync will not work.");
        }
    }

    // Find smallest timestamp
    dwTime_t earliestTimestamp = std::numeric_limits<dwTime_t>::max();

    if(m_isPendingCANMsgValid && m_pendingCANMsg.timestamp_us < earliestTimestamp)
        earliestTimestamp = m_pendingCANMsg.timestamp_us;

    if(m_isPendingIMUMsgValid && m_pendingIMUMsg.timestamp_us < earliestTimestamp)
        earliestTimestamp = m_pendingIMUMsg.timestamp_us;

    for(auto &camera : m_cameras)
    {
        if(camera.pendingImage && GenericImage::getTimestamp(camera.pendingImage) < earliestTimestamp)
            earliestTimestamp = GenericImage::getTimestamp(camera.pendingImage);
    }

    // Send an event for the first sensor that matches the earliest timestamp
    if(m_isPendingCANMsgValid && m_pendingCANMsg.timestamp_us == earliestTimestamp)
    {
        // Process CAN
        m_lastCANMsg = m_pendingCANMsg;
        m_handler->handleCAN(m_lastCANMsg);
        m_isPendingCANMsgValid = false;
    }
    else if(m_isPendingIMUMsgValid && m_pendingIMUMsg.timestamp_us == earliestTimestamp)
    {
        // Process IMU
        m_lastIMUMsg = m_pendingIMUMsg;
        m_handler->handleIMU(m_lastIMUMsg);
        m_isPendingIMUMsgValid = false;
    }
    else
    {
        for(auto &camera : m_cameras)
        {

            if(camera.pendingImage && GenericImage::getTimestamp(camera.pendingImage) == earliestTimestamp)
            {
                // Process image
                size_t idx = &camera-&m_cameras[0];
                m_lastImages[idx] = camera.pendingImage;
                m_handler->handleCamera(idx, camera.pendingImage);
                camera.pendingImage = nullptr;
                break;
            }
        }
    }
}

}
}
