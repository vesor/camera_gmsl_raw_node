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

// Driveworks
#include <dw/Driveworks.h>

// Sample framework
#include <framework/DriveWorksSample.hpp>
#include <framework/SimpleStreamer.hpp>
#include <framework/SimpleRenderer.hpp>
#include <framework/SimpleCamera.hpp>
#include <framework/MathUtils.hpp>

// GLFW keys
#include <GLFW/glfw3.h>

// Screenshot output
#include <lodepng.h>

/**
 * Class that holds functions and variables common to all samples
 */
using namespace dw_samples::common;

class RectifierApp : public DriveWorksSample
{
public:
    // ------------------------------------------------
    // Sample constants
    // ------------------------------------------------
    static const uint32_t NUM_CAMERAS      = 1;
    static const uint32_t FRAME_WIDTH      = 1280;
    static const uint32_t FRAME_HEIGHT     = 800;
    static const uint32_t FPS              = 30;

    // ------------------------------------------------
    // Driveworks context and modules
    // ------------------------------------------------
    dwContextHandle_t           m_context                       = DW_NULL_HANDLE;
    dwSALHandle_t               m_sal                           = DW_NULL_HANDLE;
    dwSensorHandle_t            m_cameraSensor                  = DW_NULL_HANDLE;
    dwCameraRigHandle_t         m_rig                           = DW_NULL_HANDLE;
    dwCalibratedCameraHandle_t  m_cameraModelIn[NUM_CAMERAS]    ={DW_NULL_HANDLE};
    dwCalibratedCameraHandle_t  m_cameraModelOut                = DW_NULL_HANDLE;
    dwImageStreamerHandle_t     m_streamerCUDA2GL               = DW_NULL_HANDLE;
    dwImageStreamerHandle_t     m_streamerCUDA2CPU              = DW_NULL_HANDLE;
    dwRectifierHandle_t         m_rectifier                     = DW_NULL_HANDLE;
    dwRendererHandle_t          m_renderer                      = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Variables
    // ------------------------------------------------
    dwImageCUDA m_rectifiedImage{};
    uint32_t    m_cameraWidth       = 0U;
    uint32_t    m_cameraHeight      = 0U;
    uint32_t    m_cameraCount       = 0U;
    dwPinholeCameraConfig m_cameraConfig;
    uint32_t    m_cameraIdx         = 0;
    uint32_t    m_screenshotCount   = 0;
    bool        m_takeScreenshot;
    bool        m_pause;
    bool        m_dump;

    std::string m_rigConfigFilename;
    std::string m_videoFilename;

    std::unique_ptr<SimpleCamera> m_camera;
    dwImageGeneric *m_image;

    // ------------------------------------------------
    // Sample constructor and methods
    // ------------------------------------------------
    RectifierApp(const ProgramArguments& args);

    // Sample framework
    bool onInitialize() override final;
    void onProcess() override final;
    void onRender() override final;
    void onProcessKey(int key) override final;
    void onReset() override final;
    void onRelease() override final;

    // Sample initialization
    bool initDriveworks();
    bool initCameras();
    bool initImageStreamer();
    bool initRenderer();
    bool initRectifier();
    bool createVideoReplay(dwSensorHandle_t *salSensor,
                           uint32_t *cameraWidth,
                           uint32_t *cameraHeight,
                           uint32_t *cameraSiblings,
                           float32_t *cameraFrameRate,
                           dwImageType *imageType,
                           dwSALHandle_t sal,
                           const std::string &videoFName);
    void createOutputCUDAImage(dwImageCUDA* output);
    void releaseOutputCUDAImage(dwImageCUDA* output);

    void setRendererRect(int x, int y);
    void takeScreenshot();

};

//#######################################################################################
RectifierApp::RectifierApp(const ProgramArguments& args)
    : DriveWorksSample(args)
{
    m_rigConfigFilename = args.get("rig");
    m_videoFilename     = m_args.get("video");
    m_cameraIdx         = atoi(args.get("cameraIdx").c_str());
    m_dump              = args.get("dumpToFile").compare("1") == 0;
    m_takeScreenshot    = false;
    m_pause    = false;
}

//#######################################################################################
bool RectifierApp::onInitialize()
{

    if(!initDriveworks()) {
        logError("DriveWorks initialization failed");
        return false;
    }

    if(!initCameras()) {
        logError("Camera initialization failed");
        return false;
    }

    if(!initImageStreamer()) {
        logError("ImageStreamer initialization failed");
        return false;
    }

    if(!initRenderer()) {
        logError("Renderer initialization failed");
        return false;
    }

    if(!initRectifier()) {
        logError("Rectifier initialization failed");
        return false;
    }

    // Limit FPS
    DriveWorksSample::setProcessRate(FPS);

    return true;
}

//#######################################################################################
void RectifierApp::onProcess()
{
    if (m_pause) {
        return;
    }

    // Read frame
    m_image = m_camera->readFrame();
    if(m_image != nullptr) {
        // Get image
        dwImageCUDA *rgbaImage = GenericImage::toDW<dwImageCUDA>(m_image);

        // Rectify image
        dwStatus status = dwRectifier_warp(&m_rectifiedImage, rgbaImage, m_rectifier);
        
        if(status == DW_SUCCESS) {
            if (m_dump || m_takeScreenshot) {
                takeScreenshot();
            }
        }
        else {
            logError("Cannot unwarp: %s\n", dwGetStatusName(status));
            DriveWorksSample::stop();
        }
        //m_rectifiedImage = *rgbaImage;
    }
    else {
        log("Camera reached end of stream\n");
        DriveWorksSample::reset();
    }
}

//#######################################################################################
void RectifierApp::onRender()
{
    dwImageGL *frameGL = m_camera->getFrameRgbaGL();

    // Render input image
    setRendererRect(0, 0);
    dwRenderer_renderTexture(frameGL->tex, frameGL->target, m_renderer);
    dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
    dwRenderer_renderText(20, 20, "Input", m_renderer);

    // Render output image
    dwImageGL *frameGLOut;
    dwImageStreamer_postCUDA(&m_rectifiedImage, m_streamerCUDA2GL);
    dwStatus status = dwImageStreamer_receiveGL(&frameGLOut, 500000, m_streamerCUDA2GL);

    if (status != DW_SUCCESS) {
        logError("Did not receive GL frame within 500ms");
    }
    else {
        // set render rectangle
        setRendererRect(DriveWorksSample::getWindowWidth()/2, 0);

        // render received texture
        dwRenderer_renderTexture(frameGLOut->tex, frameGLOut->target, m_renderer);

        // overlay text
        dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, m_renderer);
        dwRenderer_renderText(20, 20, "Rectified", m_renderer);

        // return frame
        dwImageStreamer_returnReceivedGL(frameGLOut, m_streamerCUDA2GL);
    }

    dwImageCUDA *processedCUDA;
    status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2GL);

    if (status != DW_SUCCESS || processedCUDA != &m_rectifiedImage) {
        logError("Consumer did not give frame back");
    }

    CHECK_GL_ERROR();

    // Release frame
    m_camera->releaseFrame();
}

//#######################################################################################
void RectifierApp::onProcessKey(int key)
{
    // take screenshot
    if (key == GLFW_KEY_S) {
        m_takeScreenshot = true;
    }
    else if (key == GLFW_KEY_SPACE) {
        m_pause = !m_pause;
    }
}

//#######################################################################################
void RectifierApp::onReset()
{
    // reset camera
    m_camera->resetCamera();
}

//#######################################################################################
void RectifierApp::onRelease()
{
    releaseOutputCUDAImage(&m_rectifiedImage);
    m_camera.reset();

    dwStatus status = dwSensor_stop(m_cameraSensor);
    if (status != DW_SUCCESS) {
        logError("Cannot stop sensor: %s\n", dwGetStatusName(status));
    }

    status = dwSAL_releaseSensor(&m_cameraSensor);

    if (status != DW_SUCCESS) {
        logError("Cannot release sensor: %s\n", dwGetStatusName(status));
    }

    status = dwRenderer_release(&m_renderer);
    if (status != DW_SUCCESS) {
        logError("Cannot release renderer: %s\n", dwGetStatusName(status));
    }

    for (uint32_t i = 0; i < NUM_CAMERAS; i ++){
        status = dwCalibratedCamera_release(&m_cameraModelIn[i]);
        if(status != DW_SUCCESS) {
            logError("Cannot release camera (%d) : %s\n", i, dwGetStatusName(status));
        }
    }

    status = dwCalibratedCamera_release(&m_cameraModelOut);
    if (status != DW_SUCCESS) {
        logError("Cannot release camera out: %s\n", dwGetStatusName(status));
    }

    status = dwCameraRig_release(&m_rig);
    if (status != DW_SUCCESS) {
        logError("Cannot release camera rig: %s\n", dwGetStatusName(status));
    }

    status = dwImageStreamer_release(&m_streamerCUDA2CPU);
    if (status != DW_SUCCESS) {
        logError("Cannot release CUDA2CPU streamer: %s\n", dwGetStatusName(status));
    }
    status = dwImageStreamer_release(&m_streamerCUDA2GL);
    if (status != DW_SUCCESS) {
        logError("Cannot release CUDA2GL streamer: %s\n", dwGetStatusName(status));
    }

    status = dwRectifier_release(&m_rectifier);
    if (status != DW_SUCCESS) {
        logError("Cannot release rectifier: %s\n", dwGetStatusName(status));
    }

    status = dwSAL_release(&m_sal);
    if (status != DW_SUCCESS) {
        logError("Cannot release SAL: %s\n", dwGetStatusName(status));
    }

    status = dwRelease(&m_context);
    if (status != DW_SUCCESS) {
        logError("Cannot release DriveWorks context: %s\n", dwGetStatusName(status));
    }
}

//#######################################################################################
bool RectifierApp::initDriveworks()
{
    dwStatus status;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    status = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (status != DW_SUCCESS) {
        logError("Cannot create logger: %s\n", dwGetStatusName(status));
        return false;
    }

    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

#ifdef VIBRANTE
    sdkParams.eglDisplay = DriveWorksSample::m_window->getEGLDisplay();
#endif

    status = dwInitialize(&m_context, DW_VERSION, &sdkParams);
    if (status != DW_SUCCESS) {
        logError("Cannot create DriveWorks context: %s\n", dwGetStatusName(status));
        return false;
    }

    // create sensor abstraction layer
    status = dwSAL_initialize(&m_sal, m_context);
    if (status != DW_SUCCESS) {
        logError("Cannot create DriveWorks SAL: %s\n", dwGetStatusName(status));
        return false;
    }

    return true;
}

//#######################################################################################
bool RectifierApp::initCameras()
{
    // create GMSL Camera interface
    uint32_t cameraSiblings   = 0U;
    float32_t cameraFramerate = 0.0f;
    dwImageType imageType;

    bool status = createVideoReplay(&m_cameraSensor, &m_cameraWidth, &m_cameraHeight, &cameraSiblings,
                                    &cameraFramerate, &imageType, m_sal, m_videoFilename);

    if (status != true) {
        logError("Cannot create video replay");
        return false;
    }

    std::cout << "Camera image with " << m_cameraWidth << "x" << m_cameraHeight << " at "
              << cameraFramerate << " FPS" << std::endl;
    
    dwStatus dw_status;
    dw_status = dwSensor_start(m_cameraSensor);

    if(dw_status != DW_SUCCESS) {
        logError("Cannot start camera: %s\n", dwGetStatusName(dw_status));
        return false;
    }

    // initialize input camera models from rig
    dwRigConfigurationHandle_t rigConf = DW_NULL_HANDLE;

    // load vehicle configuration
    status = dwRigConfiguration_initializeFromFile(&rigConf, m_context, m_rigConfigFilename.c_str());
    if (status != DW_SUCCESS) {
        logError("Cannot load vehicle rig configuration from %s: %s\n",
                 m_rigConfigFilename.c_str(), dwGetStatusName(dw_status));
        return false;
    }

    // camera rig
    status = dwCameraRig_initializeFromConfig(&m_rig, &m_cameraCount, m_cameraModelIn, NUM_CAMERAS,
                                              m_context, rigConf);

    if (m_cameraCount > 0) {
        dwStatus ph_status = dwRigConfiguration_getPinholeCameraConfig(&m_cameraConfig, 0, rigConf);
        if (DW_SUCCESS != ph_status)
        {
            logError("Cannot load pinhole configuration from %s: %s\n",
                 m_rigConfigFilename.c_str(), dwGetStatusName(ph_status));
            return false;
        }
    }


    dwRigConfiguration_release(&rigConf);

    if (status != DW_SUCCESS) {
        logError("Cannot initialize cameras from rig: %s\n", dwGetStatusName(dw_status));
        return false;
    }

    if ((m_cameraIdx >= m_cameraCount) || (m_cameraIdx < 0)) {
        logError("Invalid cameraIdx. Check your rig.xml file.");
        return false;
    }

    // initialize input camera
    dwSensorParams params;
    std::string arguments = "video=" + m_videoFilename;
    params.parameters = arguments.c_str();
    params.protocol   = "camera.virtual";

    dwImageProperties props{};
    props.type = DW_IMAGE_CUDA;
    props.pxlType = DW_TYPE_UINT8;
    props.pxlFormat = DW_IMAGE_RGBA;
    props.planeCount = 1;
    props.width = m_cameraWidth;
    props.height = m_cameraHeight;

    m_camera = std::unique_ptr<SimpleCamera>(new SimpleCamera(props, params, m_sal, m_context));
    m_camera->enableGLOutput();

    // initialize output camera model as simple pinhole
    dwPinholeCameraConfig cameraConf = {};
    cameraConf.distortion[0] = 0.f;
    cameraConf.distortion[1] = 0.f;
    cameraConf.distortion[2] = 0.f;

    cameraConf.u0 = static_cast<float32_t>(m_cameraWidth/2);
    cameraConf.v0 = static_cast<float32_t>(m_cameraHeight/2);
    cameraConf.width = m_cameraWidth;
    cameraConf.height = m_cameraHeight;

    cameraConf.focalX = m_cameraConfig.focalX;
    cameraConf.focalY = m_cameraConfig.focalY;

    status = dwCalibratedCamera_initializePinhole(&m_cameraModelOut, m_context, &cameraConf);

    if(status != DW_SUCCESS) {
        logError("Cannot initialize pinhole camera: %s\n", dwGetStatusName(dw_status));
        return false;
    }
    else {
        std::cout << "Created pinhole camera: focal length " << cameraConf.focalX << ", "
         << cameraConf.focalY << std::endl; 
    }

    return true;
}

//#######################################################################################
bool RectifierApp::initImageStreamer()
{
    //for frame dump and screenshot
    dwImageProperties props{};
    props.type = DW_IMAGE_CUDA;
    props.pxlType = DW_TYPE_UINT8;
    props.pxlFormat = DW_IMAGE_RGBA;
    props.planeCount = 1;
    props.width = m_cameraWidth;
    props.height = m_cameraHeight;

    dwStatus status;
    status = dwImageStreamer_initialize(&m_streamerCUDA2CPU, &props, DW_IMAGE_CPU, m_context);
    if (status != DW_SUCCESS) {
        logError("Cannot init CUDA2CPU image streamer: %s\n", dwGetStatusName(status));
        return false;
    }

    status = dwImageStreamer_initialize(&m_streamerCUDA2GL, &props, DW_IMAGE_GL, m_context);
    if (status != DW_SUCCESS) {
        logError("Cannot init CUDA2GL image streamer: %s\n", dwGetStatusName(status));
        return false;
    }

    return true;
}

//#######################################################################################
bool RectifierApp::initRenderer()
{
    dwStatus status;

    status = dwRenderer_initialize(&m_renderer, m_context);
    if (status != DW_SUCCESS) {
        logError("Cannot init renderer: %s\n", dwGetStatusName(status));
        return false;
    }

    return true;
}

//#######################################################################################
bool RectifierApp::initRectifier()
{
    dwStatus status = dwRectifier_initialize(&m_rectifier, m_cameraModelIn[m_cameraIdx],
                                             m_cameraModelOut, m_context);

    if (status != DW_SUCCESS) {
        logError("Cannot initialize rectifier: %s\n", dwGetStatusName(status));
        return false;
    }

    createOutputCUDAImage(&m_rectifiedImage);
    return true;
}

//#######################################################################################
bool RectifierApp::createVideoReplay(dwSensorHandle_t *salSensor,
                                     uint32_t *cameraWidth,
                                     uint32_t *cameraHeight,
                                     uint32_t *cameraSiblings,
                                     float32_t *cameraFrameRate,
                                     dwImageType *imageType,
                                     dwSALHandle_t sal,
                                     const std::string &videoFName)
{
    dwStatus status;
    std::string arguments = "video=" + videoFName;

    dwSensorParams params{};
    params.parameters = arguments.c_str();
    params.protocol   = "camera.virtual";
    status = dwSAL_createSensor(salSensor, params, sal);
    if (status != DW_SUCCESS) {
        logError("Cannot create sensor: %s\n", dwGetStatusName(status));
        return false;
    }

    dwImageProperties cameraImageProperties{};
    status = dwSensorCamera_getImageProperties(&cameraImageProperties,
                                               DW_CAMERA_PROCESSED_IMAGE,
                                               *salSensor);
    if (status != DW_SUCCESS) {
        logError("Cannot get camera image properties: %s\n", dwGetStatusName(status));
        return false;
    }

    dwCameraProperties cameraProperties{};
    status = dwSensorCamera_getSensorProperties(&cameraProperties, *salSensor);
    if (status != DW_SUCCESS) {
        logError("Cannot get camera sensor properties: %s\n", dwGetStatusName(status));
        return false;
    }

    *cameraWidth = cameraImageProperties.width;
    *cameraHeight = cameraImageProperties.height;
    *imageType = cameraImageProperties.type;
    *cameraFrameRate = cameraProperties.framerate;
    *cameraSiblings = cameraProperties.siblings;

    return true;
}

//#######################################################################################
void RectifierApp::createOutputCUDAImage(dwImageCUDA* output)
{
    cudaMallocPitch(&output->dptr[0], &output->pitch[0], m_cameraWidth*4, m_cameraHeight);
    output->layout = DW_IMAGE_CUDA_PITCH;
    output->prop.height = static_cast<int32_t>(m_cameraHeight);
    output->prop.width = static_cast<int32_t>(m_cameraWidth);
    output->prop.planeCount = 1;
    output->prop.pxlFormat = DW_IMAGE_RGBA;
    output->prop.pxlType = DW_TYPE_UINT8;
    output->prop.type = DW_IMAGE_CUDA;
}

//#######################################################################################
void RectifierApp::releaseOutputCUDAImage(dwImageCUDA* output)
{
    cudaFree(output->dptr[0]);
}

//#######################################################################################
void RectifierApp::setRendererRect(int x, int y)
{
    dwRect rectangle{};
    rectangle.width = DriveWorksSample::getWindowWidth()/2;
    rectangle.height = DriveWorksSample::getWindowHeight();
    rectangle.x = x;
    rectangle.y = y;

    dwRenderer_setRect(rectangle, m_renderer);
}

//#######################################################################################
void RectifierApp::takeScreenshot()
{
    dwImageStreamer_postCUDA(&m_rectifiedImage, m_streamerCUDA2CPU);
    dwImageCPU *imageCPU;
    dwImageStreamer_receiveCPU(&imageCPU, 60, m_streamerCUDA2CPU);

    char fname[128];
    sprintf(fname, "screenshot_%04u.png", m_screenshotCount++);
    lodepng_encode32_file(fname, imageCPU->data[0],
                          imageCPU->prop.width, imageCPU->prop.height);
    log("Screenshot taken to %s\n", fname);
    m_takeScreenshot = false;

    dwImageStreamer_returnReceivedCPU(imageCPU, m_streamerCUDA2CPU);
    dwImageCUDA *processedCUDA;
    dwStatus status = dwImageStreamer_waitPostedCUDA(&processedCUDA, 30000, m_streamerCUDA2CPU);
    if (status != DW_SUCCESS || processedCUDA != &m_rectifiedImage) {
        logError("Consumer did not give frame back");
    }
}

//#######################################################################################
int main(int argc, const char **argv)
{
    // define all arguments used by the application
    const ProgramArguments arguments = ProgramArguments(argc, argv,
        {
            ProgramArguments::Option_t("rig", (DataPath::get() + "/samples/sfm/triangulation/rig.xml").c_str()),
            ProgramArguments::Option_t("video", (DataPath::get() +
                                                 std::string{"/samples/sfm/triangulation/video_0.h264"}).c_str()),
            ProgramArguments::Option_t("cameraIdx", "0"),
            ProgramArguments::Option_t("dumpToFile", "0"),
        }, "The Video Rectification sample demonstrates how to remove fisheye distortion from a video captured "
        "on a camera with a fisheeye lens.");

    // Window/GL based application
    RectifierApp app(arguments);
    app.initializeWindow("Sample Video Rectifier",
                         RectifierApp::FRAME_WIDTH, RectifierApp::FRAME_HEIGHT/2,
                         arguments.enabled("offscreen"));
    return app.run();
}
