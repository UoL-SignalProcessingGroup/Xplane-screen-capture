// Copyright (c) 2012, http://code.google.com/p/example-xplane-plugins/
// Copyright (c) 2021, University of Liverpool
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of "example-xplane-plugins" nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL "example-xplane-plugins" BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*------------------------------------------------------------------------*/
// INCLUDES
/*------------------------------------------------------------------------*/

#ifdef WIN32
// Can't use strcpy_s for copying to XPlane buffers of unknown length
#pragma warning(disable:4996)
// Required for OpenGL on Windows
#include <windows.h>
#endif

// OpenGL - for glReadPixels
#include <gl/gl.h>

// Boost - for shared memory comms
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

// XPlane - for drawing callback
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMCamera.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"

// Plugin - shared constants 
#include "XPScreenGrabConstants.h"

// Timer to avoid overcalling of "get pixels"
// The drawing callbacks may have multiple calls per frame!
#include "XPIntervalTimer.h"

/*------------------------------------------------------------------------*/
// NAMESPACES
/*------------------------------------------------------------------------*/

using namespace boost::interprocess;

/*------------------------------------------------------------------------*/
// STATIC CONSTANTS AND VARIABLES
/*------------------------------------------------------------------------*/

//----------------------------------------------------------------------
// WndProc replacement to enable XPlane window resizing
//----------------------------------------------------------------------
LONG_PTR s_originalProc(NULL);
LRESULT CALLBACK WndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
//----------------------------------------------------------------------

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

struct Plane
{
    float a, b, c, d;
};

struct Matrix4x4
{
    // The elements of the 4x4 matrix are stored in
    // column-major order (see "OpenGL Programming Guide",
    // 3rd edition, pp 106, glLoadMatrix).
    float _11, _21, _31, _41;
    float _12, _22, _32, _42;
    float _13, _23, _33, _43;
    float _14, _24, _34, _44;
};

// Handle to window to resize XPlane
static HWND s_hwnd(NULL);

static int s_original_width(SCREENSHOT_DEFAULT_WIDTH);
static int s_original_height(SCREENSHOT_DEFAULT_HEIGHT);
static bool s_pluginStarted = false;

// Performance timer & delays
static XPIntervalTimer s_timer;

// Shared memory
static mapped_region* s_region_control = NULL;
static named_mutex*   s_mutex_control = NULL;
static ControlData     s_controlData;

static mapped_region* s_region_screen = NULL;
static named_mutex*   s_mutex_screen = NULL;
static ScreenData     s_screenData;

static char error_buffer[MAX_ERROR_STRING_LENGTH] = { 0 };

// Dataref for setting viewport field of view in XPlane
static XPLMDataRef s_hFoV = NULL;
static XPLMDataRef s_vFoV = NULL;
static XPLMDataRef s_zulu = NULL;  // Dataref for setting time of day in XPlane
static XPLMDataRef s_aircraftOverride(NULL); // Dataref for disabling physics model in XPlane
static XPLMDataRef s_aircraftX(NULL);        // Dataref for setting X position in XPlane NED frame of reference  
static XPLMDataRef s_aircraftY(NULL);        // Dataref for setting Y position in XPlane NED frame of reference  
static XPLMDataRef s_aircraftZ(NULL);        // Dataref for setting Z position in XPlane NED frame of reference  
static XPLMDataRef s_aircraftPsi(NULL);      // Dataref for setting yaw in XPlane NED frame of reference  
static XPLMDataRef s_aircraftTheta(NULL);    // Dataref for setting pitch in XPlane NED frame of reference  
static XPLMDataRef s_aircraftPhi(NULL);      // Dataref for setting roll in XPlane NED frame of reference  

static int s_hasCamera(0);
static bool s_newCapture(false);
static long s_capture_number(0);

static const double RAD2DEG(180.0 / M_PI);
static const double DEG2RAD(M_PI / 180.0);

/*------------------------------------------------------------------------*/
// FILE SCOPE FUNCTION DECLARATIONS
/*------------------------------------------------------------------------*/

static void NormalizePlane(Plane& plane);
static void ExtractPlanesGL(Plane* p_planes, const Matrix4x4& comboMatrix, bool normalize);

// Create the shared memory object and mutex
static void CreateSharedState();
// Delete the shared memory object and mutex
static void DestroySharedState();
// Force XPlane window to match capture settings
static void EnforceWindowSize(int setwidth, int setheight);
// Force XPlane time of day (Seconds past midnight)
static void EnforceTime(const float secPastMidnight);

// Handle any graphics updates
static int GraphicsCallBack(XPLMDrawingPhase     inPhase,
    int                  inIsBefore,
    void *               inRefcon);

static int CameraCallback(XPLMCameraPosition_t * outCameraPosition,    /* Can be NULL */
    int inIsLosingControl,
    void * inRefcon);

static void TakeCameraControl(bool take);

/*------------------------------------------------------------------------*/
// IMPLEMENTATION
/*------------------------------------------------------------------------*/

PLUGIN_API int XPluginStart(char *outName,
    char *outSig,
    char *outDesc)
{
    if (!s_pluginStarted)
    {
        // Used by widget to make sure only one widgets instance created
        s_pluginStarted = true;

        // Can't use strcpy_s - overwites the string memory with 0xFD debug bytes before string copying
        // I would need to know char array lengths in order to use it safely
        strcpy(outName, PLUGIN_NAME);
        strcpy(outSig, PLUGIN_ID);
        strcpy(outDesc, PLUGIN_DESCRIPTION);

        // Release the camera
        TakeCameraControl(false);
    }

    // Always return 1 to indicate we have handled this DLL call.
    // Really random things happen if you return 0 and carry on. (Other plugins get called instead etc.)
    return 1;
}

/*------------------------------------------------------------------------*/

PLUGIN_API void	XPluginStop(void)
{
    if (s_pluginStarted)
    {
        s_pluginStarted = false;

        // Release the camera
        TakeCameraControl(false);
    }
}

/*------------------------------------------------------------------------*/

PLUGIN_API int XPluginEnable(void)
{
    // Delete previous shared memory and create a new shared memory here
    DestroySharedState();
    CreateSharedState();

    // Remember original size
    XPLMGetScreenSize(&s_original_width, &s_original_height);

    // Start initial timer.
    s_timer.tic();

    // The function "GraphicsCallBack" will be called each frame
    XPLMRegisterDrawCallback(
        GraphicsCallBack,
        xplm_Phase_LastScene,
        0,
        NULL);

    // Datarefs
    s_hFoV = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
    s_vFoV = XPLMFindDataRef("sim/graphics/view/vertical_field_of_view_deg");

    s_zulu = XPLMFindDataRef("sim/time/zulu_time_sec");

    s_aircraftOverride = XPLMFindDataRef("sim/operation/override/override_planepath");
    s_aircraftX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    s_aircraftY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    s_aircraftZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    s_aircraftPsi = XPLMFindDataRef("sim/flightmodel/position/psi");
    s_aircraftTheta = XPLMFindDataRef("sim/flightmodel/position/theta");
    s_aircraftPhi = XPLMFindDataRef("sim/flightmodel/position/phi");

    // Main window handle
    s_hwnd = reinterpret_cast<HWND>(XPLMGetDatai(XPLMFindDataRef("sim/operation/windows/system_window")));

    // Patch to allow large screen sizes
    s_originalProc = SetWindowLongPtr(s_hwnd, GWLP_WNDPROC, (LONG_PTR)WndProc);

    // Let XPlane control the camera until we get a request from MATLAB to set a camera position
    TakeCameraControl(false);

    // Always return 1 to indicate that we have handled this function
    return 1;
}

/*------------------------------------------------------------------------*/

PLUGIN_API void XPluginDisable(void)
{
    // Patch to allow large screen sizes
    SetWindowLongPtr(s_hwnd, GWLP_WNDPROC, s_originalProc);

    // Set back to original resolution
    if (NULL != s_hwnd)
    {
        SetWindowPos(s_hwnd, NULL, 0, 0, s_original_width + 16, s_original_height + 38, SWP_NOMOVE);
    }

    // Stops XPlane from calling the "callback"
    XPLMUnregisterDrawCallback(
        GraphicsCallBack,
        xplm_Phase_LastScene,
        0,
        NULL);

    // Matching toc
    s_timer.toc();

    // Destroy shared memory object
    DestroySharedState();

    // Release the camera
    TakeCameraControl(false);
}

/*------------------------------------------------------------------------*/

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, long inMsg, void * inParam)
{
    // Do nothing - no known plugin interactions as of yet
}

/*------------------------------------------------------------------------*/

void CreateSharedState()
{
    try
    {
        // Open or create the named mutex
        s_mutex_control = new named_mutex(create_only, SHARED_MUTEX_NAME_CONTROL);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception named_mutex = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception named_mutex = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    try
    {
        // Create shared memory segment
        shared_memory_object shm_obj(
            create_only,                 // only create
            SHARED_MEMORY_NAME_CONTROL,          // name
            read_write);                 // read-write mode

        shm_obj.truncate(sizeof(s_controlData));

        // Map the whole shared memory in this process
        s_region_control = new mapped_region(shm_obj, read_write);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception shared_memory_object = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception shared_memory_object = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    try
    {
        // Open or create the named mutex
        s_mutex_screen = new named_mutex(create_only, SHARED_MUTEX_NAME_SCREEN);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception named_mutex = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception named_mutex = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    try
    {
        // Create shared memory segment
        shared_memory_object shm_obj(
            create_only,                 // only create
            SHARED_MEMORY_NAME_SCREEN,          // name
            read_write);                 // read-write mode

        shm_obj.truncate(sizeof(s_screenData));

        // Map the whole shared memory in this process
        s_region_screen = new mapped_region(shm_obj, read_write);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception shared_memory_object = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception shared_memory_object = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }
}

void DestroySharedState()
{
    // Delete the mutex
    try
    {
        named_mutex::remove(SHARED_MUTEX_NAME_CONTROL);
        delete s_mutex_control;
        s_mutex_control = 0;
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception named_mutex::remove = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception named_mutex::remove = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    // Remove shared memory segment
    try
    {
        delete s_region_control;
        s_region_control = NULL;
        shared_memory_object::remove(SHARED_MEMORY_NAME_CONTROL);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception shared_memory_object::remove = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception shared_memory_object::remove = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    // Delete the mutex
    try
    {
        named_mutex::remove(SHARED_MUTEX_NAME_SCREEN);
        delete s_mutex_screen;
        s_mutex_screen = 0;
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception named_mutex::remove = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception named_mutex::remove = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }

    // Remove shared memory segment
    try
    {
        delete s_region_screen;
        s_region_screen = NULL;
        shared_memory_object::remove(SHARED_MEMORY_NAME_SCREEN);
    }
    catch (interprocess_exception& err)
    {
        sprintf(error_buffer, "Exception shared_memory_object::remove = %s\n", err.what());
        XPLMDebugString(error_buffer);
    }
    catch (...)
    {
        sprintf(error_buffer, "Exception shared_memory_object::remove = %s\n", "UNKNOWN");
        XPLMDebugString(error_buffer);
    }
}

// GFX LOOP
int GraphicsCallBack(XPLMDrawingPhase     inPhase,
    int                  inIsBefore,
    void *               inRefcon)
{
    ///////////////////////////////////
    // Performing screen capture
    ///////////////////////////////////
    if (NULL != s_region_screen)
    {
        //------------------------------------------------
        // Populate metadata
        //------------------------------------------------
        ScreenMetaData* pMetaData(&s_screenData.metadata);

        // Get camera position at this moment in time
        XPLMCameraPosition_t cameraPosition;
        XPLMReadCameraPosition(&cameraPosition);
        // Store aircraft position 
        XPLMLocalToWorld(
            cameraPosition.x, cameraPosition.y, cameraPosition.z,
            &pMetaData->latitude, &pMetaData->longitude, &pMetaData->altitude);
        // Store NED origin (for future conversions in MATLAB)
        XPLMLocalToWorld(
            0.0, 0.0, 0.0,
            &pMetaData->latitude0, &pMetaData->longitude0, &pMetaData->altitude0);
        // Store aircraft orientation
        pMetaData->psi = cameraPosition.heading; // DEG
        pMetaData->theta = cameraPosition.pitch; // DEG
        pMetaData->phi = cameraPosition.roll;    // DEG
        // Store camera Field of View
        pMetaData->hFoV = XPLMGetDataf(s_hFoV); // DEG
        pMetaData->vFoV = XPLMGetDataf(s_vFoV); // DEG
        pMetaData->zoom = cameraPosition.zoom;  // Unitless

        // Optional (minor speed boost if disabled)
        if (s_controlData.getImageBuffer)
        {
            // Image pixels
            XPLMGetScreenSize(&(pMetaData->width), &(pMetaData->height));

            // Get pixels write directly to memory mapped section
            ScreenImageData* pImageData(&s_screenData.imagedata);
            glReadPixels(0, 0, pMetaData->width, pMetaData->height, GL_RGB, GL_UNSIGNED_BYTE, pImageData->rgb);

            pMetaData->hasRGB = true;
        }
        else
        {
            pMetaData->hasRGB = false;
        }

        // Optional (large speed boost if disabled - glReadPixels for the depth buffer is expensive)
        if (s_controlData.getDepthBuffer)
        {
            // Get pixels
            ScreenImageData* pImageData(&s_screenData.imagedata);
            glReadPixels(0, 0, pMetaData->width, pMetaData->height, GL_DEPTH_COMPONENT, GL_FLOAT, pImageData->depth);

            // Get actual distances
            GLfloat projArray[16];
            glGetFloatv(GL_PROJECTION_MATRIX, projArray);

            Matrix4x4 projMatrix = {};
            memcpy(&projMatrix, &projArray[0], sizeof(projArray));
            Plane planes[6] = {};
            ExtractPlanesGL(planes, projMatrix, true);

            // ax+by+cz = d 
            // z = d/c   when x = 0 and y = 0;
            double zNear(planes[4].d / planes[4].c);
            double zFar(planes[5].d / planes[5].c);

            // ax+by+cz = d    Using LEFT plane[0]
            // x = (d-c*z)/a   when y = 0 and z = zNear;
            double wNear((planes[0].d - planes[0].c * zNear) / planes[0].a);
            double wFar((planes[0].d - planes[0].c * zFar) / planes[0].a);

            // ax+by+cz = d    Using TOP plane[3]
            // y = (d-c*z)/b   when x = 0 and z = zNear;
            double hNear((planes[3].d - planes[3].c * zNear) / planes[3].b);
            double hFar((planes[3].d - planes[3].c * zFar) / planes[3].b);

            double HFOV(atan(wNear / zNear) / M_PI * 180);
            double VFOV(atan(hNear / zNear) / M_PI * 180);

            pMetaData->wNear = wNear;
            pMetaData->wFar = wFar;
            pMetaData->hNear = hNear;
            pMetaData->hFar = hFar;
            pMetaData->zNear = zNear;
            pMetaData->zFar = zFar;

            pMetaData->hasDepth = true;
        }
        else
        {
            pMetaData->hasDepth = false;
        }

        // If it gets here at least once, then we have new data
        s_capture_number++;
        pMetaData->capture_number = s_capture_number;

        // UPDATE THE PAYLOAD
        s_mutex_screen->lock();
        {
            // Modify flag and echo command
            memcpy(s_region_screen->get_address(), &s_screenData, sizeof(s_screenData));

            // Release the mutex
            s_mutex_screen->unlock();
        }
    }
    ///////////////////////////////////

    ///////////////////////////////////
    // Performing screen capture
    ///////////////////////////////////

    // GET THE CONTROLS (Image will update next frame)
    s_mutex_control->lock();
    {
        // Read latest controls        
        memcpy(&s_controlData, s_region_control->get_address(), sizeof(s_controlData));
        // Release the mutex
        s_mutex_control->unlock();
    }

    // Check requested image size
    if (s_controlData.width == 0)
    {
        s_controlData.width = SCREENSHOT_DEFAULT_WIDTH;
    }
    if (s_controlData.height == 0)
    {
        s_controlData.height = SCREENSHOT_DEFAULT_HEIGHT;
    }
    if (s_controlData.hFoV == 0)
    {
        s_controlData.hFoV = SCREENSHOT_DEFAULT_HFOV;
    }
    if (s_controlData.vFoV == 0)
    {
        s_controlData.vFoV = SCREENSHOT_DEFAULT_VFOV;
    }

    if (s_controlData.width < SCREENSHOT_MIN_WIDTH)
    {
        s_controlData.width = SCREENSHOT_MIN_WIDTH;
    }
    if (s_controlData.height < SCREENSHOT_MIN_HEIGHT)
    {
        s_controlData.height = SCREENSHOT_MIN_HEIGHT;
    }
    if (s_controlData.width > SCREENSHOT_MAX_WIDTH)
    {
        s_controlData.width = SCREENSHOT_MAX_WIDTH;
    }
    if (s_controlData.height > SCREENSHOT_MAX_HEIGHT)
    {
        s_controlData.height = SCREENSHOT_MAX_HEIGHT;
    }

    // Enforcing particular screen size
    EnforceWindowSize(s_controlData.width, s_controlData.height);
    XPLMSetDataf(s_hFoV, s_controlData.hFoV); // DEG
    XPLMSetDataf(s_vFoV, s_controlData.vFoV); // DEG

    // Set time of day
    EnforceTime(s_controlData.timeOfDaySec);

    // Set camera operation
    TakeCameraControl(s_controlData.takeControl);

    if (s_controlData.bringAircraft)
    {
        // Set player aircraft
        int ao(1);
        XPLMSetDatavi(s_aircraftOverride, &ao, 0, 1);

        double R(500.0);
        double theta(s_controlData.psi + 90.0);
        double dX(R*cos(theta / 180.0 * M_PI));
        double dZ(R*sin(theta / 180.0 * M_PI));

        double X(0.0);
        double Y(0.0);
        double Z(0.0);
        XPLMWorldToLocal(
            s_controlData.latitude, s_controlData.longitude, s_controlData.altitude,
            &X, &Y, &Z);
        XPLMSetDatad(s_aircraftX, X + dX);
        XPLMSetDatad(s_aircraftY, Y);
        XPLMSetDatad(s_aircraftZ, Z + dZ);

        XPLMSetDataf(s_aircraftPsi, s_controlData.psi);
        XPLMSetDataf(s_aircraftTheta, 0.0);
        XPLMSetDataf(s_aircraftPhi, 0.0);
    }
    else
    {
        // Set player aircraft
        int ao(0);
        XPLMSetDatavi(s_aircraftOverride, &ao, 0, 1);
    }
    ///////////////////////////////////

    return 1;  // Let XPlane draw the frame
}

void EnforceWindowSize(int setwidth, int setheight)
{
    int width(0);
    int height(0);
    XPLMGetScreenSize(&width, &height);

    // If different from capture size
    if (((setwidth != width) || (setheight != height)) && true)
    {
#if defined(IBM)
        if (NULL != s_hwnd)
        {
            SetWindowPos(s_hwnd, NULL, 0, 0, setwidth + 16, setheight + 38, SWP_NOMOVE);
        }
#elif defined(APL)
#error "TODO: Force screen size to capture size"
#elif defined(LIN)
#error "TODO: Force screen size to capture size"
#endif
    }
}

void EnforceTime(const float secPastMidnight)
{
    float currentTimeSec(XPLMGetDataf(s_zulu));
    float targetTimeSec(secPastMidnight);

    float MAX_TIME_DRIFT_SEC(60.0f);
    float min_time_delta_sec(targetTimeSec + MAX_TIME_DRIFT_SEC);
    float max_time_delta_sec(targetTimeSec - MAX_TIME_DRIFT_SEC);
    if ((currentTimeSec < min_time_delta_sec) || (currentTimeSec > max_time_delta_sec))
    {
        XPLMSetDataf(s_zulu, targetTimeSec);  // Enforce time
    }
}

/*------------------------------------------------------------------------*/

void NormalizePlane(Plane& plane)
{
    float mag(sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c));
    plane.a = plane.a / mag;
    plane.b = plane.b / mag;
    plane.c = plane.c / mag;
    plane.d = plane.d / mag;
}

void ExtractPlanesGL(Plane* p_planes, const Matrix4x4& comboMatrix, bool normalize)
{
    // Left clipping plane
    p_planes[0].a = comboMatrix._41 + comboMatrix._11;
    p_planes[0].b = comboMatrix._42 + comboMatrix._12;
    p_planes[0].c = comboMatrix._43 + comboMatrix._13;
    p_planes[0].d = comboMatrix._44 + comboMatrix._14;
    // Right clipping plane
    p_planes[1].a = comboMatrix._41 - comboMatrix._11;
    p_planes[1].b = comboMatrix._42 - comboMatrix._12;
    p_planes[1].c = comboMatrix._43 - comboMatrix._13;
    p_planes[1].d = comboMatrix._44 - comboMatrix._14;
    // Top clipping plane
    p_planes[2].a = comboMatrix._41 - comboMatrix._21;
    p_planes[2].b = comboMatrix._42 - comboMatrix._22;
    p_planes[2].c = comboMatrix._43 - comboMatrix._23;
    p_planes[2].d = comboMatrix._44 - comboMatrix._24;
    // Bottom clipping plane
    p_planes[3].a = comboMatrix._41 + comboMatrix._21;
    p_planes[3].b = comboMatrix._42 + comboMatrix._22;
    p_planes[3].c = comboMatrix._43 + comboMatrix._23;
    p_planes[3].d = comboMatrix._44 + comboMatrix._24;
    // Near clipping plane
    p_planes[4].a = comboMatrix._41 + comboMatrix._31;
    p_planes[4].b = comboMatrix._42 + comboMatrix._32;
    p_planes[4].c = comboMatrix._43 + comboMatrix._33;
    p_planes[4].d = comboMatrix._44 + comboMatrix._34;
    // Far clipping plane
    p_planes[5].a = comboMatrix._41 - comboMatrix._31;
    p_planes[5].b = comboMatrix._42 - comboMatrix._32;
    p_planes[5].c = comboMatrix._43 - comboMatrix._33;
    p_planes[5].d = comboMatrix._44 - comboMatrix._34;
    // Normalize the plane equations, if requested
    if (normalize)
    {
        NormalizePlane(p_planes[0]);
        NormalizePlane(p_planes[1]);
        NormalizePlane(p_planes[2]);
        NormalizePlane(p_planes[3]);
        NormalizePlane(p_planes[4]);
        NormalizePlane(p_planes[5]);
    }
}

// Function that can intecept and modify windows messages in XPlane
LRESULT CALLBACK WndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_GETMINMAXINFO:
    {
        PMINMAXINFO pMinMaxInfo = (PMINMAXINFO)(lParam);
        // Override it
        pMinMaxInfo->ptMaxSize.x = SCREENSHOT_MAX_WIDTH + 16;
        pMinMaxInfo->ptMaxSize.y = SCREENSHOT_MAX_HEIGHT + 38;
        pMinMaxInfo->ptMinTrackSize.x = SCREENSHOT_MIN_WIDTH + 16; // 1040;   // XPlane min vals, at least on my Win7 theme
        pMinMaxInfo->ptMinTrackSize.y = SCREENSHOT_MIN_HEIGHT + 38;// 806;    // XPlane min vals, at least on my Win7 theme
        pMinMaxInfo->ptMaxTrackSize.x = pMinMaxInfo->ptMaxSize.x;
        pMinMaxInfo->ptMaxTrackSize.y = pMinMaxInfo->ptMaxSize.y;
    }
    break;

    default:
        return CallWindowProc(
            (WNDPROC)s_originalProc,
            hwnd,
            uMsg,
            wParam,
            lParam);
        break;
    }

    return 0;
}

void TakeCameraControl(bool take)
{
    if (take)
    {
        if (!s_hasCamera)  // We dont have camera control already
        {
            s_hasCamera = 1;
            XPLMControlCamera(xplm_ControlCameraForever, CameraCallback, 0);
        }
    }
    else
    {
        if (s_hasCamera)  // We have camera control and need to release it
        {
            XPLMDontControlCamera();
            s_hasCamera = 0;
        }
    }
}

// Because there is only one camera we are going to just handle it here for now.
int CameraCallback(XPLMCameraPosition_t * outCameraPosition,    /* Can be NULL */
    int inIsLosingControl,
    void * inRefcon)
{
    if (NULL != outCameraPosition)
    {
        double x(0.0);
        double y(0.0);
        double z(0.0);

        double lat(s_controlData.latitude);
        double lng(s_controlData.longitude);
        double alt(s_controlData.altitude);

        XPLMWorldToLocal(lat, lng, alt, &x, &y, &z);

        // Apply update
        outCameraPosition->x = static_cast<float>(x);
        outCameraPosition->y = static_cast<float>(y);
        outCameraPosition->z = static_cast<float>(z);

        outCameraPosition->heading = s_controlData.psi;
        outCameraPosition->pitch = s_controlData.theta;
        outCameraPosition->roll = s_controlData.phi;

        outCameraPosition->zoom = s_controlData.zoom;
    }

    return s_hasCamera; // Keep camera
}
