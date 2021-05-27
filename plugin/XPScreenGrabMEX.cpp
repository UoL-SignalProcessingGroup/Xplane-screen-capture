// Copyright (c) 2012, http://code.google.com/p/example-xplane-plugins/
// Copyright (c) 2021, The University of Liverpool
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of "example-xplane-plugins" nor "The University of Liverpool" nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL "example-xplane-plugins" or "The University of Liverpool" BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*------------------------------------------------------------------------*/
// INCLUDES
/*------------------------------------------------------------------------*/

// Matlab or Octave
#include "mex.h"

// Boost - for shared memory comms
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

// Plugin - shared constants 
#include "XPScreenGrabConstants.h"

/*------------------------------------------------------------------------*/
// NAMESPACES
/*------------------------------------------------------------------------*/

using namespace boost::interprocess;

/*------------------------------------------------------------------------*/
// CONSTANTS
/*------------------------------------------------------------------------*/

enum eReadyness
{
    eERROR = -1,
    eUNINITIALISED = 0,
    eINITIALISED = +1
};

static const int XP_INIT(0);
static const int XP_GRAB(1);
static const int XP_CLEANUP(2);
static const int XP_CONTROL(3);

static const int NUM_INPUT_PARAMS(1);
static const int COMMAND_PARAM_INDEX(0);

static const int NUM_IMAGE_DIMENSIONS(3);
static const int NUM_DEPTH_DIMENSIONS(2);

/*------------------------------------------------------------------------*/
// STATIC VARIABLES
/*------------------------------------------------------------------------*/

static mapped_region* s_region_control = nullptr;
static named_mutex*   s_mutex_control = nullptr;
static ControlData s_controlData;

static mapped_region* s_region_screen = nullptr;
static named_mutex*   s_mutex_screen = nullptr;
static ScreenData s_screenData;

static eReadyness s_readyness(eUNINITIALISED);

static long s_previous_capture_number(-1);

/*------------------------------------------------------------------------*/
// FILE SCOPE FUNCTION DECLARATIONS
/*------------------------------------------------------------------------*/

static void XPInit();
static void XPCleanup();
static void XPControl(const int nrhs, const mxArray *prhs[]);
static void XPGrab(int nlhs, mxArray *plhs[]);
static void OpenGL2MatlabRGB(const unsigned char* pOpenGL_RGB, unsigned char* pMatlab_RGB, const ScreenMetaData* const pMetadata);
static void OpenGL2MatlabDepth(const float* pOpenGL_Depth, float* pMatlab_Depth, const ScreenMetaData* const pMetadata);
static mxArray*  AddMetaData(const ScreenMetaData* pMetaData);
static void MATLAB2ControlStruct(const mxArray* inData, ControlData& controls);

/*------------------------------------------------------------------------*/
// IMPLEMENTATION
/*------------------------------------------------------------------------*/

// Only for build testing when editing in VS (Must do proper build and linking in MATLAB with MEX)
#if _MSC_VER
int main()
{
}
#endif

void ExitFunc()
{
    XPCleanup();
}

// Entry point for Matlab MEX function (the MEX compiler replaces it with filename)
void mexFunction(
    int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])
{
    mexAtExit(ExitFunc);

    //if (success)
    //{
    //  if (NUM_INPUT_PARAMS!=nrhs)
    //  {
    //    mexPrintf("Incorrect number of inputs (Got %i Expected %i).\n", nrhs, NUM_INPUT_PARAMS);
    //    success = false;
    //  }
    //}

    if (NULL == prhs)
    {
        mexErrMsgTxt("Invalid MEX pointer 'prhs'.\n");
    }


    if (NULL == prhs[COMMAND_PARAM_INDEX])
    {
        mexErrMsgTxt("First input param is null pointer.\n");
    }

    int command(static_cast<int>(mxGetScalar(prhs[COMMAND_PARAM_INDEX])));

    switch (command)
    {
        case XP_INIT:
        {
            // Can call any time
            XPInit();            
        }
        break;
        case XP_CONTROL:
        {
            if (eINITIALISED == s_readyness)
            {
                XPControl(nrhs, prhs);
            }
            else
            {
                mexErrMsgTxt("Call init before grabbing.\n");
            }
        }
        break;
        case XP_GRAB:
        {
            if (eINITIALISED == s_readyness)
            {
                XPGrab(nlhs, plhs);
            }
            else
            {
                mexErrMsgTxt("Call init before grabbing.\n");
            }
        }
        break;
        case XP_CLEANUP:
        {
            // Can call any time
            XPCleanup();
        }
        break;
        default:
        {
            s_readyness = eERROR;
            mexErrMsgTxt("Unknown screengrab command.\n");
        }
    }
}

// CONNECTS TO SHARED MEMORY INTERFACE
static void XPInit()
{
    XPCleanup();

    try
    {
        // Try to get the named mutex
        s_mutex_screen = new named_mutex(open_only, SHARED_MUTEX_NAME_SCREEN);
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Failed to find named_mutex ('%s').\n", SHARED_MUTEX_NAME_SCREEN);
        mexPrintf("REASON: %s\n", err.what());
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }
    catch (...)
    {
        mexPrintf("Failed to find named_mutex ('%s').\n", SHARED_MUTEX_NAME_SCREEN);
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }

    try
    {
        // Try to get the shared memory segment
        shared_memory_object shm_obj(
            open_only,                 //only create
            SHARED_MEMORY_NAME_SCREEN,          //name
            read_write);                 //read-write mode

        shm_obj.truncate(sizeof(s_screenData));

        // Map the whole shared memory in this process
        s_region_screen = new mapped_region(shm_obj, read_write);
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Failed to find shared_memory_object ('%s').\n", SHARED_MEMORY_NAME_SCREEN);
        mexPrintf("REASON: %s\n", err.what());
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }
    catch (...)
    {
        mexPrintf("Failed to find shared_memory_object ('%s').\n", SHARED_MEMORY_NAME_SCREEN);
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }

    try
    {
        // Try to get the named mutex
        s_mutex_control = new named_mutex(open_only, SHARED_MUTEX_NAME_CONTROL);
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Failed to find named_mutex ('%s').\n", SHARED_MUTEX_NAME_CONTROL);
        mexPrintf("REASON: %s\n", err.what());
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }
    catch (...)
    {
        mexPrintf("Failed to find named_mutex ('%s').\n", SHARED_MUTEX_NAME_CONTROL);
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }

    try
    {
        // Try to get the shared memory segment
        shared_memory_object shm_obj(
            open_only,                 //only create
            SHARED_MEMORY_NAME_CONTROL,          //name
            read_write);                 //read-write mode

        shm_obj.truncate(sizeof(s_controlData));

        //Map the whole shared memory in this process
        s_region_control = new mapped_region(shm_obj, read_write);
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Failed to find shared_memory_object ('%s').\n", SHARED_MEMORY_NAME_CONTROL);
        mexPrintf("REASON: %s\n", err.what());
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }
    catch (...)
    {
        mexPrintf("Failed to find shared_memory_object ('%s').\n", SHARED_MEMORY_NAME_CONTROL);
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexPrintf("Ensure the XPlane plugin is running.\n");
        mexErrMsgTxt("XPInit failed.\n");
    }

    s_readyness = eINITIALISED;
    mexPrintf("Init succeeded.\n");
}

// DISCONNECTS FROM SHARED MEMORY INTERFACE
static void XPCleanup()
{
    delete s_mutex_screen;
    s_mutex_screen = nullptr;

    delete s_region_screen;
    s_region_screen = nullptr;

    delete s_mutex_control;
    s_mutex_control = nullptr;

    delete s_region_control;
    s_region_control = nullptr;

    s_readyness = eUNINITIALISED;
    mexPrintf("Clean up succeeded.\n");
}

// SET UP CAPTURE MODE AND CONTROL CAPTURE AT THESE SETTINGS
static void XPControl(const int nrhs, const mxArray *prhs[])
{
    if (NULL == s_region_control)
    {
        mexPrintf("Failed to get shared memory array.\n");
        mexErrMsgTxt("XPControl failed.\n");
    }
    void* pControlData(s_region_control->get_address());
    if (NULL == pControlData)
    {
        mexPrintf("Failed to access shared memory array.\n");
        mexErrMsgTxt("XPControl failed.\n");
    }
    size_t nControlData(s_region_control->get_size());
    if (0U == nControlData)
    {
        mexPrintf("Shared memory size is zero.\n");
        mexErrMsgTxt("XPControl failed.\n");
    }

    // Update s_screenData with commands
    MATLAB2ControlStruct(prhs[1], s_controlData);

    try
    {
        // TODO: give up if locked out by XPlane   boost::posix_time::ptime(???);
        // TODO: give up if locked out by XPlane   if (s_mutex_screen->timed_lock())
        {
            s_mutex_control->lock();  //
            //if (s_mutex_control->try_lock())
            {
                memcpy(pControlData, &s_controlData, sizeof(s_controlData));
                s_mutex_control->unlock();
            }
        }
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Locking/Unlocking error.\n");
        mexPrintf("REASON: %s\n", err.what());
        mexErrMsgTxt("XPControl failed.\n");
    }
    catch (...)
    {
        mexPrintf("Locking/Unlocking error.\n");
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexErrMsgTxt("XPControl failed.\n");
    }
}

// READ THE SHARED MEMORY INTERFACE INTO A MATLAB IMAGE ARRAY
static void XPGrab(int nlhs, mxArray *plhs[])
{
    unsigned char*  pMatlab_RGB(NULL);
    float*  pMatlab_Depth(NULL);

    if (NULL == s_region_screen)
    {
        mexPrintf("Failed to get shared memory array.\n");
        mexErrMsgTxt("XPGrab failed.\n");
    }
    void* pScreenData(s_region_screen->get_address());
    if (NULL == pScreenData)
    {
        mexPrintf("Failed to access shared memory array.\n");
        mexErrMsgTxt("XPGrab failed.\n");
    }
    size_t nScreenData(s_region_screen->get_size());
    if (0U == nScreenData)
    {
        mexPrintf("Shared memory size is zero.\n");
        mexErrMsgTxt("XPGrab failed.\n");
    }

    if (NULL == plhs)
    {
        mexPrintf("Invalid MEX pointer 'plhs'.\n");
        mexErrMsgTxt("XPGrab failed.\n");
    }

    if ((nlhs < 1) || (nlhs > 3))
    {
        mexPrintf("Wrong number of output parameters. (Got %i Expected 1 or 2).\n", nlhs);
        mexErrMsgTxt("XPGrab failed.\n");
    }

    try
    {
        s_mutex_screen->lock();
        memcpy(&s_screenData, pScreenData, sizeof(s_screenData));
        s_mutex_screen->unlock();
    }
    catch (interprocess_exception& err)
    {
        mexPrintf("Locking/Unlocking error.\n");
        mexPrintf("REASON: %s\n", err.what());
        mexErrMsgTxt("XPGrab failed.\n");
    }
    catch (...)
    {
        mexPrintf("Locking/Unlocking error.\n");
        mexPrintf("REASON: %s\n", "UNKNOWN EXCEPTION!");
        mexErrMsgTxt("XPGrab failed.\n");
    }

    //-------------------------------------------------------
    // Write output
    //-------------------------------------------------------
    const ScreenMetaData* pMetaData(&s_screenData.metadata);
    const ScreenImageData* pImageData(&s_screenData.imagedata);

    // Convert OpenGL frame byte order to expected matlab image format
    if (nlhs >= 1)
    {
        // Metadata is always 1st parameter
        plhs[0] = AddMetaData(pMetaData);
    }

    // If the RGB image was requested...
    if (s_controlData.getImageBuffer)
    {
        // Allocate to right size
        // When specified in this order this map to an RGBRGBRGB pattern which is the glReadPixel output
        mwSize dimsRGB[NUM_IMAGE_DIMENSIONS] =
        {
            pMetaData->height,
            pMetaData->width,
            3
        };
        mxArray* rgbMatrix = mxCreateNumericArray(NUM_IMAGE_DIMENSIONS, dimsRGB, mxUINT8_CLASS, mxREAL);
        pMatlab_RGB = reinterpret_cast<unsigned char*>(mxGetData(rgbMatrix));

        // Populate
        const unsigned char* pOpenGL_RGB(pImageData->rgb);
        OpenGL2MatlabRGB(pOpenGL_RGB, pMatlab_RGB, pMetaData);

        if (nlhs >= 2)
        {
            plhs[1] = rgbMatrix;
        }
    }
    else
    {
        // Return an empty [] RGB buffer if not requested
        if (nlhs >= 2)
        {
            plhs[1] = mxCreateNumericArray(0, NULL, mxDOUBLE_CLASS, mxREAL);
        }
    }

    // If the depth image was requested...
    if (s_controlData.getDepthBuffer)
    {
        mwSize dimsDepth[NUM_DEPTH_DIMENSIONS] =
        {
            pMetaData->height,
            pMetaData->width,
        };
        mxArray* depthMatrix = mxCreateNumericArray(NUM_DEPTH_DIMENSIONS, dimsDepth, mxSINGLE_CLASS, mxREAL);
        pMatlab_Depth = reinterpret_cast<float*>(mxGetData(depthMatrix));

        // Populate
        const float* pOpenGL_Depth(pImageData->depth);
        OpenGL2MatlabDepth(pOpenGL_Depth, pMatlab_Depth, pMetaData);

        // Depth buffer
        if (nlhs >= 3)
        {
            plhs[2] = depthMatrix;
        }
    }
    else
    {
        // Return an empty [] depth buffer if not requested
        if (nlhs >= 3)
        {
            plhs[2] = mxCreateNumericArray(0, NULL, mxDOUBLE_CLASS, mxREAL);
        }
    }
}

void OpenGL2MatlabDepth(const float* pOpenGL_Depth, float* pMatlab_Depth, const ScreenMetaData* const pMetadata)
{
    // Plus the matlab memory is column major... :(
    // ...  the openGL memory is row major.
    const int width(pMetadata->width);
    const int height(pMetadata->height);

    // Cast down to float, hopefully faster than doubles!
    const float wNear(static_cast<float>(pMetadata->wNear));
    const float wFar(static_cast<float>(pMetadata->wFar));
    const float hNear(static_cast<float>(pMetadata->hNear));
    const float hFar(static_cast<float>(pMetadata->hFar));
    const float zNear(static_cast<float>(pMetadata->zNear));
    const float zFar(static_cast<float>(pMetadata->zFar));

    int sourceRow(0);
    int sourceCol(0);

    int sourceIndex(0);
    int destIndex(0);

    float zb(0.0f);
    float zd(0.0f);
    float zFar_corrected(0.0f);  // Corrected for frustum spreading
    float rFar(0.0f);

    for (sourceCol = 0U; sourceCol < width; sourceCol++)
    {
        for (sourceRow = height - 1; sourceRow >= 0; sourceRow--)
        {
            // Other RGB indices
            sourceIndex = (sourceRow * width) + sourceCol;

            //// TODO:  zFar_corrected = zFar(sourceRow, sourceCol)
            //rFar = sqrt(   );// TODO: SLOW
            //zFar_corrected = sqrt((zFar * zFar) + (rFar * rFar));  // TODO: SLOW
            zFar_corrected = zFar;

            // Convert to real distance
            zb = pOpenGL_Depth[sourceIndex];
            zd = -zFar_corrected * zNear / (zb * (zFar_corrected - zNear) - zFar_corrected);

            // Copy and split data into colour planes
            pMatlab_Depth[destIndex] = zd;

            // Move to next locations in each "2D array"
            ++destIndex;
        }
    }
}

void OpenGL2MatlabRGB(const unsigned char* pOpenGL_RGB, unsigned char* pMatlab_RGB, const ScreenMetaData* const pMetadata)
{
    const int width(pMetadata->width);
    const int height(pMetadata->height);
    const int planes(3);

    // OpenGL RGB RGB RGB RGB 
    // Matlab RRRR GGGG BBBB  3 x [SCREENSHOT_WIDTH*SCREENSHOT_HEIGHT]
    // Plus the matlab memory is column major... :(
    // ...  the openGL memory is row major.
    int sourceRow(0);
    int sourceCol(0);
    int pixelOffset(0);

    int sourceR_n(0);
    int sourceG_n(0);
    int sourceB_n(0);

    int destR_n(0U * width*height);
    int destG_n(1U * width*height);
    int destB_n(2U * width*height);

    for (sourceCol = 0U; sourceCol < width; sourceCol++)
    {
        for (sourceRow = height - 1; sourceRow >= 0; sourceRow--)
        {
            // Other RGB indices
            pixelOffset = (sourceRow * width) + sourceCol;
            sourceR_n = planes * pixelOffset;
            sourceG_n = sourceR_n + 1;
            sourceB_n = sourceR_n + 2;

            // Copy and split data into colour planes
            pMatlab_RGB[destR_n] = pOpenGL_RGB[sourceR_n];
            pMatlab_RGB[destG_n] = pOpenGL_RGB[sourceG_n];
            pMatlab_RGB[destB_n] = pOpenGL_RGB[sourceB_n];

            // Move to next locations in each "2D array"
            ++destR_n;
            ++destG_n;
            ++destB_n;
        }
    }
}

mxArray* AddMetaData(const ScreenMetaData* pMetaData)
{
    static const char *metadataNames[] = {
      "capture_number", "has_RGB", "has_depth", "width", "height", 
      "latitude", "longitude", "altitude",
      "psi", "theta", "phi",
      "hFoV", "vFoV", "zoom",
      "latitude_origin", "longitude_origin", "altitude_origin"
    };

    mxArray* pMetaArray(mxCreateStructMatrix(1, 1, 17, metadataNames));

    mxSetField(pMetaArray, 0, "capture_number", mxCreateDoubleScalar(pMetaData->capture_number));
    mxSetField(pMetaArray, 0, "has_RGB", mxCreateDoubleScalar(pMetaData->hasRGB));
    mxSetField(pMetaArray, 0, "has_depth", mxCreateDoubleScalar(pMetaData->hasDepth));

    mxSetField(pMetaArray, 0, "width", mxCreateDoubleScalar(pMetaData->width));
    mxSetField(pMetaArray, 0, "height", mxCreateDoubleScalar(pMetaData->height));

    mxSetField(pMetaArray, 0, "latitude", mxCreateDoubleScalar(pMetaData->latitude));
    mxSetField(pMetaArray, 0, "longitude", mxCreateDoubleScalar(pMetaData->longitude));
    mxSetField(pMetaArray, 0, "altitude", mxCreateDoubleScalar(pMetaData->altitude));

    mxSetField(pMetaArray, 0, "psi", mxCreateDoubleScalar(pMetaData->psi));
    mxSetField(pMetaArray, 0, "theta", mxCreateDoubleScalar(pMetaData->theta));
    mxSetField(pMetaArray, 0, "phi", mxCreateDoubleScalar(pMetaData->phi));

    mxSetField(pMetaArray, 0, "hFoV", mxCreateDoubleScalar(pMetaData->hFoV));
    mxSetField(pMetaArray, 0, "vFoV", mxCreateDoubleScalar(pMetaData->vFoV));
    mxSetField(pMetaArray, 0, "zoom", mxCreateDoubleScalar(pMetaData->zoom));

    mxSetField(pMetaArray, 0, "latitude_origin", mxCreateDoubleScalar(pMetaData->latitude0));
    mxSetField(pMetaArray, 0, "longitude_origin", mxCreateDoubleScalar(pMetaData->longitude0));
    mxSetField(pMetaArray, 0, "altitude_origin", mxCreateDoubleScalar(pMetaData->altitude0));

    return pMetaArray;
}

double ReadField_double(const mxArray* inData, const mwIndex index, const char* name)
{
    double value(0.0);

    // FUNCTION?
    mxArray* fieldPtr(mxGetField(inData, index, name));
    if (fieldPtr)
    {
        value = mxGetScalar(fieldPtr);
        // mexPrintf("INFO: %s=%f.\n", name, value);
    }
    else
    {
        mexPrintf("Missing field '%s'", name);
        mexErrMsgTxt("Attempt to access missing field.");
    }

    return value;
}

void MATLAB2ControlStruct(const mxArray* inData, ControlData& controls)
{
    const mwIndex index(0U);

    controls.jobId = static_cast<int>(ReadField_double(inData, index, "jobID"));

    controls.takeControl = static_cast<bool>(ReadField_double(inData, index, "takeControl") != 0.0);
    controls.bringAircraft = static_cast<int>(ReadField_double(inData, index, "bringAircraft"));      
    
    controls.getImageBuffer = static_cast<bool>(ReadField_double(inData, index, "getImageBuffer") != 0.0);
    controls.getDepthBuffer = static_cast<bool>(ReadField_double(inData, index, "getDepthBuffer") != 0.0);

    controls.hFoV = static_cast<float>(ReadField_double(inData, index, "hFoV"));
    controls.vFoV = static_cast<float>(ReadField_double(inData, index, "vFoV"));
    controls.width = static_cast<int>(ReadField_double(inData, index, "width"));
    controls.height = static_cast<int>(ReadField_double(inData, index, "height"));

    controls.latitude = ReadField_double(inData, index, "latitude");
    controls.longitude = ReadField_double(inData, index, "longitude");
    controls.altitude = ReadField_double(inData, index, "altitude");

    controls.psi = static_cast<float>(ReadField_double(inData, index, "psi"));
    controls.theta = static_cast<float>(ReadField_double(inData, index, "theta"));
    controls.phi = static_cast<float>(ReadField_double(inData, index, "phi"));

    controls.zoom = static_cast<float>(ReadField_double(inData, index, "zoom"));
    controls.timeOfDaySec = static_cast<float>(ReadField_double(inData, index, "timeOfDay"));
}

/*------------------------------------------------------------------------*/
