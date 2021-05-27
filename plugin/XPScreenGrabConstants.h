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


#ifndef XPSCREENGRAB_CONSTANTS_H
#define XPSCREENGRAB_CONSTANTS_H


// SHARE SAME FILE - OR ENSURE CONTENTS MATCH LOCAL VERSIONS IN BOTH XPLANE AND MATLAB PLUGINS

#define PLUGIN_NAME         "ScreenGrabber"
#define PLUGIN_ID           "exam.examples.screengrabber"
#define PLUGIN_DESCRIPTION  "Plug-in that exposes the opengl window pixels via a BOOST shared memory interface."

static const double PI(3.14159265358979);
static const double RAD_TO_DEG(180.0 / PI);
static const double DEG_TO_RAD(1.0 / RAD_TO_DEG);

static const int MAX_WIDGET_STRING_LENGTH(256);
static const int MAX_ERROR_STRING_LENGTH(1024);

static const int SCREENSHOT_NUM_COLOR_PLANES(3);

//static const int SCREENSHOT_MAX_WIDTH(2592);
//static const int SCREENSHOT_MAX_HEIGHT(1944);

static const int SCREENSHOT_MAX_WIDTH(2048);
static const int SCREENSHOT_MAX_HEIGHT(2048);

static const int SCREENSHOT_DEFAULT_WIDTH(1024);
static const int SCREENSHOT_DEFAULT_HEIGHT(768);

static const int SCREENSHOT_MIN_WIDTH(640);
static const int SCREENSHOT_MIN_HEIGHT(480);

static const float SCREENSHOT_DEFAULT_HFOV(60.0f);
static const float SCREENSHOT_DEFAULT_VFOV(50.0f);
static const float SCREENSHOT_DEFAULT_ZOOM(1.0f);

static const int SCREENSHOT_NUMPIXELS(SCREENSHOT_MAX_WIDTH * SCREENSHOT_MAX_HEIGHT);
static const int SCREENSHOT_NUMBYTES(SCREENSHOT_NUM_COLOR_PLANES * SCREENSHOT_NUMPIXELS);

static const char SHARED_MEMORY_NAME_SCREEN[] = {"xplane_screengrab_mem_screen"};
static const char SHARED_MUTEX_NAME_SCREEN[] = {"xplane_screengrab_mutex_screen"};

static const char SHARED_MEMORY_NAME_CONTROL[] = { "xplane_screengrab_mem_control" };
static const char SHARED_MUTEX_NAME_CONTROL[] = { "xplane_screengrab_mutex_control" };

struct ControlData
{
  int mode;   // Set by Matlab side to change operating mode of Xplane side
  int bringAircraft;
  bool takeControl;

  int stale;   // Unset by XPlane side to indicate a read is req, and Set by Matlab side to indicate ok to overwrite
  int jobId; // Image counter -> num frames grabbed since plugin was enabled
  long capture_number;

  int getImageBuffer;
  int getDepthBuffer;

  int width;
  int height;

  double latitude;
  double longitude;
  double altitude;

  float psi;
  float theta;
  float phi;

  float hFoV;
  float vFoV;
  float zoom;
  float timeOfDaySec;
};

struct ScreenMetaData
{
  long capture_number;

  int width;
  int height;

  double latitude;
  double longitude;
  double altitude;

  float psi;
  float theta;
  float phi;

  float hFoV;
  float vFoV;
  float zoom;

  bool hasRGB;
  bool hasDepth;

  double wNear;
  double wFar;
  double hNear;
  double hFar;
  double zNear;
  double zFar;

  double latitude0;
  double longitude0;
  double altitude0;
};

struct ScreenImageData
{
  unsigned char rgb[3*SCREENSHOT_NUMPIXELS];
  float depth[SCREENSHOT_NUMPIXELS];
};

struct ScreenData
{
  ScreenMetaData metadata;
  ScreenImageData imagedata;
};


#endif
