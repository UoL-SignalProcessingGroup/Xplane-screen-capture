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


#include "XPIntervalTimer.h"


XPIntervalTimer::XPIntervalTimer():
m_startTime(),
m_frequency(),
m_isTimerSupported(false)
{
#ifdef IBM
  m_isTimerSupported = QueryPerformanceFrequency(&m_frequency);
#else
#error "Unsupported OS"
#endif
}

/*virtual*/ XPIntervalTimer::~XPIntervalTimer()
{
  // Destruct
}

void XPIntervalTimer::tic()
{
  if (m_isTimerSupported)
  {
#ifdef IBM
    QueryPerformanceCounter(&m_startTime);
#else
#error "Unsupported OS"
#endif
  }
}

double XPIntervalTimer::toc()
{ 
  double elapsedTimeInSeconds(0.0);

  if (m_isTimerSupported)
  {
#ifdef IBM
    LARGE_INTEGER endTime;
    QueryPerformanceCounter(&endTime); 

    // Ensure toc() is called after tic()!
    if (endTime.QuadPart >= m_startTime.QuadPart)
    {
      elapsedTimeInSeconds = static_cast<double>((endTime.QuadPart - m_startTime.QuadPart)) / static_cast<double>(m_frequency.QuadPart);
    }
#else
#error "Unsupported OS"
#endif
  }

  return elapsedTimeInSeconds;
}
