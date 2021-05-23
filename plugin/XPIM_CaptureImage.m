% Copyright (c) 2012, http://code.google.com/p/example-xplane-plugins/
% Copyright (c) 2021, University of Liverpool
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of "example-xplane-plugins" nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL "example-xplane-plugins" BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function [metadata, rgb, depth] = XPIM_CaptureImage(filter_duplicate_frames)

if (nargin<1)
    filter_duplicate_frames = true;  % Default to skipping duplicate frame access
end

persistent prev_capture_number; 
if (isempty(prev_capture_number))
   prev_capture_number = 0; 
end

CAPTURE_MODE = 1;

if (filter_duplicate_frames)
    MAX_TRIES = 10; % Retry for a different frame
else
    MAX_TRIES = 1;  % Go with first check (which could be the same frame accessed twice)
end

success = false;
for N=1:MAX_TRIES
    [metadata, rgb, depth] = XPScreenGrabMEX(CAPTURE_MODE);
    if (metadata.capture_number ~= prev_capture_number)
        success = true;
        break;
    else
       a=0; 
    end
end

prev_capture_number = metadata.capture_number;

if (~success)
  warning('Duplicate frame read'); 
end

end