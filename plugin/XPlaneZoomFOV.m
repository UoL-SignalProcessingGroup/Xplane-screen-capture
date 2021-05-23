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

function [ zoomFactor, XPLANE_VFOV, ZOOMED_VFOV] = XPlaneZoomFOV( XPLANE_HFOV, ZOOMED_HFOV, ScreenAspectRatio )

% X-Plane FOV when zoom is at x1
HFOV0 = XPLANE_HFOV;

% Horizontal distance off boresight for every 1 unit along boresight
HDIST0 = tand(HFOV0/2);

% Vertical distance off boresight for every 1 unit along boresight
VDIST0 = HDIST0/ScreenAspectRatio;  

% Vertical FOV when zoom is at x1
VFOV0 = atand(VDIST0)*2;

% REQUIRED Horizontal FOV when zoomed in
HFOVZ = ZOOMED_HFOV;

% Vertical FOV when zoomed in
VFOVZ = HFOVZ/ScreenAspectRatio;

% Convert back to off boresight distances (frustrum distances)
HDISTZ = tand(HFOVZ/2);
VDISTZ = tand(VFOVZ/2);

% The actual zoom! Ratio of the offboresight frustrum H distances
zoomFactor = HDIST0/HDISTZ;   % 19.842 for ARGUS with 60deg XPLANE_HFOV

% Also return the vertical FOVs
XPLANE_VFOV = VFOV0;
ZOOMED_VFOV = atand(VDISTZ)*2;

end

