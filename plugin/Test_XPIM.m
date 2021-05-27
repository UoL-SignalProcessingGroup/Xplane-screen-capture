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

close all;
clear;
clc;
drawnow;

hfig = 1;

figure(hfig);
set(hfig, 'DoubleBuffer', 'on');
hax = gca(hfig);

%% CAPTURE
data.jobID = 123;
data.takeControl = false;
data.bringAircraft = false;
data.getImageBuffer = true;
data.getDepthBuffer = true;
data.width = 1280;
data.height = 800;
data.latitude = 42.35;   %Deg
data.longitude = -71.43;  %Deg
data.altitude = 2000;   %Deg
data.psi = 30;         %Deg
data.theta = -5;       %Deg
data.phi = 0;         %Deg

XPLANE_HFOV = 60; % Needs to match settings in XPlane preferences
Required_HFOV = 50;
ScreenAspectRatio = data.width / data.height;
[ zoomFactor, XPLANE_VFOV, Required_VFOV] = XPlaneZoomFOV( XPLANE_HFOV, Required_HFOV, ScreenAspectRatio ); 
data.hFoV = Required_HFOV;
data.vFoV = Required_VFOV;
data.zoom = zoomFactor;
data.timeOfDay = 12*60*60;  % midday

%SL = stoploop('Stop sensor simulation');

% Prealloc images for faster drawing
hsp1 = subplot(1,2,1);
h_rgb = imshow(zeros(data.height, data.width, 3));
str1 = sprintf('RGB');
t1 = title(str1);

hsp2 = subplot(1,2,2);
h_depth = imagesc(zeros(data.height,data.width), [0, log10(200000)]);
set(gca, 'XTick', []);
set(gca, 'YTick', []);
axis image;
colormap(jet(256));
str2 = sprintf('Depth');
t2 = title(str2);

subplot(hsp1); % Switch back to 1st plot (RGB image)

clkXPlaneFPS = tic; LoopFPS =0;
clkLoopFPS = tic;
prev_capture_number = 0;
FPSEvery = 1.0;
XPlaneFPS = 0;

XPIM_Init();
while (true) %(~SL.Stop())
        
    clkLoopFPS = tic;
    
    data.latitude = data.latitude + 0.001000;    
    data.phi = data.phi + 0.01000;    
    
    clkCapture = tic;
    XPIM_Control(data);
    [meta, rgb, depth] = XPIM_CaptureImage();
    tCapture = toc(clkCapture);
    
    % Theoretical max FPS estimate based on repeated MEX call overheads.
    CaptureRate =  1/tCapture; 
    
    % FPS estimate based on frame counter in XPlane
    tXPlaneFPS = toc(clkXPlaneFPS);
    if (tXPlaneFPS>FPSEvery)        
        XPlaneFPS = (meta.capture_number - prev_capture_number)/FPSEvery;
        prev_capture_number = meta.capture_number;
        clkXPlaneFPS = tic;
        drawnow;  % call occasionally here
    end

    minDepth = min(depth(:));
    maxDepth = max(depth(:));

    set(h_rgb,'CData',rgb); 
    set(h_depth,'CData',log10(depth)); 
    
    %drawnow;  % Slows down MATLAB
    pause(0);
    
    % FPS based on all factors (including MATLAB drawing - slow)
   	tLoop = toc(clkLoopFPS);
    LoopFPS =  1/tLoop; 
    
    str = sprintf('Frame: %d (LoopFPS: %0.2f, XPlaneFPS: %0.2f)', meta.capture_number, LoopFPS, XPlaneFPS);
    set(t1,'string',str);
    str = sprintf('Min Dist %0.2fm, Max Dist %0.2fm', minDepth, maxDepth);
    set(t2,'string',str);   
end

XPIM_Cleanup();
close(hfig);
% SL.Clear();