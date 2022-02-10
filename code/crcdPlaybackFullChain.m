% script that plays back sim data

clear
close all
clc

% This script loads the files and sets up the kinematics
% This gives us:
%   g_s - the initial frames for each link (all joint angles theta = 0)
%   w - the axis of rotation for each link
%   q - a point on the axis of rotation for each link
crcdLoadT2;

% theta = zeros(NJ,1);
% crcdUpdateKin_v2;
% crcdDraw_v2;
% crcdSetCam;

% comment this line out if you are running Matlab instead of GNU Octave:
%more off
 
%% Load joint trajectories
%load optimizedbtvec.mat
%load('generatedTraj.mat');
load('generatedTraj_nomovetochain.mat');
%load('generatedTraj_strainenergy.mat');

[btrows,btcols] = size(btvec);

%% Draw chain for each phi of btvec
for jj = 1:btcols
    theta = [btvec(:,jj) ; -btvec(end:-1:1,jj)];
    [J, link] = crcdUpdateKin_v2(g_s, w, q, theta, link);    
    crcdDraw_v2;
	drawnow;
    if (jj == 1)
        crcdSetCam;
    end
    %link{13}.config
    % getframe is not supported in GNU Octave. 
	% uncomment line below if you are running Matlab
    movv(jj) = getframe(gcf);
    crcdDelete_v2;
end

%% Plot joint angles
set1 = btvec([1 3 5 7 9 11 13],:)';
set2 = btvec([2 4 6 8 10 12],:)';
phivec = linspace(0,(btcols-1)/btcols*2*pi,btcols);

figure
subplot(1,2,1)
%plot(phivec,360/2/pi*set1)
plot(phivec,set1)
hold on
legend('1', '3', '5', '7', '9', '11', '13')
xlabel('\phi (radians)')
ylabel('\theta_i (radians)');
title('odd numbered joint angles');
%axis([0 9 -1.5 1.5])
xlim([0, 2*pi])
ax = gca();
ax.FontSize = 12;
ax.TickDir = 'in';
ax.LineWidth = 1;
            
subplot(1,2,2)
%plot(phivec, 360/2/pi*set2)
plot(phivec, set2)
legend('2', '4', '6', '8', '10', '12')
xlabel('\phi (radians)')
ylabel('\theta_i (radians)');
title('even numbered joint angles');
%axis([0 9 -1.5 1.5])
xlim([0, 2*pi])
ax = gca();
ax.FontSize = 12;
ax.TickDir = 'in';
ax.LineWidth = 1;

% movie2avi not supported in GNU Octave, uncomment below if running matlab:
% movie2avi([movv movv movv movv movv movv],'optimizedFullChainMovie.avi','Compression','none','FPS',10)
% 
