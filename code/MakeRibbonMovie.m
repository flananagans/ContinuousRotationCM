% script that makes a movie of the CRCM ribbon
clear
close all
clc

%% Create chain object with default parameters
c = CRCM();
c.updateTheta(zeros(c.N, 1));
c.toPlot = true; % whether or not to plot the chain

% Width of the ribbon to draw
wr = 0.7;

nthet = 48;
phi = linspace(0, (nthet-1)/nthet*2*pi, nthet);

for jj = 1:nthet
    if(jj == 1)
        c.setCam;
        zlim([0, 4*c.C]) 
    end
    c.drawRibbon(phi(jj), wr);
    
	% getframe is not supported in GNU Octave. If you are running Matlab, you can
	% uncomment this line:
	% movv(jj) = getframe(gcf);
    drawnow
	cla
end

% movie2avi is not supported in GNU Octave. If you are running Matlab, you can
% uncomment this line:
% movie2avi([movv movv movv],'newRibbonMovie.avi','Compression','none','FPS',8)