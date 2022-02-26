% script that plays back sim data

clear
close all
clc


%% Load joint trajectories
%load('optimizedbtvec.mat')
load(sprintf('generatedTraj_strainenergy%d.mat', c.N));

%% Create chain object
try
    c = CRCM(N, C, alpha, k); % chain parameters may be loaded with trajectory
catch
    c = CRCM();
end
c.toPlot = true; % whether or not to plot the chain


%% Draw chain for each phi of btvec
[btrows,btcols] = size(btvec);
for jj = 1:btcols
    
    theta = btvec(:,jj) ;
    c.updateTheta(theta);
    if (jj == 1)
        c.camSet = true;
    end
    % getframe is not supported in GNU Octave. 
	% uncomment line below if you are running Matlab
    movv(jj) = getframe(gcf);
end

%% Plot joint angles
set1 = btvec(1:2:c.N,:)';
set2 = btvec(2:2:c.N,:)';
phivec = linspace(0,(btcols-1)/btcols*2*pi,btcols);

figure
subplot(1,2,1)
%plot(phivec,360/2/pi*set1)
plot(phivec,set1)
hold on
legend(compose('%d', 1:2:c.N));
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
legend(compose('%d', 2:2:c.N));
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
