% crcdLoadT2.m
%
% Loads stl files for continuous rotation compliant device.
% This is based on crcdLoad_v2. It loads the full chain.
%
mainpath = './';

% linkbase is the fixed base part
hanbase = [];
han = {};
linkbase = solpart([mainpath 'manip_base.stl']);

% NJ is the number of joints in the half chain (ground to rotor)
NJ = 15;
two_chains = false; % Do you want two chains (lower and upper)

% Scaling factor of backbone curve to fit links better
C = .7*33/4/pi;
C = (NJ/13)*C;

% g_s is a 4x4xNJ array where each (:,:,i) entry is the initial
% configuration for link i
g_s = zeros(4,4,NJ);

% w is a 3xNJ array where each (:,NJ) entry is the twist axis for link i
w = zeros(3,NJ);

% q is a 3xNJ array where each (:,NJ) entry is a point on the 
% axis for link i
q = zeros(3,NJ);

for i = 1:NJ - 1
   g_s(:,:,i) = [1 0 0 0; 0 1 0 0; 0 0 1 (i-1); 0 0 0 1];
   q(:,i) = [0 0 i-1]';
   switch mod(i,2)
       case 0
           link{i} = solpart([mainpath 'crcdTet2.stl']);
           w(:,i) = [1 0 0]';    
       case 1
           link{i} = solpart([mainpath 'crcdTet1.stl']);
           w(:,i) = [0 1 0]';
   end
end

link{NJ} = solpart([mainpath 'manip_tool_v4.stl']);
g_s(:,:,NJ) = [1 0 0 0; 0 1 0 0; 0 0 1 NJ-1; 0 0 0 1];
w(:,NJ) = [0 1 0]';
q(:,NJ) = [0 0 NJ-1]';

if(two_chains)
    for i = NJ + 1:2*NJ - 1
        g_s(:,:,i) = [1 0 0 0; 0 1 0 0; 0 0 1 (i-1); 0 0 0 1];
       q(:,i) = [0 0 i-1]';
       switch mod(i,2)
           case 1
               link{i} = solpart([mainpath 'crcdTet2.stl']);
               w(:,i) = [1 0 0]';    
           case 0
               link{i} = solpart([mainpath 'crcdTet1.stl']);
               w(:,i) = [0 1 0]';
       end
    end

    link{2*NJ} = solpart([mainpath 'manip_tool.stl']);
    g_s(:,:,2*NJ) = [1 0 0 0; 0 1 0 0; 0 0 1 2*NJ - 1; 0 0 0 1];
    w(:,2*NJ) = [0 1 0]';
    q(:,2*NJ) = [0 0 2*NJ-1]';
    
    NJ = NJ*2;
end

