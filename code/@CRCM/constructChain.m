function constructChain(obj)
% constructChain
%
% Sets up link configurations and loads stl files for continuous rotation 
% compliant mechanism. This is based on the original crcdLoad_T2 by Matt Moses.

    N = obj.N;

    obj.linkbase = solpart('manip_base.stl');

    % g_s is a 4x4xNJ array where each (:,:,i) entry is the initial
    % configuration for link i
    obj.g_s = zeros(4,4,N);

    % w is a 3xNJ array where each (:,NJ) entry is the twist axis for link i
    obj.w = zeros(3,N);

    % q is a 3xNJ array where each (:,NJ) entry is a point on the 
    % axis for link i
    obj.q = zeros(3,N);
    
    for i = 1:N - 1
       obj.g_s(:,:,i) = [1 0 0 0; 0 1 0 0; 0 0 1 (i-1); 0 0 0 1];
       obj.q(:,i) = [0 0 i-1]';
       switch mod(i,2)
           case 0
               obj.link{i} = solpart('crcdTet2.stl');
               obj.w(:,i) = [1 0 0]';    
           case 1
               obj.link{i} = solpart('crcdTet1.stl');
               obj.w(:,i) = [0 1 0]';
       end
    end

    obj.link{N} = solpart('manip_tool_v4.stl');
    obj.g_s(:,:,N) = [1 0 0 0; 0 1 0 0; 0 0 1 N-1; 0 0 0 1];
    obj.w(:,N) = [0 1 0]';
    obj.q(:,N) = [0 0 N-1]';
end
