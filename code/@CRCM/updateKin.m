function updateKin(obj)
% updateKin
%
% updates forward kinematics and Jacobian for 
% continuous rotation compliant device
%
% This uses the product of exponentials form as described in 
% Murray Li Sastry (MLS)
% J is a 6xN matrix, the "Spatial Manipulator Jacobian", see MLS page 116-121 
% the ith column of J is the twist axis of the ith joint in the current
% configuration
%
% Based on crcdUpdateKin_v2 by Matt Moses
    
    % initialize the 4x4 transformation matrix
    tform = eye(4);

    % this loops through each joint, constructing J and finding the
    % configuration of each link
    for i = 1:length(obj.link)
        % first we get the column of the Jacobian corresponding to joint i
        obj.J(:,i) = CRCM.adjoint(tform)* ...
                    [CRCM.hat(-obj.w(:,i)) * obj.q(:,i) ; obj.w(:,i)];

        % next we find the transformation of link i
        tform = tform * ...
                CRCM.twist2g(CRCM.hat(-obj.w(:,i)) * obj.q(:,i), ...
                             obj.w(:,i), obj.theta(i));

        % set current configuration to transformation * initial config
        obj.link{i}.config = tform * obj.g_s(:,:,i);
    end
    
    obj.rotor_frame = obj.link{obj.N}.config;
end