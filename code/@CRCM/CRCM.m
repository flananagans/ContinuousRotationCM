classdef CRCM < matlab.mixin.Copyable
%CRCM Continuous Rotating Compliant Mechanism
%   This class contructs the mechanism chain and allows the user to update
%   kinematics for set joint angles or generate trajectories for desired
%   crank angles 'phi'. 
    
    properties
        % Chain parameters
        N {mustBeNumeric} = 13; % Number of links in HALF CHAIN
        C {mustBeNumeric} = 0.7*33/(4*pi); % 1/2 rotor height
        k {mustBeNumeric} = 15; % Joint stiffnesses (scalar or Nx1 vector)
        alpha {mustBeNumeric} = pi/2; % Skew angle (radians)
        
        % Chain configuration variables
        g_s % initial chain configuration
        w % twist axis for each link
        q % points on axis of each link
        link = {}; % Stored link configuration
        theta % Joint angles for HALF CHAIN
        J %Jacobian
        rotor_frame;
        strain_energy = 0;
        
        % Trajectory generation variables
        dt {mustBeNumeric} = 0.05; % timestep for integration
        
        % Plotting/display stuff
        toPlot {mustBeNumericOrLogical} = false;
        hanbase = [];
        han = {};
        linkbase
        lightSet = false;
        camSet = false;
    end
    
    methods
        % Constructor
        function obj = CRCM(N, C, alpha, k)
            if(nargin == 0) % default chain
            elseif(nargin == 4) % user-defined parameters
                obj.N = N;
                obj.C = C;
                obj.alpha = alpha;
                obj.k = k;
            else
                error('Wrong number of input arguments')
            end
            obj.constructChain();
        end
        
        % Calculate the strain energy of the chain
        strain_energy = calculateStrainEnergy(obj, theta);
        
        % updateTheta updates kinematics and strain energy
        function updateTheta(obj, theta)
            if(numel(theta) ~= obj.N)
                error('Given theta is wrong size')
            end
            obj.theta = theta;
            obj.updateKin();
            obj.strain_energy = obj.calculateStrainEnergy(obj.theta);
            
            if(obj.toPlot)
                obj.draw();
                obj.setCam();
                drawnow();
            end
        end
        
        [traj, strain_energy] = generateTrajectory(obj, phis);
    end
    
    methods (Static)
         A = adjoint(g);
         W = hat(w);
         w = wedge(W);
         W = log_SE3(g);
         g = twist2g(v,w,theta)
    end
    
    methods (Access = protected)
        % Copy constructor
        function cp = copyElement(obj)
            % Shallow copy object
            cp = copyElement@matlab.mixin.Copyable(obj);
        end 
    end
    
    methods (Access = private)
        % Construct the chain using this objects' parameters
        constructChain(obj)
        % Update the kinematics of the chain
        updateKin(obj);
        % Calculate partial derivative of strain energy wrt joint angles
        dE = calculateDeDtheta(obj);
        % Calculate cost of being off the desired backbone curve
        dP = calculateDpDtheta(obj, phi);
        [r, Rotation, EA, EB, T, N, B] = backboneFunc(obj, phi,s,wr)
        
        % Plot methods
        draw(obj);
        plotDelete(obj);
    end
end

