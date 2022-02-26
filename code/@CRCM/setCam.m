function setCam(obj)
% sets camera parameters for the crcd code snippets

% camlight is not supported in GNU Octave. If you are running Matlab, you 
% can uncommnet this line:
    
    if(~obj.lightSet)
        camlight left
        obj.lightSet = true;
    end

    if(~obj.camSet)
        axis equal
        view(3)
        xlabel('x')
        ylabel('y')
        zlabel('z')

        z = obj.rotor_frame(3, end);

        axis(obj.C*[-3 3 -3 3 0 1.5*z])
        grid on
    end
end