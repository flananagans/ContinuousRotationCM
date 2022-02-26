function W = log_SE3(g)
%Function W = log_SE3(g)
%   Function to convert SE(3) homogenous transformation matrix to se(3) twist
%   matrix (hat of a twist)
%
%   This is the log function for matrices within Lie algebra

    R = g(1:3, 1:3); % Rotation matrix
    p = g(1:3, 4); % Translation vector
    
    w_hat = logm(R);
    w = CRCM.wedge(w_hat);
    
    if(all(w == 0))
        A_inv = eye(3);
    else
        n_w = norm(w);
        A_inv = eye(3) - 0.5*w_hat + ...
                    (2*sin(n_w) - n_w*(1+cos(n_w))) / ...
                    (2*(n_w^2)*sin(n_w))*(w_hat^2);
    end
    
    W = [w_hat, A_inv*p;
         zeros(1, 4)];
end