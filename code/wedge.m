function w = wedge(W)
% Function w = wedge(W)
% wedge() maps an element of so(3) to if W is 3x3, or
% se(3) to R6 if W 4x4.

% [0 -w3 w2; w3 0 -w1; -w2 w1 0] = hat([w1; w2; w3])

s = length(W);

if (s==3)
    w = [W(3, 2); W(1, 3); W(2, 1)];
end

if (s==4)
    w = [W(1:3, 4); W(3, 2); W(1, 3); W(2, 1)];
end