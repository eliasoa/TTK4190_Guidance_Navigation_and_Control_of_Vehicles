function [xk1,yk1,xk,yk,last] = WP_selector(x,y)
load('WP.mat');
%12.52 in Fossen
persistent k
    if isempty(k)
        k = 1;
    end
L               = 161;          % length (m)
R               = 2*L;          %
xk              = WP(1,k); 
yk              = WP(2,k);
xk1             = WP(1,k+1);
yk1             = WP(2,k+1);

% If close enough to next waypoint, select next waypoint
if (xk1 - x)^2 + (yk1 -y)^2 <= R^2
    if k >= length(WP)-1
        k = k;
    else
        k = k + 1;
    end
end
last = 0;

end