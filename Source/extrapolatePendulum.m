function result = extrapolatePendulum(x, v, pivot_v, l)
% Extrapolate the position of the end-point of a pendulum given its
% velocity, length, & the velocity of the pivot point.
%
% Inputs:
%   x - position of the end point of a pendulum. Can be a time-indexed array.
%   v - velocity of the end-point of a pendulum. Can be a time-indexed array.
%   pivot_v - velocity of the pivot point. 
%   l - length of the pendulum.

% Define the gravity constant.
g = 9.80665;

% Compute extrapolated position. 
offset_vel = v - pivot_v;
result = x + (offset_vel + pivot_v)/sqrt(g/l);

end
