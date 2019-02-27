function result = extrapolateMovingPendulum(x, v, pivot_v, l)

% Define the gravity constant.
g = 9.80665;

% Compute MoS.
%offset_vel = pivot_v - v;  % This gives better results but need to think this through.
offset_vel = v - pivot_v;
result = x + (offset_vel + pivot_v)/sqrt(g/l);

end
