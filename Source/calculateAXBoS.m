function result = calculateAXBoS(x, v, com_x, com_v, l)

% Define the gravity constant.
g = 9.80665;

% Compute MoS.
offset_vel = com_v - v;
result = x + (offset_vel + com_v)/sqrt(g/l);

end
