function result = extrapolatePendulum(x, v, l)

% Define the gravity constant.
g = 9.80665;

% Compute MoS.
result = x + v/sqrt(g/l);

end
