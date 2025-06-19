clear all

% inputs
xr = [1.2, 0.7]; % metros (equilibrium point)
fd = [10, 0]; % Newtons (desired reference constant force)
xe0 = [1.3, 0.7]; % metros, xe initial

% parameters
Cf = [0.05, 0; 0, 0]; % adimensional (meaning of compliance)
Kp = [5000, 0; 0, 5000]; % Newtons/metro? (Damping matrix B)
Md = [1000, 0; 0, 1000]; % Newtons/metro? (Inertia matrix M)
Kd = [5000, 0; 0, 5000]; % Newtons/metro? (Stiffness matrix K)
K = [1000, 0; 0, 0]; % Newtons/metro? (stiffness of the environment)


% xe = end-effector operational space pose

% x

% fe=K(xe-xr) % Newtons (tbd)
% xf = Cf(fd-fe)
