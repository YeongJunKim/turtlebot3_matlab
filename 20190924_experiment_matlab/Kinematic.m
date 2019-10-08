function v = Kinematic(x_hat,x_ref)
persistent gamma1 gamma2 h
persistent firstRun

if isempty(firstRun)
    gamma1 = 0.08;                       %0.5;
    gamma2 = 0.4;                     %0.5;
    h = 2;
	
    firstRun = 1;
end

v = [0 0]';

%% e, phi, alpha
delta = x_ref - x_hat;
delta(3) = wrapToPi(delta(3));      % Wrap angle in radians to [-pi,pi]
theta_r = x_ref(3);
theta_r = wrapToPi(theta_r);

e = norm(delta(1:2));
phi = theta_r - wrapToPi(angle(delta(1) + 1i*delta(2)));
phi = wrapToPi(phi);
alpha = phi - delta(3);
alpha = wrapToPi(alpha);

%% Velocity
v(1) = gamma1 * e * cos(alpha);
if alpha ~= 0
    v(2) = -gamma2 * alpha - gamma1 * cos(alpha) * sin(alpha) / alpha * (alpha + h*phi);
else
    v(2) = 0;
end


    
end