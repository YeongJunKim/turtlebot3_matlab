function v = Kinematic2(x_hat,x_ref)
persistent gamma1_2 gamma2_2 h_2
persistent firstRun_2

if isempty(firstRun_2)
    gamma1_2 = 0.08;                       %0.5;
    gamma2_2 = 0.4;                     %0.5;
    h_2 = 2;
	
    firstRun_2 = 1;
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
v(1) = gamma1_2 * e * cos(alpha);
if alpha ~= 0
    v(2) = -gamma2_2 * alpha - gamma1_2 * cos(alpha) * sin(alpha) / alpha * (alpha + h_2*phi);
else
    v(2) = 0;
end


    
end