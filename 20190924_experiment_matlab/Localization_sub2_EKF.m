function [state_hat] = Localization_sub2_EKF(f,Jacobian_F,h,Jacobian_H,state_previous,z,control_commands)
%% Parameters
persistent P Q R firstRun
if isempty(firstRun)
	P = eye(6);
    Q = 0.1 * eye(6);
    R = 0.1 * eye(6);
    
    firstRun = 1;
end

arguments = num2cell([state_previous' control_commands']);

%% Substituting variables into Jacobian matrices
F = Jacobian_F(arguments{:});
H = Jacobian_H(arguments{:});

%% EKF
% state_hat_temp = double(subs(f,[x,y,theta,u1,u2],[state_previous' control_commands']))';    % Prediction
state_hat_temp = f(arguments{:});    % Prediction
P = F * P * F' + Q;
K = P * H' / (H*P*H' + R);
Inno = z - h(arguments{:});  % Innovation
state_hat = state_hat_temp + K * Inno;   % Correction
P = (eye(6) - K*H) * P * (eye(6) - K*H)' + K*R*K';
end