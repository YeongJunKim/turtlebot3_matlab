function [state_hat] = Localization_sub1_PEFFME(f,Jacobian_F,h,Jacobian_H,state_previous,z,control_commands,alpha)
% Localization based on tag-anchor distances using Hybrid PFIR filter
%%% Input arguments %%%
% -> f,F : State equations, Jacobian (Symbolic, ���� ���� ���� �ʿ�)
% -> h,H : Measurement equations, Jacobian (Symbolic, ���� ���� ���� �ʿ�)
% -> z : Tag 1�κ����� �Ÿ����� �� heading angle, [d12 d13 d14 theta]'
% -> control_commands : [delta_d delta_theta]'
% -> alpha : Measurement missing ���� (0�� �� missing, 1�� �� ����)

%%% Output arguments %%%
% -> state_hat(1) = x_coor : ��ġ x ��ǥ��
% -> state_hat(2) = y_coor : ��ġ y ��ǥ��
% -> state_hat(3) = heading : heading angle

%% Parameters
persistent firstRun
persistent M F_array H_array z_array control_array y_tilde_array u_tilde_array
persistent dim_state dim_z
persistent state_tilde count

if isempty(firstRun)
	M = 6;                                  % Horizon size
        
    dim_state = size(state_previous,1);         % n
    dim_z = size(z,1);                          % m
    
    F_array = zeros(dim_state,dim_state,M);
    H_array = ones(dim_z,dim_state,M);
    z_array = zeros(dim_z,M);
    y_tilde_array = zeros(dim_z,M);
    control_array = zeros(length(control_commands),M);
    u_tilde_array = zeros(dim_state,M);
	
    count = 0;
    state_tilde = state_previous;
    
    firstRun = 1;
end

%% Substituting variables into Jacobian matrices
arguments = num2cell([state_previous' control_commands']);
F = Jacobian_F(arguments{:});
H = Jacobian_H(arguments{:});

f_hat = f(arguments{:});
% f_hat(6,1) = wrapToPi(f_hat(6,1));
arguments2 = num2cell([f_hat' control_commands']);
h_hat = h(arguments2{:});

% %% Predict measurement if alpha = 0
% if alpha == 0
%     z = h_hat;
% end

z = (1 - alpha) * h_hat + alpha * z;

%% Array matrices
F_array(:,:,1:M-1) = F_array(:,:,2:M);
F_array(:,:,M) = F;
H_array(:,:,1:M-1) = H_array(:,:,2:M);
H_array(:,:,M) = H;
z_array(:,1:M-1) = z_array(:,2:M);
z_array(:,M) = z;
control_array(:,1:M-1) = control_array(:,2:M);
control_array(:,M) = control_commands;
y_tilde_array(:,1:M-1) = y_tilde_array(:,2:M);
y_tilde_array(:,M) = z - (h_hat - H * f_hat);
u_tilde_array(:,1:M-1) = u_tilde_array(:,2:M);
u_tilde_array(:,M) = f_hat - F * state_previous;

count = count + 1;

%% Final result
if count > M
    state_hat = PEFFME(F_array,H_array,y_tilde_array,u_tilde_array,M);
else
    state_hat = state_tilde;
end

end