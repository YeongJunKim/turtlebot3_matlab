function filters = filter_init(tile_size, time_interval,robot_num)
%% filter init
a = tile_size;

x1 = 0; y1 = 0;
x2 = 0; y2 = a;
x3 = a; y3 = a;
x4 = a; y4 = 0;

dt = time_interval;   % Sampling time

syms x y theta
syms v_l v_theta
syms x_sub y_sub theta_sub
syms v_l_sub v_theta_sub
% State of main robot
f_main.x(x,y,theta,v_l,v_theta) = x + v_l * cos(theta) * dt;
f_main.y(x,y,theta,v_l,v_theta) = y + v_l * sin(theta) * dt;
f_main.theta(x,y,theta,v_l,v_theta) = theta + v_theta * dt;
f_main = [f_main.x f_main.y f_main.theta]';
Jacobian_F_main = jacobian(f_main,[x,y,theta]);
f_main = matlabFunction(f_main);
Jacobian_F_main = matlabFunction(Jacobian_F_main);

% State of sub robots
f_sub.x_main(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = x;
f_sub.y_main(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = y;
f_sub.x_sub(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = x_sub + v_l_sub * cos(theta_sub) * dt;
f_sub.y_sub(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = y_sub + v_l_sub * sin(theta_sub) * dt;
f_sub.theta(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = theta_sub + v_theta_sub * dt;
f_sub = [f_sub.x_main f_sub.y_main f_sub.x_sub f_sub.y_sub f_sub.theta]';
Jacobian_F_sub = jacobian(f_sub,[x,y,x_sub,y_sub,theta_sub]);
f_sub = matlabFunction(f_sub);
Jacobian_F_sub = matlabFunction(Jacobian_F_sub);

% Measurement of main robot
h_main.dist1(x,y,theta,v_l,v_theta) = sqrt((x-x1)^2 + (y-y1)^2);
h_main.dist2(x,y,theta,v_l,v_theta) = sqrt((x-x2)^2 + (y-y2)^2);
h_main.dist3(x,y,theta,v_l,v_theta) = sqrt((x-x3)^2 + (y-y3)^2);
h_main.dist4(x,y,theta,v_l,v_theta) = sqrt((x-x4)^2 + (y-y4)^2);
h_main.theta(x,y,theta,v_l,v_theta) = theta;
h_main = [h_main.dist1 h_main.dist2 h_main.dist3 h_main.dist4 h_main.theta]';
Jacobian_H_main = jacobian(h_main,[x,y,theta]);
h_main = matlabFunction(h_main);
Jacobian_H_main = matlabFunction(Jacobian_H_main);

% Measurement of sub robots
h_sub.x_main(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = x;
h_sub.y_main(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = y;
h_sub.d_x(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = x - x_sub;
h_sub.d_y(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = y - y_sub;
h_sub.theta(x,y,x_sub,y_sub,theta_sub,v_l_sub,v_theta_sub) = theta_sub;
h_sub = [h_sub.x_main h_sub.y_main h_sub.d_x h_sub.d_y h_sub.theta]';
Jacobian_H_sub = jacobian(h_sub,[x,y,x_sub,y_sub,theta_sub]);
h_sub = matlabFunction(h_sub);
Jacobian_H_sub = matlabFunction(Jacobian_H_sub);



filters = [];
for i = 1:robot_num
    filters(i).EKF = EKF;
    filters(i).FIR = FIR;
end
FIR_init(filters(1).FIR, 6, 3, 5, 3, f_main, Jacobian_F_main, h_main, Jacobian_H_main, 0);
FIR_init(filters(2).FIR, 6, 5, 5, 3, f_sub, Jacobian_F_sub, h_sub, Jacobian_H_sub, 0);
FIR_init(filters(3).FIR, 6, 5, 5, 3, f_sub, Jacobian_F_sub, h_sub, Jacobian_H_sub, 0);

EKF_init(filters(1).EKF, eye(3), (blkdiag(0.1, 0.1, 0.1)), (blkdiag(0.2,0.2,0.2,0.2,0.1)), f_main, Jacobian_F_main,h_main, Jacobian_H_main);
EKF_init(filters(2).EKF, eye(5), (0.1 * eye(5)), (0.1*eye(5)), f_sub, Jacobian_F_sub, h_sub, Jacobian_H_sub);
EKF_init(filters(3).EKF, eye(5), (0.1 * eye(5)), (0.1*eye(5)), f_sub, Jacobian_F_sub, h_sub, Jacobian_H_sub);

end