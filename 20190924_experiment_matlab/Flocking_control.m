function v_input = Flocking_control(x_hat1,x_hat2, x_ref, x_ref_dot )

%% parameter 

% original initial position

% straight pattern %
p1 = [0 -0.5]; % pattern position for turtlebot 1
p2 = [0 0.5]; % pattern position for turtlebot 2


v_linear_max = 0.22;
% v_linear_max = 0.22;
% v_angle_max = 1.82;
% v_linear_max = 1.0;
v_angle_max = 1.82;

% % triangle patter %
% p1 = [1 0]; % pattern position for turtlebot 1
% p2 = [-1 0]; % pattern position for turtlebot 2


a=0.5;
% k=0.7;
k=0.03;
mu = [1, 0]';

b1 = 0; % calibration angle1 value
b2 = 0; % calibration angle2 value 


%% control

    
x1 = x_hat1(1);
y1 = x_hat1(2);
theta1 = x_hat1(3);

x2 = x_hat2(1);
y2 = x_hat2(2);
theta2 = x_hat2(3);


xd = x_ref(1);
yd = x_ref(2);
xd_dot = x_ref_dot(1);
yd_dot = x_ref_dot(2);

x1_err = x1-xd-p1(1);
y1_err = y1-yd-p1(2);
x2_err = x2-xd-p2(1);
y2_err = y2-yd-p2(2);


x1_input = -mu(1)*x1_err-a*(x1_err-x2_err)+xd_dot;
y1_input = -mu(1)*y1_err-a*(y1_err-y2_err)+yd_dot;

x2_input = -mu(1)*x2_err-a*(x2_err-x1_err)+xd_dot;
y2_input = -mu(1)*y2_err-a*(y2_err-y1_err)+yd_dot;



x1_input_dot = -mu(1)*x1_input-a*(x1_input-x2_input);
y1_input_dot = -mu(1)*y1_input-a*(y1_input-y2_input);

x2_input_dot = -mu(1)*x2_input-a*(x2_input-x1_input);
y2_input_dot = -mu(1)*y2_input-a*(y2_input-y1_input);



v_linear1=sqrt((x1_input)^2+(y1_input)^2);
v_linear2=sqrt((x2_input)^2+(y2_input)^2);

% 
% v_linear1=v_linear1/10;
% v_linear2=v_linear2/10;

% if v_linear1 > v_linear_max
%     v_linear1= v_linear_max;
% elseif v_linear1 < -1*v_linear_max
%     v_linear1= -1*v_linear_max;
%     
% end
% 
% if v_linear2 > v_linear_max
%     v_linear2=v_linear_max;
%     
% elseif v_linear2 < -1*v_linear_max
%     v_linear2= -1*v_linear_max;
% end



theta1_bar = atan2(y1_input,x1_input)+2*b1*pi;
theta2_bar = atan2(y2_input,x2_input)+2*b2*pi;

theta1_bar= wrapToPi(theta1_bar);
theta2_bar= wrapToPi(theta2_bar);


theta1_err = theta1-theta1_bar;
theta2_err = theta2-theta2_bar;


theta1_err= wrapToPi(theta1_err);
theta2_err= wrapToPi(theta2_err);


theta1_bar_dot = (-x1_input_dot*sin(theta1_bar)+y1_input_dot*cos(theta1_bar))/v_linear1;
theta2_bar_dot = (-x2_input_dot*sin(theta2_bar)+y2_input_dot*cos(theta2_bar))/v_linear2;


v_angle1 = -[x1_err y1_err]* [cos(theta1_bar) -sin(theta1_bar); sin(theta1_bar) cos(theta1_bar)]*[(cos(theta1_err)-1)/theta1_err; sin(theta1_err)/theta1_err]*v_linear1-k*theta1_err+theta1_bar_dot;
v_angle2 = -[x2_err y2_err]* [cos(theta2_bar) -sin(theta2_bar); sin(theta2_bar) cos(theta2_bar)]*[(cos(theta2_err)-1)/theta2_err; sin(theta2_err)/theta2_err]*v_linear2-k*theta2_err+theta2_bar_dot;

% v_angle1 = v_angle1/10;
% v_angle2 = v_angle2/10;

% if v_angle1 > v_angle_max
%     v_angle1 = v_angle_max;
% elseif v_angle1 < -1*v_angle_max
%     v_angle1 = -1*v_angle_max;
% end
% 
% if v_angle2 > v_angle_max
%     v_angle2= v_angle_max;
% elseif v_angle2 < -1*v_angle_max
%     v_angle2 = -1*v_angle_max;
% end


v_input = [v_linear1 v_angle1 v_linear2 v_angle2]';


end

