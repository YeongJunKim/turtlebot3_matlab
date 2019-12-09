%% Matlab���� ��� i(��Ʈ -1) �Է��ϴ� ��
% -> 1i �� �ᵵ �ǰ� 1j �� ��
% -> ����� �ٴ� ��� 2i �̷��� �ᵵ �ǰ� 2 * 1i �̷��� �ص� �˴ϴ�
% -> �׳� i�θ� ���� i ��� ������ ���ԵǴ� ���(Ư�� �ݺ�������) ���� ��ġ�鼭 �Ǽ��� �� �����ϱ� �ǵ��� ����ó�� ������
% -> ���� ����: 2+3i, 2+3j, 2+3*1i, 2+3*1j
% -> ������ �� �ִ� ����: 2+3*i

%% ��Ȳ ���� ������
% �ϴ� �츮�� ��쿡 ���� �κ��� 3���� ��Ȳ(n=3)�� �������� (���ϸ� ��� �÷��� ��)
% ���� i = 1,2,3 �̰�, i = 1 �� follower, i = 2,3 �� leader
% 1773�ʺ��� ������ Rigid formation control �� �����Ϸ���, �κ��� 3���� ��� ��� edge�� ���� ����Ǿ� �־�� �ϴ� ����
% �׷��ϱ� �� ����Ǿ� �ִٰ� �����ϰ� �غ�����
% �Ե��� �ؾ��� ��: ��� K �� design
% ��Ʈ: ���ۿ� eigenvalue assignment, pole placement ���� �ĺ����� (���� �ĺ�����)
% �߰��� �ϸ� ���� ��: �κ� ��� �ø��� �پ��� ������ �׷����� ���ؼ��� �غ���
clear
%% Initialize
dt = 0.02;           % Sampling time, ����: ��
robot_num = 4;      % �κ� ����

alpha = 0.02;          % �� (12)
k = [1 1 1]';       % �� (13)

pose1 = [5 -5];
pose2 = [-5 3];
pose3 = [8 -3];     % ó�� �κ��� ��ġ x,y ��ǥ
pose4 = [-8  8];
z = [pose1(1) + 1i*pose1(2) , pose2(1) + 1i*pose2(2) , pose3(1) + 1i*pose3(2), pose4(1) + 1i*pose4(2)]';    % Aggregate state
z

target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % ��ǥ x,y ��ǥ
target_pose4 = [4 1]';      
z_target = [target_pose1(1) + 1i*target_pose1(2) , target_pose2(1) + 1i*target_pose2(2) , target_pose3(1) + 1i*target_pose3(2), target_pose4(1) + 1i*target_pose4(2)]';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% below line %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = z;
target_pos(1).data = [target_pose1(1), target_pose1(2) * 1i];
target_pos(2).data = [target_pose2(1), target_pose2(2) * 1i];
target_pos(3).data = [target_pose3(1), target_pose3(2) * 1i];
target_pos(4).data = [target_pose4(1), target_pose4(2) * 1i];
target_pos(1).value = 0;
target_pos(2).value = 0;
target_pos(3).value = 0;
target_pos(4).value = 0;

now_pos = [];
now_pos(1).data = [pose1(1) , pose1(2) * 1i];
now_pos(2).data = [pose2(1) , pose2(2) * 1i];
now_pos(3).data = [pose3(1) , pose3(2) * 1i];
now_pos(4).data = [pose4(1) , pose4(2) * 1i];
now_pos(1).value = 0;
now_pos(2).value = 0;
now_pos(3).value = 0;
now_pos(4).value = 0;
now_value_vector= zeros(robot_num, 1);
now_weight = zeros(robot_num, robot_num);

appended_data = []
% appended_data.pose(1).data = [];
% appended_data.pose(2).data = [];
% appended_data.pose(3).data = [];
% appended_data.pose(1).data = now_pos(1).data;
% appended_data.pose(2).data = now_pos(2).data;
% appended_data.pose(3).data = now_pos(3).data;
% appended_data.pose(1).value = 0;
% appended_data.pose(2).value = 0;
% appended_data.pose(3).value = 0;
appended_data.weight = zeros(robot_num, robot_num);
appended_data.vector = zeros(robot_num, 1);

clc
clf
now_pos(1).value = now_pos(1).data(1) + now_pos(1).data(2);
now_pos(2).value = now_pos(2).data(1) + now_pos(2).data(2);
now_pos(3).value = now_pos(3).data(1) + now_pos(3).data(2);
now_pos(4).value = now_pos(4).data(1) + now_pos(4).data(2);

target_pos(1).value = target_pos(1).data(1) + target_pos(1).data(2);
target_pos(2).value = target_pos(2).data(1) + target_pos(2).data(2);
target_pos(3).value = target_pos(3).data(1) + target_pos(3).data(2);
target_pos(4).value = target_pos(4).data(1) + target_pos(4).data(2);

target_pos.data
target_pos.value

movement = 0.5;

%% Simulation

figure(1);
N = 250;
appended_data1 = zeros(2, N);
appended_data2 = zeros(2, N);
appended_data3 = zeros(2, N);
appended_data4 = zeros(2, N);
for count = 1:N
    count
    % weight matrix
    weight = zeros(robot_num, robot_num);
    eigenvector = zeros(robot_num, 1);
    laplacian = zeros(robot_num, robot_num);
    
    % update pos
    
%     now_pos(1).value = now_pos(1).value + movement;
    random = rand();
    
     pos_val = [now_pos(1).value, now_pos(2).value, now_pos(3).value, now_pos(4).value]'
    
    % Determine weight values (�� (5)�� �� �����ִ� ���� ����)
    % L�� rank�� 2�� ���־�
%     index_i = 1;
%     index_j = 2;
%     index_k = 3;
%     L = zeros(robot_num, robot_num);
%     for robot = 1:robot_num
%        
%        random_complex= 1;
%        
%        if(index_i == robot)
%            L(index_i,index_j) = random_complex*(target_pos(index_k).value - target_pos(index_i).value);
%            L(index_i,index_k) = random_complex*(target_pos(index_i).value - target_pos(index_j).value);
%        end
%        if(index_j == robot)
%            L(index_j,index_k) = -random_complex*(target_pos(index_i).value - target_pos(index_j).value);
%            L(index_j,index_i) = -random_complex*(target_pos(index_j).value - target_pos(index_k).value);
%        end
%        if(index_k == robot)
%            L(index_k,index_i) = random_complex*(target_pos(index_j).value - target_pos(index_k).value);
%            L(index_k,index_j) = random_complex*(target_pos(index_k).value - target_pos(index_i).value);
%        end
%         L(robot, robot) = -sum(L(robot, :));
%     end
    
    L = zeros(robot_num,robot_num);
    for i = 1:robot_num
        for j = 1:robot_num-1
            if j == i
                continue;
            end
            for k = j+1:robot_num
                if k == i
                    continue;
                end
                random_complex = 1;
%                 fprintf("%d, %d, %d", i,j,k)
                L(i,j) = L(i,j) + random_complex*(z_target(k) - z_target(i));
                L(i,k) = L(i,k) + random_complex*(z_target(i) - z_target(j));
%                 L
            end
        end
        L(i,i) = - sum(L(i,:));
    end
    
    
    % Design K  
    [Q, R] = qr(L,0);
%     laplacian - P'*L*U
    U_ = Q;
    V_ = R;
    zeros(robot_num-2)
    V_(1:robot_num-2,1:robot_num-2)
    a = place(zeros(robot_num-2),V_(1:robot_num-2,1:robot_num-2),[1 2]);
    b = a / U_(1:robot_num-2,1:robot_num-2);
    K = [b zeros(robot_num-2,2) ; zeros(2, robot_num-2) eye(2)];
    
    
    
    z_dot = zeros(robot_num,1);
    for i = 1:robot_num-2       % Followers
        for j = 1:robot_num
            if j == i
                continue;
            end
            z_dot(i) = z_dot(i) + K(i,i) * L(i,j) * (z(j) - z(i));
        end
    end
    
%     d_bar = norm(z_target(robot_num) - z_target(robot_num-1));
%     diff_leaders = z(robot_num) - z(robot_num-1);
%     z_dot(robot_num-1) = alpha * diff_leaders * (norm(diff_leaders)^2 - d_bar^2);
%     z_dot(robot_num) = alpha * (-diff_leaders) * (norm(diff_leaders)^2 - d_bar^2);
%     z_dot(robot_num) = -z_dot(robot_num-1);
	z = z + z_dot * dt;
    
    diff_leaders = z(robot_num) - z(robot_num-1);
    d_bar = norm(z_target(robot_num) - z_target(robot_num-1));
    z_dot(robot_num-1) = alpha * diff_leaders * (norm(z(robot_num) - z(robot_num-1))^2 - d_bar^2);
    z_dot(robot_num) = alpha * (-diff_leaders) * (norm(z(robot_num-1) - z(robot_num))^2 - d_bar^2);
    
    z_dot(robot_num-1) = z_dot(robot_num - 1) + (z_target(robot_num-1) - z(robot_num-1));
    z_dot(robot_num) = z_dot(robot_num) + (z_target(robot_num) - z(robot_num));
    
%     z_dot(robot_num) = -z_dot(robot_num-1);
	z = z + z_dot * dt;
    
    z_dot
    
    pos_val(1) = z(1);
    pos_val(2) = z(2);
    pos_val(3) = z(3);
    pos_val(4) = z(4);
    
    % draw now
    plot(real(z_target(1)), imag(z_target(1)), 'ro');
    hold on;
    plot(real(z_target(2)), imag(z_target(2)), 'bo');
    hold on;
    plot(real(z_target(3)), imag(z_target(3)), 'go');
    hold on;
    plot(real(z_target(4)), imag(z_target(4)), 'yo');
    hold on;
    
    plot(real(s(1)), imag(s(1)), 'rs');
    hold on;
    plot(real(s(2)), imag(s(2)), 'bs');
    hold on;
    plot(real(s(3)), imag(s(3)), 'gs');
    hold on;
    plot(real(s(4)), imag(s(4)), 'ys');
    hold on;

    
    p1 = plot(real(pos_val(1)),imag(pos_val(1)), '*', "Color", "red");
    hold on;
    p2 = plot(real(pos_val(2)),imag(pos_val(2)), '*', "Color", "blue");
    hold on;
    p3 = plot(real(pos_val(3)),imag(pos_val(3)), '*', "Color", "green");
    hold on;
    p4 = plot(real(pos_val(4)),imag(pos_val(4)), '*', "Color", "yellow");
    hold off;
    xlim([-10, 10]);
    ylim([-10, 10]);
    grid on;
    drawnow;
    pause(0.03);
    % save simulation data
    appended_data1(:, count) = [real(pos_val(1)), imag(pos_val(1))];
    appended_data2(:, count) = [real(pos_val(2)), imag(pos_val(2))];
    appended_data3(:, count) = [real(pos_val(3)), imag(pos_val(3))];
    appended_data4(:, count) = [real(pos_val(4)), imag(pos_val(4))];
end
























figure(2)
targetplt1 = plot(real(z_target(1)), imag(z_target(1)), 'ro');
hold on;
targetplt2 = plot(real(z_target(2)), imag(z_target(2)), 'bo');
hold on;
targetplt3 = plot(real(z_target(3)), imag(z_target(3)), 'go');
hold on;
targetplt3 = plot(real(z_target(4)), imag(z_target(4)), 'yo');
hold on;

startplt1 = plot(real(s(1)), imag(s(1)), 'rs');
hold on;
startplt2 = plot(real(s(2)), imag(s(2)), 'bs');
hold on;
startplt3 = plot(real(s(3)), imag(s(3)), 'gs');
hold on;
startplt3 = plot(real(s(4)), imag(s(4)), 'ys');
hold on;

for i = 1:N
plot(appended_data1(1, i), appended_data1(2, i), "*", "Color", "red");
hold on;
plot(appended_data2(1, i), appended_data2(2, i), "*", "Color", "blue");
hold on;
plot(appended_data3(1, i), appended_data3(2, i), "*", "Color", "green");
hold on;
plot(appended_data4(1, i), appended_data4(2, i), "*", "Color", "yellow");
hold on;
end
hold on;
hold on;
% plot(appended_data.pose(3, :).data);
grid on;
xlim([-10, 10]);
ylim([-10, 10]);






 