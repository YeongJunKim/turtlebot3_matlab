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

%% Initialize
dt = 0.1;           % Sampling time, ����: ��
robot_num = 3;      % �κ� ����

alpha = 1;          % �� (12)
k = [1 1 1]';       % �� (13)

pose1 = [1 -5]';
pose2 = [-3 0]';
pose3 = [4 1]';     % ó�� �κ��� ��ġ x,y ��ǥ
z = [pose1(1) + 1i*pose1(2) , pose2(1) + 1i*pose2(2) , pose3(1) + 1i*pose3(2)]';    % Aggregate state
target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % ��ǥ x,y ��ǥ
z_target = [target_pose1(1) + 1i*target_pose1(2) , target_pose2(1) + 1i*target_pose2(2) , target_pose3(1) + 1i*target_pose3(2)]';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% below line %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
target_pos(1).data = [target_pose1(1), target_pose1(2) * 1i];
target_pos(2).data = [target_pose2(1), target_pose2(2) * 1i];
target_pos(3).data = [target_pose3(1), target_pose3(2) * 1i];
target_pos(1).value = 0;
target_pos(2).value = 0;
target_pos(3).value = 0;

now_pos = [];
now_pos(1).data = [pose1(1) , pose1(2) * 1i];
now_pos(2).data = [pose2(1) , pose2(2) * 1i];
now_pos(3).data = [pose3(1) , pose3(2) * 1i];
now_pos(1).value = 0;
now_pos(2).value = 0;
now_pos(3).value = 0;
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

target_pos(1).value = target_pos(1).data(1) + target_pos(1).data(2);
target_pos(2).value = target_pos(2).data(1) + target_pos(2).data(2);
target_pos(3).value = target_pos(3).data(1) + target_pos(3).data(2);

target_pos.data
target_pos.value



movement = 0.5;

%% Simulation
figure(1);
N = 10;
appended_data1 = zeros(2, N);
appended_data2 = zeros(2, N);
appended_data3 = zeros(2, N);
for count = 1:N
    % weight matrix
    weight = zeros(robot_num, robot_num);
    eigenvector = zeros(robot_num, 1);
    laplacian = zeros(robot_num, robot_num);
    v = zeros(1, 3);
    K = diag(v);
    
    % update pos
    
%     now_pos(1).value = now_pos(1).value + movement;
    random = rand();
    now_pos(2).value = now_pos(2).value + movement * (1-random) ;
    now_pos(3).value = now_pos(3).value + movement * (1-random) ;
    
    
    
    % now pos data
    
    
    
    pos_val = [now_pos(1).value, now_pos(2).value, now_pos(3).value]'
    
    % Determine weight values (�� (5)�� �� �����ִ� ���� ����)
    for i = 1:robot_num
        j = i - 1;
        k = i + 1;
        if(j == 0)
           j = robot_num;
        end
        if(k == robot_num + 1)
           k = 1;
        end
        
        random_weight = 1; % = (round(mod(rand()*5+1, 10)) + round(mod(rand()*5+1, 10))*1i);
%         weight(i, j) = random_weight*(now_pos(k).value - now_pos(i).value);
        weight(i, j) = random_weight*(target_pos(k).value - target_pos(i).value);
        random_weight = 1; % = (round(mod(rand()*5+1, 10)) + round(mod(rand()*5+1, 10))*1i);
%         weight(i, k) = random_weight*(now_pos(i).value - now_pos(j).value);
        weight(i, k) = random_weight*(target_pos(i).value - target_pos(j).value);
        
%         weight(j, i) = weight(i, j);
%         weight(k, i) = weight(i, k);
    end
    
    laplacian = -weight;
    % Determine L (Laplacian matrix)
    % L�� rank�� 2�� ���־�
    disp("weight : ");
    disp(weight);
    for i = 1:robot_num
        j = i - 1;
        k = i + 1;
        if(j == 0)
            j = robot_num;
        end
        if(k == robot_num + 1)
            k = 1;
        end
        laplacian(i, i) = -(laplacian(i, j) + laplacian(i, k));
    end
    disp("Laplacian : ");
    disp(laplacian);
    
    % Design K
    C = laplacian;
    [L, U, P] = lu(C);    
    [Q, R] = qr(C,0);
%     laplacian - P'*L*U
    
    k1 = 1;
    z1 = k1 * weight(1,2)*(pos_val(2) - pos_val(3));
    
    disp("z1:")
    disp(z1)
    
%     z_dot = ?
%     z = z + z_dot * dt;
%     
%     figure(1);
%     plot(real(z(1)),imag(z(1)),'ro'); hold on;
%     plot(real(z(2)),imag(z(2)),'bo');
%     plot(real(z(3)),imag(z(3)),'go'); hold off;
    p1 = plot(real(pos_val(1)),imag(pos_val(1)), '*', "Color", "red");
    hold on;
    p2 = plot(real(pos_val(2)),imag(pos_val(2)), '*', "Color", "blue");
    hold on;
    p3 = plot(real(pos_val(3)),imag(pos_val(3)), '*', "Color", "black");
    hold off;
    xlim([-10, 10]);
    ylim([-10, 10]);
    drawnow;
    grid on;
    pause(0.03);
%     drawnow;
    
    
    % save simulation data
    appended_data1(:, count) = [real(pos_val(1)), imag(pos_val(1))];
    appended_data2(:, count) = [real(pos_val(2)), imag(pos_val(2))];
    appended_data3(:, count) = [real(pos_val(3)), imag(pos_val(3))];
%     appended_data.pose(1,count).data = [real(pos_val(1)), imag(pos_val(1))];
%     appended_data.pose(2,count).data = [real(pos_val(2)), imag(pos_val(2))];
%     appended_data.pose(3,count).data = [real(pos_val(3)), imag(pos_val(3))];
end

figure(2)
for i = 1:N
plot(appended_data1(1, i), appended_data1(2, i), "*", "Color", "red");
hold on;
plot(appended_data2(1, i), appended_data2(2, i), "*", "Color", "blue");
hold on;
plot(appended_data3(1, i), appended_data3(2, i), "*", "Color", "black");
hold on;
end
hold on;
hold on;
% plot(appended_data.pose(3, :).data);
grid on;
xlim([-10, 10]);
ylim([-10, 10]);


% 
% LL = [-1-1i, 2, -1+1i;
%     1+1i, -2, 1-1i;
%     1+1i, -2, 1-1i];
% rank(LL)
% rank(laplacian)
% rank(L)
% rank(U)
% rank(P)
% L
% U
% P






 