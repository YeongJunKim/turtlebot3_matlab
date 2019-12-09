%% Matlab에서 허수 i(루트 -1) 입력하는 법
% -> 1i 로 써도 되고 1j 도 됨
% -> 계수가 붙는 경우 2i 이렇게 써도 되고 2 * 1i 이렇게 해도 됩니다
% -> 그냥 i로만 쓰면 i 라는 변수를 쓰게되는 경우(특히 반복문에서) 변수 겹치면서 실수할 수 있으니깐 되도록 위에처럼 쓰세요
% -> 옳은 예시: 2+3i, 2+3j, 2+3*1i, 2+3*1j
% -> ㅈ망할 수 있는 예시: 2+3*i

%% 상황 설명 간략히
% 일단 우리의 경우에 맞춰 로봇이 3대인 상황(n=3)을 가정했음 (원하면 대수 늘려도 됨)
% 따라서 i = 1,2,3 이고, i = 1 이 follower, i = 2,3 이 leader
% 1773쪽부터 나오는 Rigid formation control 이 가능하려면, 로봇이 3대일 경우 모든 edge가 서로 연결되어 있어야 하는 듯함
% 그러니까 다 연결되어 있다고 가정하고 해보세요
% 님들이 해야할 것: 행렬 K 를 design
% 힌트: 구글에 eigenvalue assignment, pole placement 등을 쳐보세요 (뭐든 쳐보세요)
% 추가로 하면 좋은 것: 로봇 대수 늘리고 다양한 형태의 그래프에 대해서도 해보기

%% Initialize
dt = 0.05;           % Sampling time, 단위: 초
robot_num = 3;      % 로봇 개수

alpha = 0.05;          % 식 (12)
k = [1 1 1]';       % 식 (13)

pose1 = [1 -5]';
pose2 = [-3 0]';
pose3 = [4 1]';     % 처음 로봇들 위치 x,y 좌표
z = [pose1(1) + 1i*pose1(2) , pose2(1) + 1i*pose2(2) , pose3(1) + 1i*pose3(2)]';    % Aggregate state
target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % 목표 x,y 좌표
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
N = 100;
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
    
    % Determine weight values (식 (5)나 그 위에있는 내용 참고)
%     for i = 1:robot_num
%         j = i - 1;
%         k = i + 1;
%         if j == 0
%            j = robot_num;
%         end
%         if k == (robot_num + 1)
%            k = 1;
%         end
%         i
%         j
%         k
%         random_weight = 1; % = (round(mod(rand()*5+1, 10)) + round(mod(rand()*5+1, 10))*1i);
% %         weight(i, j) = random_weight*(now_pos(k).value - now_pos(i).value);
%         weight(i, j) = random_weight*(target_pos(k).value - target_pos(i).value);
%         random_weight = 1; % = (round(mod(rand()*5+1, 10)) + round(mod(rand()*5+1, 10))*1i);
% %         weight(i, k) = random_weight*(now_pos(i).value - now_pos(j).value);
%         weight(i, k) = random_weight*(target_pos(i).value - target_pos(j).value);
%         
% %         weight(j, i) = weight(i, j);
% %         weight(k, i) = weight(i, k);
%     end
    
    laplacian = -weight;
    % Determine L (Laplacian matrix)
    % L의 rank는 2로 가주앙
    
    L = zeros(robot_num, robot_num);
    
    index_i = 1;
    index_j = 2;
    index_k = 3;
    
    for robot = 1:robot_num
       
       random_complex= 1;
       
       if(index_i == robot)
           L(index_i,index_j) = random_complex*(target_pos(index_k).value - target_pos(index_i).value);
           L(index_i,index_k) = random_complex*(target_pos(index_i).value - target_pos(index_j).value);
       end
       if(index_j == robot)
           L(index_j,index_k) = -random_complex*(target_pos(index_i).value - target_pos(index_j).value);
           L(index_j,index_i) = -random_complex*(target_pos(index_j).value - target_pos(index_k).value);
       end
       if(index_k == robot)
           L(index_k,index_i) = random_complex*(target_pos(index_j).value - target_pos(index_k).value);
           L(index_k,index_j) = random_complex*(target_pos(index_k).value - target_pos(index_i).value);
       end
       
%  -4.0000 + 0.0000i   2.0000 + 4.0000i   2.0000 - 4.0000i
%   4.0000 + 0.0000i  -2.0000 - 4.0000i  -2.0000 + 4.0000i
%  -4.0000 + 0.0000i   2.0000 + 4.0000i   2.0000 - 4.0000i    z_target
%        L(i,j) = random_complex*(target_pos(k).value - target_pos(i).value);
%        L(i,k) = random_complex*(target_pos(i).value - target_pos(j).value);
        L(robot, robot) = -sum(L(robot, :));
        L
    end
    laplacian = L;
    % Design K
    C = laplacian;
    [L, U, P] = lu(C);    
    [Q, R] = qr(C,0);
%     laplacian - P'*L*U
    U_ = Q
    V_ = R
    zeros(robot_num-2)
    V_(1:robot_num-2,1:robot_num-2)
    K = place(zeros(robot_num-2),V_(1:robot_num-2,1:robot_num-2),[3]);
%     K = place(zeros(robot_num-2),laplacian(1:robot_num-2,1:robot_num-2),[3]);
    K = K / U_(1:robot_num-2,1:robot_num-2);
    K = [K zeros(robot_num-2,2) ; zeros(2, robot_num-2) eye(2)];
    
    
    
    z_dot = zeros(robot_num,1);
    for i = 1:robot_num-2       % Followers
        for j = 1:robot_num
            if j == i
                continue;
            end
            z_dot(i) = z_dot(i) + K(i,i) * L(i,j) * (pos_val(j) - pos_val(i));
%             z_dot(i) = z_dot(i) + K(i,i) * (z(j) - z(i));
        end
    end
    d_bar = norm(z_target(robot_num) - z_target(robot_num-1));
    diff_leaders = z(robot_num) - z(robot_num-1);
    z_dot(robot_num-1) = alpha * diff_leaders * (norm(diff_leaders)^2 - d_bar^2);
    z_dot(robot_num) = -z_dot(robot_num-1);
    z
	z = z + z_dot * dt;
    
    pos_val(1) = z(1);
    pos_val(2) = z(2);
    pos_val(3) = z(3);
    
    targetplt1 = plot(real(z_target(1)), imag(z_target(1)), 'ro');
    hold on;
    targetplt2 = plot(real(z_target(2)), imag(z_target(2)), 'bo');
    hold on;
    targetplt3 = plot(real(z_target(3)), imag(z_target(3)), 'go');
    hold on;
    
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






 