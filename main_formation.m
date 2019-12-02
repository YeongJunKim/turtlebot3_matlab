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
dt = 0.1;           % Sampling time, 단위: 초
robot_num = 3;      % 로봇 개수

alpha = 1;          % 식 (12)
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
    
    % Determine weight values (식 (5)나 그 위에있는 내용 참고)
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
    % L의 rank는 2로 가주앙
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






 