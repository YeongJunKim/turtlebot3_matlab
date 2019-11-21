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

now_pos = [];
now_pos(1).data = [pose1(1) + pose1(2) * 1i];
now_pos(2).data = [pose2(1) + pose2(2) * 1i];
now_pos(3).data = [pose3(1) + pose3(2) * 1i];

appended_data = [];
appended_data.pose(1).data = now_pos(1).data;
appended_data.pose(2).data = now_pos(2).data;
appended_data.pose(3).data = now_pos(3).data;
appended_data.weight = zeros(robot_num, robot_num);

%% Simulation
N = 1;
for i = 1:N
    % weight matrix
    weight = zeros(robot_num, robot_num);
    
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
        disp(j);
        disp(k);
        random_weight = (rand()*10 + rand()*10*1i);
        weight(i, j) = random_weight*(now_pos(j).data - now_pos(i).data);
        random_weight = (rand()*10 + rand()*10*1i);
        weight(i, k) = random_weight*(now_pos(k).data - now_pos(k).data);
        weight = weight + weight.';
        
    end
    
    % Determine L (Laplacian matrix)
    
    
    % Design K
    
    
%     z_dot = ?
%     z = z + z_dot * dt;
%     
%     figure(1);
%     plot(real(z(1)),imag(z(1)),'ro'); hold on;
%     plot(real(z(2)),imag(z(2)),'bo');
%     plot(real(z(3)),imag(z(3)),'go'); hold off;
%     drawnow;
    
end