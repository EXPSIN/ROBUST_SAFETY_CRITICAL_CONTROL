clear all;
clc;
close all;
addpath('Auxiliary/');
po=[-0.2;0];%障碍物位置
p1n=[1;0];%无人机1初始位置
p2n=[0.1;0];%无人机2初始位置
Do=0.2;%障碍物半径
Dq=0.1;%无人机半径

aim1=[0.3;0];%无人机1目标位置
aim2=[0.1;0];%无人机2目标位置

alphav=1.75;%k类函数
step=0.0001;%步长
times=30000;%迭代次数

p1=zeros(2,times);
p2=zeros(2,times);
v1=zeros(2,times);
v2=zeros(2,times);%位置与速度统计
p1(:,1)=p1n;
p2(:,1)=p2n;

x1  = zeros(8, 1);   % 无人机1的状态量
x2  = zeros(8, 1);   % 无人机2的状态量

k=1;%循环计数
while k<=times
    %计算标称速度
    if norm(aim1-p1(:,k))>=1
        vp1=(aim1-p1(:,k))/norm(aim1-p1(:,k));
    else
        vp1=aim1-p1(:,k);
    end
    if norm(aim2-p2(:,k))>=1
        vp2=(aim2-p2(:,k))/norm(aim2-p2(:,k));
    else
        vp2=aim2-p2(:,k);
    end
    %安全控制器
    [v_set1] = collision_avoidance(p1(:,k),p2(:,k),po, 1.5*vp1, Do,Dq, 2*alphav);
    [v_set2] = collision_avoidance(p2(:,k),p1(:,k),po, 1.5*vp2, Do,Dq, 2*alphav);

    x1(:,1)=m_speed(x1(:,1),v_set1)*step+x1(:,1);
    x2(:,1)=m_speed(x2(:,1),v_set2)*step+x2(:,1);

    C=[ 0.7761   -1.9815         0         0    2.1315    2.4144         0         0
        0         0   -2.2000   -2.8153         0         0   -1.5060   -0.9899];
    p1(:,k+1)=m_position(p1(:, k),C*x1(:,1))*step+p1(:,k);
    p2(:,k+1)=m_position(p2(:, k),C*x2(:,1))*step+p2(:,k);

    k=k+1;

end

figure(1)
hold on;
axis equal;
box off;
ax=gca
ax.XAxis.Visible='off';
ax.YAxis.Visible='off';
% ax.ZAxis.Visible='off';
% plot(po(1)+Do*cos(0:0.01*pi:2*pi),po(2)+Do*sin(0:0.01*pi:2*pi));
% plot(po(1)+(Do+Dq)*cos(0:0.01*pi:2*pi),po(2)+(Do+Dq)*sin(0:0.01*pi:2*pi));
plot(p1(1,:),p1(2,:),'b','LineWidth',2);
plot(p2(1,:),p2(2,:),'r','LineWidth',2);
drawMultiQuad(p1n(1, 1), p1n(2, 1), 0.0, 0, 0, 0, [0,0,0], 0.15, 0.04, 1);
drawMultiQuad(p2n(1, 1), p2n(2, 1), 0.0, 0, 0, 0, [0,0,0], 0.15, 0.04, 1);
H.shapeO(1) = patch(po(1,1)+(Do+Dq)*sin(linspace(0, 2*pi, 30)), po(2,1)+(Do+Dq)*cos(linspace(0, 2*pi, 30)), 'c', 'facealpha', 0.1, 'HandleVisibility', 'off', 'linestyle', '--', 'EdgeColor', 'k');
        H.realO(1)  = patch(po(1,1)+Do*sin(linspace(0, 2*pi, 30)), po(2,1)+Do*cos(linspace(0, 2*pi, 30)), 'k', 'facealpha', 1.0, 'HandleVisibility', 'off');
        H.shapeO_hatch(1) = hatchfill(H.shapeO(1), 'single', 45, 5, [1.0, 1.0, 1.0]);

function [v_set] = collision_avoidance(p1,p2,po, vp, Do,Dq, alphav)
persistent opt;
if(isempty(opt))
    %     opt = optimoptions('quadprog',  'Algorithm','interior-point-convex','Display','off', 'MaxIterations', 1e3);
    opt = optimoptions('quadprog',  'Display','off', 'MaxIterations', 1e3);
end

%输入当前位置信息p1 当前运动障碍物信息p2 静态障碍物位置po 当前标称速度vp 静态障碍物半径Do 无人机半径Dq

pm    = p1;
v_aim = vp;
nao   = 2;

% generate conditions
A = zeros(2, 2);
b = zeros(2, 1);
%运动障碍物约束条件
p_delta1=p1-p2;
h1=norm(p1-p2)-2*Dq;
A(1,:)=-p_delta1'/norm(p_delta1);
b(1, :) = 0.5*alphav*h1;
%静态障碍物约束条件
p_delta2=p1-po;
h2=norm(p1-po)-Dq-Do;
A(2,:)=-p_delta2'/norm(p_delta2);
b(2, :) = 0.5*alphav*h2;

% quadratic programming
[v_set,fval,exitflag,output] = quadprog(...
    diag([1, 1]), -v_aim,...                             % cost function
    A, b, [], [],  ...                   % constraints
    [-inf; -inf], [inf; inf], zeros(2, 1), opt);    % limits and setting

if exitflag==1
    v_set=v_set;
else
    v_set=[0;0];
end
end


function x = rungekutta(fun, x0, u, h)
% FcnHandlesUsed  = isa(fun,'function_handle');
k1 = fun(x0       , u);
k2 = fun(x0+h/2*k1, u);
k3 = fun(x0+h/2*k2, u);
k4 = fun(x0+  h*k3, u);
x = x0 + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function dx = m_speed(x, u)
% Model 210918, res_06.mat
A=[-1.5828    2.9188         0         0         0         0         0         0
    -2.9188   -1.5828         0         0         0         0         0         0
    0         0   -2.6833    7.1816         0         0         0         0
    0         0   -7.1816   -2.6833         0         0         0         0
    0         0         0         0   -2.5615    6.8558         0         0
    0         0         0         0   -6.8558   -2.5615         0         0
    0         0         0         0         0         0   -2.1391    3.7051
    0         0         0         0         0         0   -3.7051   -2.1391];

B=[ 1.6527         0
    0.6473         0
    1.4972         0
    0.9178         0
    0    1.5791
    0    0.8422
    0    1.5056
    0   -2.2906];
dx = A*x + B*u;
end

function dx = m_position(x, u)
A  = 0*[-1, 0; 0, -1];
B  = [1, 0; 0 1];
dx = A*x + B*u;
end



function handle = drawMultiQuad(x, y, z, roll, pitch, yaw, CLR, L, R, N)
figure(1); hold on;
% drawMultiQuad(0, 0, 0, 0, 0, 0, 2, 0.5, 'r')
persistent lastHandle keyPoint planeCNT motor cntPoints cntCP handleTrack QuadPoints;
if(~isempty(lastHandle))
%     delete(lastHandle);
else
    keyPoint = [L/2, L/2, 0; -L/2, L/2, 0;  -L/2, -L/2, 0;  L/2, -L/2, 0];
    planeCNT = N;
    cirPoints = [R*sin(linspace(0, 2*pi, 20))', R*cos(linspace(0, 2*pi, 20))'];
    cntCP = size(cirPoints, 1);   %cntCP
    cirPoints(:,3) = 0;
    for mIdx = 1:4
        motor(cntCP*(mIdx-1)+1:cntCP*mIdx,:) = keyPoint(mIdx,:) + cirPoints;
    end
    QuadPoints = [keyPoint; motor];
    cntPoints = size(QuadPoints, 1);
    
    % trajectory
    for idx = 1:planeCNT
        tempHandle = animatedline('Color',CLR(idx,:),'LineWidth',1, 'LineStyle', 'none', 'HandleVisibility', 'off');
        handleTrack = [handleTrack tempHandle];
    end
end

Frame = zeros(cntPoints, 3, planeCNT);

for i = 1:planeCNT
    if(roll(1) ~= 0 || pitch(1) ~= 0 || yaw(1) ~= 0)
        quaternion = angle2quat(roll(i), pitch(i), yaw(i), 'XYZ');
        for idx = 1:cntPoints
            Frame(idx, :, i) = [x(i), y(i), z(i)] + quatrotate(quaternion, QuadPoints(idx, :));
        end
    else
        Frame(:, :, i) = [x(i), y(i), z(i)] + QuadPoints(:, :);
    end
end
picHandle = [];
drawnow;
for i = 1:planeCNT
    addpoints(handleTrack(i), x(i), y(i), z(i));
    temp = line(Frame([1,3],1,i), Frame([1,3],2,i), Frame([1,3],3,i),'Color',CLR(i,:), 'LineWidth', 3, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
    temp = line(Frame([2,4],1,i), Frame([2,4],2,i), Frame([2,4],3,i),'Color',CLR(i,:), 'LineWidth', 3, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
    temp = line(Frame(5        :4+  cntCP,1,i), Frame(5        :4+  cntCP,2,i), Frame(5        :4+  cntCP,3,i),'Color',CLR(i,:), 'LineWidth', 1, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
    temp = line(Frame(5+  cntCP:4+2*cntCP,1,i), Frame(5+  cntCP:4+2*cntCP,2,i), Frame(5+  cntCP:4+2*cntCP,3,i),'Color',CLR(i,:), 'LineWidth', 1, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
    temp = line(Frame(5+2*cntCP:4+3*cntCP,1,i), Frame(5+2*cntCP:4+3*cntCP,2,i), Frame(5+2*cntCP:4+3*cntCP,3,i),'Color',CLR(i,:), 'LineWidth', 1, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
    temp = line(Frame(5+3*cntCP:4+4*cntCP,1,i), Frame(5+3*cntCP:4+4*cntCP,2,i), Frame(5+3*cntCP:4+4*cntCP,3,i),'Color',CLR(i,:), 'LineWidth', 1, 'HandleVisibility', 'off'); picHandle = [picHandle temp];
end
lastHandle = picHandle;
end