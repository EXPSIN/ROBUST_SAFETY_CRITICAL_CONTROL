clear all; clc; close all;
clear myGraphic;
clear drawMultiQuad;
addpath('Auxiliary/');
Time= 10;
T   = 10e-3;
N   = floor(Time/T);
t   = 0;

p1=[[ 1.5; 0.15], ...
    [ 0.0; 0.0]];
vp = [-2; 0];
p2 = p1;

na  = 1;
no  = size(p1, 2)-1;
nao = na+no;
x1  = zeros(8, na);
x2  = zeros(8, na);
vp1 = zeros(2, na);
vp2 = zeros(2, na);
v_set1 = zeros(2, na);
v_set2 = zeros(2, na);
delta1 = zeros(1, na);
delta2 = zeros(1, na);

Ap_i= getPolyAb(11, 1, 0);
ap_i= zeros(size(Ap_i, 1), na);
A1   = zeros(nao-1, 2, na);
b1   = zeros(nao-1, 1, na);
A2   = zeros(nao-1, 2, na);
b2   = zeros(nao-1, 1, na);

Ds  = 0.3;
Do  = 0.2;
alpha_v = 1.75;
vpd_bar = 1;
vp_bar  = 1;
aim_point = [-1; 0];

H = myGraphic([], p1, x1, p2, x2, Ds, Ap_i, ap_i);

for k = 1:N
    t = k*T;

    k_vp = 0.5;
    vp1(:, 1)= rungekutta(@(x,u)m_idealvelocity(x, u, vp_bar, vpd_bar), vp1(:, 1), k_vp*(aim_point - p1(:, 1)), T);
    vp2(:, 1)= rungekutta(@(x,u)m_idealvelocity(x, u, vp_bar, vpd_bar), vp2(:, 1), k_vp*(aim_point - p2(:, 1)), T);

    [v_set1(:,1), A1(:,:,1), b1(:,:,1)] = collision_avoidance_no_uncertainty(p1, vp1, Ds, alpha_v);
    [v_set2(:,1), A2(:,:,1), b2(:,:,1)] = collision_avoidance(p2, vp2, Ds, alpha_v);
    x1(:, 1) = rungekutta(@m_speed,    x1(:, 1), v_set1(  :, 1), T);
    x2(:, 1) = rungekutta(@m_speed,    x2(:, 1), v_set2(  :, 1), T);

    C=[ 0.7761   -1.9815         0         0    2.1315    2.4144         0         0
        0         0   -2.2000   -2.8153         0         0   -1.5060   -0.9899];
    p1(:, 1) = rungekutta(@m_position, p1(:, 1),     v_set1(  :, 1), T);
    p2(:, 1) = rungekutta(@m_position, p2(:, 1),     C*x2, T);


    H = myGraphic(H, p1, x1, p2, x2, Ds, Ap_i, ap_i);
end

function [v_set, A, b] = collision_avoidance(p, vp, Ds, alpha_v)
persistent opt;
if(isempty(opt))
    opt = optimoptions('quadprog',  'Display','off', 'MaxIterations', 1e3);
end

pm    = p(:, 1);
v_aim = vp(:, 1);
nao   = size(p, 2);

A = zeros(nao-1, 2);
b = zeros(nao-1, 1);
count = 0;
for k = 2:nao
    count = count + 1;
    p_delta = pm - p(:, k);
    dist    = norm(p_delta, 2);
    h = dist - Ds;
    V = 1/(h+Ds);
    A(count, :) = -p_delta'/norm(p_delta);
    b(count, :) = -alpha_v*(V-1/Ds);
    b(count, :) = alpha_v*(dist-Ds);
end

v_set = quadprog(...
    diag([1, 1]), -v_aim,...
    A, b, [], [],  ...
    [-inf; -inf], [inf; inf], zeros(2, 1), opt);
end

function [v_set, A, b] = collision_avoidance_no_uncertainty(p, vp, Ds, alpha_v)
persistent opt;
if(isempty(opt))
    opt = optimoptions('quadprog',  'Display','off', 'MaxIterations', 1e3);
end

pm    = p(:, 1);
v_aim = vp(:, 1);
nao   = size(p, 2);

A = zeros(nao-1, 2);
b = zeros(nao-1, 1);
count = 0;
for k = 2:nao
    count = count + 1;
    p_delta = pm - p(:, k);
    dist    = norm(p_delta, 2);
    h = dist - Ds;
    V = 1/(h+Ds);
    A(count, :) = -p_delta'/norm(p_delta);
    b(count, :) = -alpha_v*(V-1/Ds);
    b(count, :) = alpha_v*(dist-Ds);
end

v_set = quadprog(...
    diag([1, 1]), -v_aim,...
    A, b, [], [],  ...
    [-inf; -inf], [inf; inf], zeros(2, 1), opt);
end

function H = myGraphic(H, p1, x1, p2, x2, Ds, Ap_i, ap_i)
persistent crashflag_2;
if(isempty(crashflag_2))
    crashflag_2 = false;
end

nao = size(p1, 2);
na  = 1;
no  = nao - na;
Do  = evalin('base', 'Do');
drawnow limitrate;
if(isempty(H))
    figure(1); clf;
    axis equal;
    hold on;
    grid minor;
    set(gcf, 'position', [0,500, 800, 240 ], 'color', 'w');
    set(gca, 'fontsize', 16);

    text(-1.0, 0, '{\bf\circ}', 'fontsize', 24, 'HorizontalAlignment', 'center');

    drawMultiQuad(p1(1, 1), p1(2, 1), 0.0, 0, 0, 0, [0,0,0], 0.15, 0.06, 1);

    for iO = (na+1):(na+no)
        H.shapeO(iO) = patch(p1(1,iO)+Ds*sin(linspace(0, 2*pi, 30)), p1(2,iO)+Ds*cos(linspace(0, 2*pi, 30)), 'c', 'facealpha', 0.1, 'HandleVisibility', 'off', 'linestyle', '--', 'EdgeColor', 'k');
        H.realO(iO)  = patch(p1(1,iO)+Do*sin(linspace(0, 2*pi, 30)), p1(2,iO)+Do*cos(linspace(0, 2*pi, 30)), 'k', 'facealpha', 1.0, 'HandleVisibility', 'off');
        H.shapeO_hatch(iO) = hatchfill(H.shapeO(iO), 'single', 45, 5, [1.0, 1.0, 1.0]);
    end
    H.posO = plot(p1(1, na+1:end), p1(2, na+1:end), 'k.', 'markersize', 5, 'linestyle', 'none', 'HandleVisibility', 'off');

    HandleVisibility  = 'on';
    H.trajA(1) = animatedline('color', 'b', 'HandleVisibility', HandleVisibility, 'linewidth', 2, 'linestyle', '-', 'marker', 'none', 'Displayname', 'RPRF');
    H.trajA(2) = animatedline('color', 'k', 'HandleVisibility', HandleVisibility, 'linewidth', 2, 'linestyle', ':', 'marker', 'none', 'Displayname', 'RP');
    H.trajA(3) = animatedline('color', 'k', 'HandleVisibility', HandleVisibility, 'linewidth', 2, 'linestyle', '--', 'marker', 'none', 'Displayname', 'RP after crash');
    axis([-1.2, 1.7, -0.4, 0.5]);
    xlabel('$[p_1]_1$ (m)','Interpreter','latex'); ylabel('$[p_1]_2$ (m)','Interpreter','latex');
    axis off;
    set(gca, 'position', [0.05, 0.05, 0.9,  0.9])



else
    t     = evalin('base', 't');
    T     = evalin('base', 'T');

    delta1= evalin('base', 'delta1');
    v1_real= x1(3:4, :);
    A1    = evalin('base', 'A1');
    b1    = evalin('base', 'b1');
    v_set1 = evalin('base', 'v_set1');
    vp1   = evalin('base', 'vp1');

    delta2= evalin('base', 'delta2');
    v2_real= x2(3:4, :);
    A2    = evalin('base', 'A2');
    b2    = evalin('base', 'b2');
    v_set2 = evalin('base', 'v_set2');
    vp2   = evalin('base', 'vp2');

    H.A.XData = [p1(1, 1), p2(1, 1)];
    H.A.YData = [p1(2, 1), p2(2, 1)];

    addpoints(H.trajA(1), p1(1, 1), p1(2, 1));
    if(min(vecnorm(p2(:, 1) - p2(:, 2:end))) < Ds || crashflag_2)
        addpoints(H.trajA(3), p2(1, 1), p2(2, 1));
        crashflag_2 = true;
    else
        addpoints(H.trajA(2), p2(1, 1), p2(2, 1));
    end

end
end

function dx = m_speed(x, u)
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

function x = rungekutta(fun, x0, u, h)
k1 = fun(x0       , u);
k2 = fun(x0+h/2*k1, u);
k3 = fun(x0+h/2*k2, u);
k4 = fun(x0+  h*k3, u);
x = x0 + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function  res = limit(res, lower, upper)
res = min(max(res, lower), upper);
end

function ap = gen_a(Ap, A, a, tau)
N  = size(Ap, 1);
ap = zeros(N, 1);
cos_tau = cos(2*pi/N);
A_unit     = unit_vector(A);
polyA_unit = unit_vector(Ap);
cos_theta  = polyA_unit*A_unit';
chi = cos_theta.*a' - limit(1*(cos_tau-cos_theta), 0, 1).*(tau+cos_theta.*a');
ap  = max(chi, [], 2);
end

function A = getPolyAb(N, Radius, theta_bias)
theta    = linspace(theta_bias, theta_bias+2*pi, N+1)';
position = Radius*[cos(theta(1:end-1)), sin(theta(1:end-1))];
[A, ~]   = getlines(position);
end

function [A, a] = getlines(p)
N = size(p, 1);
m = size(p, 2);
A = zeros(N, m);
a = -ones(N, 1);
for idx = 1:N
    p_index   = mod((idx:idx+(m-1))-1, N) + 1;
    p_tmp     = p(p_index, :);
    A(idx, :) = (p_tmp\ones(m, 1))';
    len = norm(A(idx, :));
    A(idx, :) = A(idx, :)/len;
    a(idx, :) = a(idx, :)/len;
end
end


function  M = unit_vector(M)

for idx = 1:size(M,1)
    M(idx, :) = M(idx, :)/ norm(M(idx, :));
end

end


function res = maxLog10_0(in)
res = max(log10(in), 0);
end

function dx = m_idealvelocity(x, u, vp_bar, vpd_bar)
T   = 0.5;
x_r = u/norm(u)*min(norm(u), vp_bar);
dx = -1/T*(x - x_r);
dx = dx/norm(dx)*min(norm(dx), vpd_bar);
end

function handle = drawMultiQuad(x, y, z, roll, pitch, yaw, CLR, L, R, N)
figure(1); hold on;
persistent lastHandle keyPoint planeCNT motor cntPoints cntCP handleTrack QuadPoints;
if(~isempty(lastHandle))
else
    keyPoint = [L/2, L/2, 0; -L/2, L/2, 0;  -L/2, -L/2, 0;  L/2, -L/2, 0];
    planeCNT = N;
    cirPoints = [R*sin(linspace(0, 2*pi, 20))', R*cos(linspace(0, 2*pi, 20))'];
    cntCP = size(cirPoints, 1);
    cirPoints(:,3) = 0;
    for mIdx = 1:4
        motor(cntCP*(mIdx-1)+1:cntCP*mIdx,:) = keyPoint(mIdx,:) + cirPoints;
    end
    QuadPoints = [keyPoint; motor];
    cntPoints = size(QuadPoints, 1);

    for idx = 1:planeCNT
        tempHandle = animatedline('Color',CLR(idx,:),'LineWidth',1, 'LineStyle', '--', 'HandleVisibility', 'off');
        handleTrack = [handleTrack, tempHandle];
    end
end
drawnow;
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
