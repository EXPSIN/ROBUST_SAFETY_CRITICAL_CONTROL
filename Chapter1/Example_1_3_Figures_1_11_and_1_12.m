clear all; close all; clc;
addpath('Auxiliary/');
Time= 20;
T   = 50e-3;
N   = floor(Time/T);
t   = 0;
p0  = 3.0;
p   = p0;
o_1 = 0.0;
x   = zeros(2, 1);
k_h = [0.5, 1, 1, 1];
D_s = [1.0, 1, 2, 2];
D   = 0.5;

figure(1);
set(gcf, 'position', [10, 10, 800, 400], 'color', 'w');
hold on;
grid on;
axis([0, Time, -D, 5.0]);
xlabel('$t~(s)$', 'Interpreter', 'latex');
ylabel('$p~(m)$', 'Interpreter', 'latex');
set(gca, 'fontsize', 16);
box on;


px = [  0, Time, Time,  0,   0];
py = [ -D,  -D,     D,  D,  -D];
patch(px,  py, 'c', 'facecolor', [1,1,1]*0.5, 'facealpha', 0.2, 'displayname', 'Unsafe set', 'handlevisibility', 'off', 'linestyle', 'none');
klinestyle = {'-', '--', ':', '-.'};
for i_Sim  = 1:length(k_h)
    H(i_Sim) = animatedline('color', 'k', 'displayname', sprintf('$k_h=%.1f,~D_s=%.1f$', k_h(i_Sim), D_s(i_Sim)), 'linewidth', 2, 'linestyle', klinestyle{i_Sim});
    plot([0, Time], [D_s(i_Sim),  D_s(i_Sim)], 'b--', 'linewidth', 0.5, 'HandleVisibility', 'off');
end
H(length(k_h)).DisplayName = [H(length(k_h)).DisplayName, ' ($v\equiv v^*$)'];
plot([0, Time], [o_1,  o_1], 'k--', 'linewidth', 0.5, 'displayname', '$o_1$', 'HandleVisibility', 'off');
plot([0, Time], [  D,    D], 'b--', 'linewidth', 0.5, 'displayname', '$o_1$', 'HandleVisibility', 'off');

set(gca, 'position', [0.1, 0.2, 0.85, 0.7]);
lH = legend('location', 'northeast');
set(lH, 'Interpreter','latex');


figure(2);
set(gcf, 'position', [810, 10, 800, 400], 'color', 'w');
hold on;
grid on;
xlim([0, Time]);
ylim([-0.3,0.8]);
xlabel('$t~(s)$', 'Interpreter', 'latex');
ylabel('$\tilde{v}~(m/s)$', 'Interpreter', 'latex');
set(gca, 'fontsize', 16);
box on;

klinestyle = {'-', '--', ':', '-.'};
for i_Sim  = 1:length(k_h)
    H_vtilde(i_Sim) = animatedline('color', 'k', 'displayname', sprintf('$k_h=%.1f,~D_s=%.1f$', k_h(i_Sim), D_s(i_Sim)), 'linewidth', 2, 'linestyle', klinestyle{i_Sim});
end
H_vtilde(length(k_h)).DisplayName = [H_vtilde(length(k_h)).DisplayName, ' ($v\equiv v^*$)'];

set(gca, 'position', [0.1, 0.2, 0.85, 0.7]);
lH_vtilde = legend('location', 'northeast');
set(lH_vtilde, 'Interpreter','latex');


for i_Sim  = 1:length(k_h)
    p   = p0;
    x   = zeros(2, 1);
    for k = 1:N
        t = k*T;

        v_set = collision_avoidance(p, -1, D_s(i_Sim), k_h(i_Sim), o_1);

        x = rungekutta(@m_speed,    x, v_set, T);
        C=[-1, 1];

        if(i_Sim == length(k_h))
            p = rungekutta(@m_position, p,  v_set, T);
            addpoints(H(i_Sim), t, p);
            addpoints(H_vtilde(i_Sim), t, 0);
        else
            p = rungekutta(@m_position, p,  C*x, T);
            addpoints(H(i_Sim), t, p);
            addpoints(H_vtilde(i_Sim), t, C*x-v_set);
        end



        drawnow limitrate;
    end
end



function dx = m_speed(x, u)
A=[-2, -2;  1, 0];
B=[2; 0];
dx = A*x + B*u;
end

function dx = m_position(x, u)
A  = 0;
B  = 1;
dx = A*x + B*u;
end

function x = rungekutta(fun, x0, u, h)
k1 = fun(x0       , u);
k2 = fun(x0+h/2*k1, u);
k3 = fun(x0+h/2*k2, u);
k4 = fun(x0+  h*k3, u);
x = x0 + h/6*(k1 + 2*k2 + 2*k3 + k4);
end


function v_set= collision_avoidance(p, v_c, Ds, k_h, o_1)
persistent opt;
if(isempty(opt))
    opt = optimoptions('quadprog',  'Algorithm','interior-point-convex','Display','off', 'MaxIterations', 1e3);
end

A = zeros(1, 1);
A(1, :) = -(p-o_1)'/norm(p-o_1);
b(1, :) = -k_h*(1/norm(p-o_1)-1/Ds);

v_set = quadprog(...
    1, -v_c, ...
    A,   b, [], [],  ...
    [], [], zeros(1, 1), opt);

if(isempty(v_set))
    1;
end
end
