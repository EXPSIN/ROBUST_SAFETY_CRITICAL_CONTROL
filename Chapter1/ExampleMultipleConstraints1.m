%% MATLAB Version R2017b
clear all; clc; close all;
addpath('Auxiliary/');
Time= 5;               % simulation time, sec
T   = 5e-3;             % step, sec
N   = floor(Time/T);    % steps
t   = 0;
% agent and obstacle setting
na  = 1;            % number of mobile agents
no  = 2;            % number of obstacles
nao = na+no; 
p = zeros(3, nao);  % position of agents and obstalces, 2D 
x = zeros(4, na);   % actuation state of agent
v_set = zeros(2, na);   % the input of the actuation
delta = zeros(1, na);   % the slack variable

% define the initial position
p = [[-2.0; 2.0], ...      % position of agent-1
     [ 0.5; 0.5], ...
     [-0.5;-0.5]];        % position of obstacle-1
 
Do  = 0.65;

vp = [1; -1];
% vp = vp/norm(vp);
v_set(:, 1) = vp;

 
 
% quadraitc programming setting
Ap_i= getPolyAb(5, 1, 0);      % 
ap_i= zeros(size(Ap_i, 1), na);
A   = zeros(nao-1, 2, na);
b   = zeros(nao-1, 1, na);
% Ds  = sqrt(2)*Do - 1e-3;
Ds  = sqrt(2)/2;

% initialize graphic
% H = myGraphic([], p, x, Ds, Ap_i, ap_i, delta);

Nx = 301;
% Nx = 1001;
k_map = 1.0;
x1 = k_map*linspace(-Do*6/5, Do*6/5, Nx); 
x2 = k_map*linspace(Do*6/5, -Do*6/5, Nx);

% x1 = k_map*linspace(-1, 1, Nx); 
% x2 = k_map*linspace( 1,-1, Nx);
a_partial_spec  = zeros(Nx, Nx);
aO_partial_spec = zeros(Nx, Nx);

for k1 = 1:size(x1, 2)
    k1/size(x1, 2)
    for k2 = 1:size(x2, 2)
        if(mod(k1, 2) == 1)
            k2 = size(x2, 2)- k2 + 1;
        end
        p(:, 1) = [x1(k1); x2(k2)];
        
%         if(norm(p(:, 1)-p(:, 2)) <= Do || norm(p(:, 1)-p(:, 3)) <= Do || k1 ~= k2)
        if(norm(p(:, 1)-p(:, 2)) <= Do || norm(p(:, 1)-p(:, 3)) <= Do )
%         if(norm(p(:, 1)-p(:, 2)) <= Ds || norm(p(:, 1)-p(:, 3)) <= Ds )
            continue;
        end
        
%         pause(1);
        [a_partial_spec(k2, k1), aO_partial_spec(k2, k1), adata] = spectral_norm(p, vp, Ds, Ap_i);
%         [v_set, delta, ap_i, A, b, v_set_detO] = collision_avoidance(p, vp, Ds, Ap_i, iA);
        % H = myGraphic(H, p, x, Ds, Ap_i, ap_i, delta);
        
        if(max(a_partial_spec(k2, k1), aO_partial_spec(k2, k1)) > 1e3)
            1;
        end
    end
end
% showpartial_contour(x1, x2, a_partial_spec, aO_partial_spec)
showpartial_surf(x1, x2, a_partial_spec, aO_partial_spec);



function showpartial_surf(x1, x2, a_partial_spec, aO_partial_spec)
% obstacles
Do = evalin('base', 'Do');
Ds = evalin('base', 'Ds');
na = evalin('base', 'na');
no = evalin('base', 'no');
p  = evalin('base', 'p');
N  = 1e3;

blue_map_edited = 1- hot(1000);
blue_map_edited = blue_map_edited(1:750, :);
[X1,X2] = meshgrid(x1,x2);
upper = max(max(max(a_partial_spec)), max(max(aO_partial_spec)));
% contourf(X1, X2, a_partial_spec, 'EdgeColor', 'none'); hold on;
% surf(X1, X2, a_partial_spec, 'EdgeColor', 'none'); hold on;  view(2);


figure(7); 
clf; 
set(gcf, 'position', [1500, 0, 650, 500 ], 'color', 'w');
% contourf(X1, X2, aO_partial_spec, 'EdgeColor', 'none');
surf(X1, X2, aO_partial_spec, 'EdgeColor', 'none'); hold on; view(2);
hold on;
for iO = (na+1):(na+no)
%     H.shapeO(iO) = patch(p(1,iO)+Ds*sin(linspace(0, 2*pi, 30)), p(2,iO)+Ds*cos(linspace(0, 2*pi, 30)), 'c', 'facealpha', 0.2);
%     H.realO(iO)  = patch(p(1,iO)+Do*sin(linspace(0, 2*pi, N)), p(2,iO)+Do*cos(linspace(0, 2*pi, N)), 'w', 'facealpha', 0.0, 'linewidth', 2);
    H.realO(iO)  = patch(p(1,iO)+Do*sin(linspace(0, 2*pi, N)), p(2,iO)+Do*cos(linspace(0, 2*pi, N)), 10*ones(1, N),  'w', 'facealpha', 0.1, 'linewidth', 0.5);
    H.shapeO(iO) = patch(p(1,iO)+Ds*sin(linspace(0, 2*pi, N)), p(2,iO)+Ds*cos(linspace(0, 2*pi, N)), 10*ones(1, N),  'w', 'facealpha', 0.1, 'linewidth', 0.5);
    H.f1_obs{iO} = hatchfill(H.realO(iO), 'single', 45, 10, [1.0, 1.0, 1.0]);
    H.f1_obs{iO}.HandleVisibility = 'off';
    % H.f1_obs{iO}.ZData = ones(size(H.f1_obs{iO}.XData));
end
% axis equal;


H.posO = plot(p(1, na+1:end), p(2, na+1:end), 'k.', 'markersize', 5, 'linestyle', 'none');
% text(p(1,2), p(2,2)-0.05, '$p_2$', 'fontsize', 20,'interpreter','latex', 'HorizontalAlignment', 'center');
% text(p(1,3), p(2,3)-0.05, '$p_3$', 'fontsize', 20,'interpreter','latex', 'HorizontalAlignment', 'center');
text(p(1,2), p(2,2)-0.05, '$o_1$', 'fontsize', 20, 'HorizontalAlignment', 'center', 'Interpreter','latex');
text(p(1,3), p(2,3)-0.05, '$o_2$', 'fontsize', 20, 'HorizontalAlignment', 'center', 'Interpreter','latex');

axis equal;
% xlabel('$[p_1]_{1}$','interpreter','latex', 'FontSize', 18); 
% ylabel('$[p_1]_{2}$','interpreter','latex', 'FontSize', 18); 
xlabel('$[p]_{1}$', 'FontSize', 17,'interpreter','latex');  
ylabel('$[p]_{2}$', 'FontSize', 17,'interpreter','latex'); 
caxis manual; caxis([0 upper]); 
c = colorbar;
c.Label.String = '$|v^*|$';
c.Label.Interpreter = 'latex';
set(gca, 'fontsize', 16);
colormap(blue_map_edited);
% set(gca, 'position',[0.01,0.05,0.85,0.9]);
axis equal
axis([-0.6,0.6,-0.6,0.6]*1.21);
set(gca, 'xtick', get(gca, 'ytick'), 'box', 'on');
hold on
end

%% end of main.

%% 
function ap = gen_a(Ap, A, a, k_tau)
N  = size(Ap, 1);
ap = zeros(N, 1);
cos_tau = cos(2*pi/N);              % 最低角度
A_unit     = unit_vector(A);        % R^{m x n}, a\in R{m x 1}
polyA_unit = unit_vector(Ap);       % R^{N x n}
cos_theta  = polyA_unit*A_unit';    % R^{N x m}

% R^{N x m}  R^{m x 1}
% ap = max(cos_theta.*a' + limit(cos_tau-cos_theta, 0, 1).*(a'-k_tau), [], 2); % 选取每一行的最小元素，组成列向量。
chi = cos_theta.*a' - limit(1*(cos_tau-cos_theta), 0, 1).*(k_tau+cos_theta.*a');
% chi = max(chi, -k_tau);
ap  = max(chi, [], 2); % 选取每一行的最小元素，组成列向量。
% ap = max(cos_theta.*a' + limit(1000e3*(cos_tau-cos_theta), 0, 1).*(a'-k_tau), [], 2); % 选取每一行的最小元素，组成列向量。
end

function [v_set_da, v_set_daO, data] = spectral_norm(p, vp, Ds, Ap_i)
persistent opt;
if(isempty(opt))
    opt = optimoptions('quadprog',  'Algorithm','interior-point-convex','Display','off', ...
        'MaxIterations', 1e3, 'OptimalityTolerance', 1e-12);
end
pm    = p(:, 1);
v_aim = vp(:, 1);
nao   = size(p, 2);
delta_bar       = 1e2;
alpha_v         = 1;

% generate conditions
A = zeros(nao-1, 2);
b = zeros(nao-1, 1);
count = 0;
for k = 1:nao
    if(k == 1)
        continue;
    end
    count = count + 1;
    p_delta = pm - p(:, k);     % relative position
    dist    = norm(p_delta);    % distance 
    h = dist - Ds;              % barrier function 
    V = 1/(h+Ds);               % Lyapunov function
    % condition - k
    A(count, :) = -p_delta'/norm(p_delta);
    b(count, :) = -alpha_v*(1/dist-1/Ds);
%     b(count, :) = alpha_v*(dist^2-Ds^2) /delta_bar;
%     b(count, :) = 2*alpha_v*(dist-Ds) /delta_underline;
end

% quadratic programming
f_aim = [v_aim];
ap_i  = gen_a(Ap_i, A, -b, 2.5/delta_bar);  % Ap_i x + a delta \le 0
M     = [Ap_i, ap_i];


% % Quadratic Programming with Slack Variables - Lipschitz
% [v_set_det, f, exitflag, output, lambda] = quadprog(...
%     diag([1, 1, 1]), -f_aim,...                             % cost function
%     M, zeros(size(M, 1), 1), [], [],  ...                   % constraints
%     [-inf; -inf; 0], [inf; inf; inf], zeros(3, 1), opt);    % limits and setting
% activeIndex = abs(M*v_set_det) <= 1e-8;
% if(v_set_det(3) >= 1e-2)
%     cidx = find(activeIndex,2);
%     activeIndex = false(size(activeIndex));
%     activeIndex(cidx) = true;
% else
%     1;
% end
% 
% % activeIndex = getMin2(abs(M*v_set_det), 1e-8);
% [v_set_det, v_set_da] = calculate_solution_derivative(M, f_aim, v_set_det, activeIndex, delta_bar);
% % v_set_da = 0;

% Standard Quadratic Programming with Slack Variables
[v_set_detO, fO, exitflagO, outputO, lambdaO] =  quadprog(...
    diag([1, 1]), -v_aim,...                             % cost function
    A, b, [], [],  ...             % constraints
    [-inf; -inf], [inf; inf], zeros(2, 1), opt);    % limits and setting


v_set_daO = norm(v_set_detO(1:2));
v_set_da  = v_set_daO;


% activeIndexO = abs([A, -b]*v_set_detO) <= 1e-8;
% [v_set_detO, v_set_daO] = calculate_solution_derivative([A, -b], f_aim, v_set_detO, activeIndexO, delta_bar);
data  =[];
% if(pm(1) > 0.1 && pm(2) < 0)
%     1;
% end
% data.A    = A;
% data.b    = b*v_set_detO(3);
% data.Ap_i =  Ap_i;
% data.ap_i = -ap_i*v_set_det(3);
% 
% v_set_da  = norm(v_set_det(1:2));
% v_set_daO = norm(v_set_detO(1:2));
end

function [v_set_det, v_set_da] = calculate_solution_derivative(M, f_aim, res, activeIndex, k_delta)
Ma = M(activeIndex, :);
A = Ma(:, 1:end-1);
a = Ma(:, end);
m = size(Ma, 1);

if(m == 0)
    v_set_det = res;
    v_set_da  = 0;
    return;
end

if(m > 2)
    v_set_det = res;
    v_set_da  = -1;
    fprintf('No solution.\n');
    return;
end

v_set_det = (eye(3)-Ma'/(Ma*Ma')*Ma) * f_aim;
v_p = f_aim(1:2);
% k   = f_aim(3);

if(norm(res - v_set_det) > 1e-2)
    v_set_da  = 0;    
    warning('error res, %f\n', norm(res - v_set_det));
%     return;
end

Ax = A'/(A*A');
Bx = eye(size(a, 2));
Cx = (A*A')\A*v_p;
% Kmn = commutation_matrix(a);
Kmn = eye(size(a, 1));

% Lambda0 = (1+a'/(A*A')*a);
% v_set_dan2 =  -( (kron((Bx*a'*Cx)',Ax) + kron(Cx',Ax*a*Bx)*Kmn)*Lambda0 - 2*(A'/(A*A')*(a*a')/(A*A')*A)*v_p*(a'/(A*A')) )  / Lambda0^2 ... % vp;
%     + ( (A'/(A*A')*k_delta) * (1+a'/(A*A')*a) - 2*(A'/(A*A')*a*k_delta) * (a'/(A*A')) ) /Lambda0^2;



% 备份 2021 0706
Lambda0 = A*A';
Lambda1 = (1+a'/Lambda0*a);
Lambda2 = A'/Lambda0;
Lambda3 = (a*a');
v_set_dan = ...
    +( 2*(Lambda2*Lambda3*Lambda2'*v_p)*(a'/Lambda0) )  / Lambda1^2 ... % vp;    
    +( k_delta*Lambda2*(Lambda1 - 2*(a*a') /Lambda0) ) /Lambda1^2 ...    
    -( (kron(v_p'*Lambda2*a, Lambda2) + kron(v_p'*Lambda2, Lambda2*a)*Kmn)*Lambda1  )  / Lambda1^2;
% norm(v_set_dan-v_set_dan2)

v_set_da = norm(v_set_dan);
% v_set_da = 5;
end

function Kmn = commutation_matrix(A)
[m, n] = size(A);
I = reshape(1:m*n, [m, n]); % initialize a matrix of indices of size(A)
I = I'; % Transpose it
I = I(:); % vectorize the required indices
Kmn = eye(m*n); % Initialize an identity matrix
Kmn = Kmn(I,:); % Re-arrange the rows of the identity matrix
end

% 获取二维点
function A = getPolyAb(N, Radius, theta_bias)
% 2维情况
theta    = linspace(theta_bias, theta_bias+2*pi, N+1)';
% position = Radius*[cos(theta(1:end-1)), sin(theta(1:end-1))];
% [A, ~]   = getlines(position);
A = [cos(theta(1:end-1)), sin(theta(1:end-1))];
% fprintf('幂集中的有效集合的最小奇异值： %f \n', min(matrix_minsvd(A)));
end
% 获取线
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
% 获取矩阵
function M = unit_vector(M)
for idx = 1:size(M,1)
    M(idx, :) = M(idx, :)/ norm(M(idx, :));
end
end
% 限幅
function res = limit(res, lower, upper)
res = min(max(res, lower), upper);
end
function res = maxLog10_0(in)
res = max(log10(in), 0);
end
function flag = getMin2(vals, e)
[vals_sort, idx] = sort(vals);
valued_flag = idx(vals_sort < e);
if(size(valued_flag, 1) > 2)
    valued_flag = valued_flag(1:2, 1);
end
flag = false(size(vals));
flag(valued_flag) = true;
end