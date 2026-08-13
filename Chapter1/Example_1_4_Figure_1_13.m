clear all; clc; close all;
addpath('Auxiliary/');
Time= 5;
T   = 5e-3;
N   = floor(Time/T);
t   = 0;
na  = 1;
no  = 2;
nao = na+no;
p = zeros(3, nao);
x = zeros(4, na);
v_set = zeros(2, na);
delta = zeros(1, na);

p = [[-2.0; 2.0], ...
     [ 0.5; 0.5], ...
     [-0.5;-0.5]];

Do  = 0.65;

vp = [1; -1];
v_set(:, 1) = vp;



Ap_i= getPolyAb(5, 1, 0);
ap_i= zeros(size(Ap_i, 1), na);
A   = zeros(nao-1, 2, na);
b   = zeros(nao-1, 1, na);
Ds  = sqrt(2)/2;


Nx = 301;
k_map = 1.0;
x1 = k_map*linspace(-Do*6/5, Do*6/5, Nx);
x2 = k_map*linspace(Do*6/5, -Do*6/5, Nx);

a_partial_spec  = zeros(Nx, Nx);
aO_partial_spec = zeros(Nx, Nx);

for k1 = 1:size(x1, 2)
    k1/size(x1, 2)
    for k2 = 1:size(x2, 2)
        if(mod(k1, 2) == 1)
            k2 = size(x2, 2)- k2 + 1;
        end
        p(:, 1) = [x1(k1); x2(k2)];

        if(norm(p(:, 1)-p(:, 2)) <= Do || norm(p(:, 1)-p(:, 3)) <= Do )
            continue;
        end

        [a_partial_spec(k2, k1), aO_partial_spec(k2, k1), adata] = spectral_norm(p, vp, Ds, Ap_i);

        if(max(a_partial_spec(k2, k1), aO_partial_spec(k2, k1)) > 1e3)
            1;
        end
    end
end
showpartial_surf(x1, x2, a_partial_spec, aO_partial_spec);



function showpartial_surf(x1, x2, a_partial_spec, aO_partial_spec)
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


figure(7);
clf;
set(gcf, 'position', [50, 50, 650, 500 ], 'color', 'w');
surf(X1, X2, aO_partial_spec, 'EdgeColor', 'none'); hold on; view(2);
hold on;
for iO = (na+1):(na+no)
    H.realO(iO)  = patch(p(1,iO)+Do*sin(linspace(0, 2*pi, N)), p(2,iO)+Do*cos(linspace(0, 2*pi, N)), 10*ones(1, N),  'w', 'facealpha', 0.1, 'linewidth', 0.5);
    H.shapeO(iO) = patch(p(1,iO)+Ds*sin(linspace(0, 2*pi, N)), p(2,iO)+Ds*cos(linspace(0, 2*pi, N)), 10*ones(1, N),  'w', 'facealpha', 0.1, 'linewidth', 0.5);
    H.f1_obs{iO} = hatchfill(H.realO(iO), 'single', 45, 10, [1.0, 1.0, 1.0]);
    H.f1_obs{iO}.HandleVisibility = 'off';
end


H.posO = plot(p(1, na+1:end), p(2, na+1:end), 'k.', 'markersize', 5, 'linestyle', 'none');
text(p(1,2), p(2,2)-0.05, '$o_1$', 'fontsize', 20, 'HorizontalAlignment', 'center', 'Interpreter','latex');
text(p(1,3), p(2,3)-0.05, '$o_2$', 'fontsize', 20, 'HorizontalAlignment', 'center', 'Interpreter','latex');

axis equal;
xlabel('$[p]_{1}$', 'FontSize', 17,'interpreter','latex');
ylabel('$[p]_{2}$', 'FontSize', 17,'interpreter','latex');
caxis manual; caxis([0 upper]);
c = colorbar;
c.Label.String = '$|v^*|$';
c.Label.Interpreter = 'latex';
set(gca, 'fontsize', 16);
colormap(blue_map_edited);
axis equal
axis([-0.6,0.6,-0.6,0.6]*1.21);
set(gca, 'xtick', get(gca, 'ytick'), 'box', 'on');
hold on
end


function ap = gen_a(Ap, A, a, k_tau)
N  = size(Ap, 1);
ap = zeros(N, 1);
cos_tau = cos(2*pi/N);
A_unit     = unit_vector(A);
polyA_unit = unit_vector(Ap);
cos_theta  = polyA_unit*A_unit';

chi = cos_theta.*a' - limit(1*(cos_tau-cos_theta), 0, 1).*(k_tau+cos_theta.*a');
ap  = max(chi, [], 2);
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

A = zeros(nao-1, 2);
b = zeros(nao-1, 1);
count = 0;
for k = 1:nao
    if(k == 1)
        continue;
    end
    count = count + 1;
    p_delta = pm - p(:, k);
    dist    = norm(p_delta);
    h = dist - Ds;
    V = 1/(h+Ds);
    A(count, :) = -p_delta'/norm(p_delta);
    b(count, :) = -alpha_v*(1/dist-1/Ds);
end

f_aim = [v_aim];
ap_i  = gen_a(Ap_i, A, -b, 2.5/delta_bar);
M     = [Ap_i, ap_i];



[v_set_detO, fO, exitflagO, outputO, lambdaO] =  quadprog(...
    diag([1, 1]), -v_aim,...
    A, b, [], [],  ...
    [-inf; -inf], [inf; inf], zeros(2, 1), opt);


v_set_daO = norm(v_set_detO(1:2));
v_set_da  = v_set_daO;


data  =[];
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

if(norm(res - v_set_det) > 1e-2)
    v_set_da  = 0;
    warning('error res, %f\n', norm(res - v_set_det));
end

Ax = A'/(A*A');
Bx = eye(size(a, 2));
Cx = (A*A')\A*v_p;
Kmn = eye(size(a, 1));




Lambda0 = A*A';
Lambda1 = (1+a'/Lambda0*a);
Lambda2 = A'/Lambda0;
Lambda3 = (a*a');
v_set_dan = ...
    +( 2*(Lambda2*Lambda3*Lambda2'*v_p)*(a'/Lambda0) )  / Lambda1^2 ...
    +( k_delta*Lambda2*(Lambda1 - 2*(a*a') /Lambda0) ) /Lambda1^2 ...
    -( (kron(v_p'*Lambda2*a, Lambda2) + kron(v_p'*Lambda2, Lambda2*a)*Kmn)*Lambda1  )  / Lambda1^2;

v_set_da = norm(v_set_dan);
end

function Kmn = commutation_matrix(A)
[m, n] = size(A);
I = reshape(1:m*n, [m, n]);
I = I';
I = I(:);
Kmn = eye(m*n);
Kmn = Kmn(I,:);
end

function A = getPolyAb(N, Radius, theta_bias)
theta    = linspace(theta_bias, theta_bias+2*pi, N+1)';
A = [cos(theta(1:end-1)), sin(theta(1:end-1))];
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
function M = unit_vector(M)
for idx = 1:size(M,1)
    M(idx, :) = M(idx, :)/ norm(M(idx, :));
end
end
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
