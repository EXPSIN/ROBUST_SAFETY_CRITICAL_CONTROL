close all; clear all; clc;
addpath('Auxiliary/');
range      = [-3.0, 3.0, -3.0, 3.0];
zoom_range = [-0.2, 0.2, -0.1, 0.1];
cnt        = 350+1;
grid_size  = abs([range(2) - range(1), range(4) - range(3)]) / cnt;
[X, Y]     = meshgrid(linspace(range(1), range(2), cnt), linspace(range(3), range(4), cnt));
cnt_x2     = 20;
x2_bar     = 1.0;
x2_list    = x2_bar*[cos(linspace(0,2*pi,cnt_x2)); sin(linspace(0,2*pi,cnt_x2))];

p_o(:, 1) = [-1;   1.0];
p_o(:, 2) = [ 1;  -1.0];
D         = 1.0;
D_s       = sqrt(2)/1.01;

p_o_circ    = D*[cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
p_o_circ_Ds = D_s*[cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];

[norm_original, norm_composing, norm_composing2, norm_nonsmooth, norm_reshaped] = norm_of_solution_compare(X, Y, p_o, 1.0, D, D_s, [1; 1], [1;2]);

p = [-1.0; 0];
for i_o = 1:size(p_o, 2)
    h(i_o, 1) = norm(p-p_o(:, i_o))-D_s;
    A(i_o, :) = -(p-p_o(:, i_o))'/norm(p-p_o(:, i_o));
    b(i_o, 1) = -(1/norm(p-p_o(:, i_o)) - 1/D_s);
end



figure(1); clf;
set(gcf, 'Position', [10, 10, 450, 400], 'color', 'w');
clrs = [[0.0, 1.0, 1.0]; [0.0, 1.0, 1.0]];
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal');
H_original = plotColoredGrid_compare(X, Y, norm_original, clrs(1, :), grid_size, 0.7, true);
for i_o = 1:size(p_o, 2)
    H_obs(i_o)  = patch(p_o(1, i_o)+p_o_circ_Ds(1, :), p_o(2, i_o)+p_o_circ_Ds(2, :), 'c', 'facealpha', 0.0, 'handlevisibility', 'off');
    H_obs1(i_o) = patch(p_o(1, i_o)+p_o_circ(1, :), p_o(2, i_o)+p_o_circ(2, :), 'c', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs1(i_o).ZData         = 5.1*ones(size(H_obs1(i_o).XData));
    H_obs1(i_o).FaceColor = 'w';
    H_obs1(i_o).FaceAlpha = 1;
    H_obs1_hatch(i_o) = hatchfill(H_obs1(i_o),'single',45,10, [1,1,1]);
    H_obs(i_o).ZData         = 5.0*ones(size(H_obs(i_o).XData));
    H_obs1_hatch(i_o).ZData  = 5.1*ones(size(H_obs1_hatch(i_o).XData));
end

plot3(p_o(1, :), p_o(2, :), 6*ones(size(p_o(2, :))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text(p_o(1, 1), p_o(2, 1), 6, '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text(p_o(1, 2), p_o(2, 2), 6, '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[p]_1$', 'Interpreter','latex');
ylabel('$[p]_2$', 'Interpreter','latex');
axis(range)







figure(2); clf;
set(gcf, 'Position', [510, 10, 450, 400], 'color', 'w');
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal');
H_reshaped = plotColoredGrid_compare(X, Y, norm_reshaped, clrs(2, :), grid_size, 0.7, true);
for i_o = 1:size(p_o, 2)
    H_obs(i_o)  = patch(p_o(1, i_o)+p_o_circ_Ds(1, :), p_o(2, i_o)+p_o_circ_Ds(2, :), 'c', 'facealpha', 0.0, 'handlevisibility', 'off');
    H_obs2(i_o) = patch(p_o(1, i_o)+p_o_circ(1, :), p_o(2, i_o)+p_o_circ(2, :), 'c', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs2(i_o).ZData         = 5.1*ones(size(H_obs2(i_o).XData));
    H_obs2(i_o).FaceColor = 'w';
    H_obs2(i_o).FaceAlpha = 1;
    H_obs2_hatch(i_o) = hatchfill(H_obs2(i_o),'single',45,10, [1,1,1]);
    H_obs(i_o).ZData         = 5.0*ones(size(H_obs(i_o).XData));
    H_obs2_hatch(i_o).ZData  = 5.1*ones(size(H_obs1_hatch(i_o).XData));
end
plot3(p_o(1, :), p_o(2, :), 6*ones(size(p_o(2, :))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text(p_o(1, 1), p_o(2, 1), 6, '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text(p_o(1, 2), p_o(2, 2), 6, '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[p]_1$', 'Interpreter','latex');
ylabel('$[p]_2$', 'Interpreter','latex');
axis(range)




function h = plotColoredGrid_compare(X, Y, values, clr, grid_size, face_alpha, enable_colorbar)
    numRows = size(values, 1);
    numCols = size(values, 2);

    values(values==-2) = nan;

    h = surf(X, Y, values, 'LineStyle', 'none');
    colorbar;
    axis equal;
    set(gca, 'YDir', 'normal');

    colormap('sky');
end
function h = plotColoredGrid(X, Y, values, clr, grid_size, face_alpha, enable_colorbar)
    numRows = size(values, 1);
    numCols = size(values, 2);

    X = [X ,X(:, end)+grid_size(1)];
    X = [X; X(end, :)] - grid_size(1)/2;
    Y = [Y, Y(:, end)];
    Y = [Y; Y(end, :)+grid_size(2)] - grid_size(2)/2;
    vertices = [X(:), Y(:)];

    clr_3d(1, 1, 1:3) = 1-clr;
    faceColors = ones(numRows, numCols, 3) .* values .* clr_3d;

    faceColors  = reshape(faceColors, [], 3);
    faceColors  = 1-faceColors;

    numFaces    = numCols * numRows;
    faces       = zeros(numFaces, 4);
    [row, col]  = ind2sub([numRows, numCols], 1:numFaces);
    faces(:, 1) = sub2ind([numRows+1, numCols+1],   row,   col);
    faces(:, 2) = sub2ind([numRows+1, numCols+1], row+1,   col);
    faces(:, 3) = sub2ind([numRows+1, numCols+1], row+1, col+1);
    faces(:, 4) = sub2ind([numRows+1, numCols+1],   row, col+1);

    indices = ~all(faceColors == -clr, 2);
    line_clr = faceColors(find(indices == 1, 1), :);
    if(isempty(line_clr))
        line_clr = [1,1,1];
    end

    h = patch('Vertices', vertices, ...
        'Faces', faces(indices, :), ...
        'FaceVertexCData', faceColors(indices, :), ...
        'FaceColor', 'flat', ...
        ... 'EdgeColor', line_clr, ...
        'linestyle', 'none', ...
        'facealpha', face_alpha);
    if(enable_colorbar)
        colormap(1-linspace(0,1,100)'.*(1-clr));
        colorbar;
    end

end

function [norm_original, norm_composing, norm_composing2, norm_nonsmooth, norm_reshaped] = norm_of_solution_compare(X, Y, o, k_phi, D, D_s, v0, k)

norm_original   = QP_with_original_feasible_set(X, Y, o, k_phi, D, D_s, v0, k);
norm_composing  = QP_controller_composing(X, Y, o, k_phi, D, D_s, v0);
norm_composing2 = QP_controller_composing2(X, Y, o, k_phi, D, D_s, v0);
norm_nonsmooth  = QP_controller_nonsmooth(X, Y, o, k_phi, D, D_s, v0);
norm_reshaped   = QP_with_reshaped_feasible_set(X, Y, o, k_phi, D, D_s, v0, k);

end

function C = QP_with_original_feasible_set(X, Y, o, k_phi, D, D_s, v0, k)
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
A      = zeros(n_o, 2);
b      = zeros(n_o, 1);
C      = ones(m, n, 1);

for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-D_s;
            A(i_o, :) = -(p-o(:, i_o))'/norm(p-o(:, i_o));
            b(i_o, 1) = -alpha_h(1/norm(p-o(:, i_o)) - 1/D_s, k);
        end


        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A, b, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end


        if(min(h)+D_s-D < 0)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A*[sol(1); sol(2)]-b > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end

function C = QP_with_reshaped_feasible_set(X, Y, o, k_phi, D, D_s, v0, k)
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
A_c    = zeros(n_o, 2);
b_c    = zeros(n_o, 1);
c_c    = zeros(n_o, 1);
C      = ones(m, n, 1);
A_L    = get_positive_basis(5);
for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-D_s;
            A_c(i_o, :) = -(p-o(:, i_o))'/norm(p-o(:, i_o));
            b_c(i_o, 1) = -alpha_h((1/norm(p-o(:, i_o)) - 1/D_s), k);
        end

        [b_L, u_L]= reshape_b_L(A_L, A_c, c_c, b_c, k_phi);

        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A_L, b_L, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end

        if(min(h)+D_s-D < 0)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A_L*[sol(1); sol(2)]-b_L > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end

function y = alpha_h(s, k)
if(s >= 0)
    y = k(1)*s;
else
    y = k(2)*s;
end
end



function [b_L, nu_s, c_A_c]= reshape_b_L(A_L, A_c, c_c, b_c, k_psi)
n_l = size(A_L, 1);
c_A  = cos(2*pi/n_l);
nu_s = A_c' * min(b_c, 0);
psi  = (b_c-A_c*nu_s)' .* max(A_L * A_c', c_A) + k_psi*max(c_A - A_L*A_c', 0);
b_L  = A_L*nu_s + min(psi, [], 2);
end

function A = get_positive_basis(N)
    theta = linspace(0, 2*pi, N+1)';
    A     = [cos(theta(1:end-1)), sin(theta(1:end-1))];
end


function C = QP_controller_composing(X, Y, o, k_phi, D, D_s, v0)
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
dh_dx  = zeros(n_o, 2);
lambda = zeros(n_o, 1);
C      = ones(m, n, 1);

kappa = 0.6;
return;

for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-D_s;
            dh_dx(i_o, :) = (p-o(:, i_o))/norm(p-o(:, i_o));
        end

        h_composing = 0;
        for i_o = 1:n_o
            h_composing = h_composing + exp(-kappa.*h(i_o));
        end
        h_composing = - 1 ./ kappa .* log(h_composing);
        dh_composing_dx = zeros(1, 2);
        for i_o = 1:n_o
            lambda(i_o) = exp(-kappa*(h(i_o)-h_composing));
            dh_composing_dx  = dh_composing_dx + lambda(i_o)*dh_dx(i_o, :);
        end

        A = -dh_composing_dx;
        b = h_composing;

        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A, b, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end


        if(min(h)+D_s-D < 0)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A*[sol(1); sol(2)]-b > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end

function C = QP_controller_composing2(X, Y, o, k_phi, D, D_s, v0)
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
dh_dx  = zeros(n_o, 2);
lambda = zeros(n_o, 1);
C      = ones(m, n, 1);

kappa = 6.0;
return;
for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-D_s;
            dh_dx(i_o, :) = (p-o(:, i_o))/norm(p-o(:, i_o));
        end

        h_composing = 0;
        for i_o = 1:n_o
            h_composing = h_composing + exp(-kappa.*h(i_o));
        end
        h_composing = - 1 ./ kappa .* log(h_composing);
        dh_composing_dx = zeros(1,2);
        for i_o = 1:n_o
            lambda(i_o) = exp(-kappa*(h(i_o)-h_composing));
            dh_composing_dx  = dh_composing_dx + lambda(i_o)*dh_dx(i_o, :);
        end

        A = -dh_composing_dx;
        b = h_composing;

        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A, b, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end


        if(min(h)+D_s-D < 0)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A*[sol(1); sol(2)]-b > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end


function C = QP_controller_nonsmooth(X, Y, o, k_phi, D, D_s, v0)
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
dh_dx  = zeros(n_o, 2);
lambda = zeros(n_o, 1);
C      = ones(m, n, 1);

kappa = 6.0;
return;

for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-D_s;
            dh_dx(i_o, :) = (p-o(:, i_o))/norm(p-o(:, i_o));
        end

        h_min = min(h);
        mask  = h==h_min;

        A = -dh_dx(mask, :);
        b = h(mask, 1);

        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A, b, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end


        if(min(h)+D_s-D < 0)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A*[sol(1); sol(2)]-b > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end

