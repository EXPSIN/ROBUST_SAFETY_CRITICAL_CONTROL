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

p_o_circ    = D   * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];
p_o_circ_Ds = D_s * [cos(linspace(0,2*pi,100)); sin(linspace(0,2*pi,100))];


p = [-1.0; 0];

for i_o = 1:size(p_o, 2)
    h(i_o, 1) = norm(p-p_o(:, i_o))-D_s;
    A(i_o, :) = -(p-p_o(:, i_o))'/norm(p-p_o(:, i_o));
    b(i_o, 1) = -(1/norm(p-p_o(:, i_o)) - 1/D_s);
end

illustrate_reshaping_step1(A, b, 0, 101);
illustrate_reshaping_step2(A, b, 0, 102);
illustrate_reshaping_step3(A, b, 0, 103);
illustrate_reshaping_step4(A, b, 0, 104);





figure(1); clf;
set(gcf, 'Position', [10, 10, 450, 400], 'color', 'w');
clrs = [[0.0, 1.0, 1.0]; [0.0, 1.0, 1.0]];
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal');
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

return;

plot3(p_o(1, :), p_o(2, :), 6*ones(size(p_o(2, :))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text(p_o(1, 1), p_o(2, 1), 6, '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text(p_o(1, 2), p_o(2, 2), 6, '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[p]_1$', 'Interpreter','latex');
ylabel('$[p]_2$', 'Interpreter','latex');
axis(range)







figure(2); clf;
set(gcf, 'Position', [10, 10, 450, 400], 'color', 'w');
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

function illustrate_reshaping_step1(A, b, c_delta, fig_index)

theta   = 0;

axis_range = [-1.5, 1.0, -1.7, 0.8]*1.0;

figure(fig_index); clf;
set(gcf, 'color', 'w', 'position',[10,10,600,600]);
hold on;  axis equal; axis(axis_range*0.99);
xticks(-4:1:4);
yticks(-4:1:4);
set(gca, 'XAxisLocation','origin', 'YAxisLocation', 'origin', 'fontsize', 16, 'position', [0,0,1,1]);

H = half_plane([], A, b, [0; 0], axis_range, 'c', '$\mathcal{V}$');


n_l   = [7, 14, 27];
k_phi = [1,  4,  8];
clr   = {'b', 'b', 'g'};
clr_v = {[23, 41, 77]/255, [46, 73,106]'/255, [53,122,145]'/255};
for i = 1
    A_L          = get_positive_basis(n_l(i));
    [b_L, u_L]   = reshape_b_L(A_L, A, c_delta, b, k_phi(i));

    plot(u_L(1), u_L(2), 'k.', 'MarkerSize', 20);


end

legend('Interpreter','latex', 'Location','northwest');
xlabel('$[v^*]_1$', 'Interpreter','latex');
ylabel('$[v^*]_2$', 'Interpreter','latex');
legend off;
end




function illustrate_reshaping_step2(A, b, c_delta, fig_index)

theta   = 0;

axis_range = [-1.5, 1.0, -1.7, 0.8]*1.0;


figure(fig_index); clf;
set(gcf, 'color', 'w', 'position',[100,10,600,600]);
hold on;  axis equal; axis(axis_range*0.99);
xticks(-4:1:4);
yticks(-4:1:4);
set(gca, 'XAxisLocation','origin', 'YAxisLocation', 'origin', 'fontsize', 16, 'position', [0,0,1,1]);

H = half_plane([], A, b, [0; 0], axis_range, 'c', '$\mathcal{V}$');


n_l   = [7, 14, 27];
k_phi = [1,  4,  8];
clr   = {'b', 'b', 'g'};
clr_v = {[23, 41, 77]/255, [46, 73,106]'/255, [53,122,145]'/255};
for i = 1
    A_L          = get_positive_basis(n_l(i));
    [b_L, u_L]   = reshape_b_L(A_L, A, c_delta, b, k_phi(i));

    plot(u_L(1), u_L(2), 'k.', 'MarkerSize', 20);


    if(i == 1)
        H_basis      = quiver(u_L(1)*ones(size(b_L,1),1), u_L(2)*ones(size(b_L,1),1), A_L(:,1).*1, A_L(:,2).*1, 1.0, 'linewidth', 2, 'color', 'k', 'MaxHeadSize',0.2, 'HandleVisibility', 'off');
    end
end

legend('Interpreter','latex', 'Location','northwest');
xlabel('$[v^*]_1$', 'Interpreter','latex');
ylabel('$[v^*]_2$', 'Interpreter','latex');
legend off;
end






function illustrate_reshaping_step3(A, b, c_delta, fig_index)

theta   = 0;

axis_range = [-1.5, 1.0, -1.7, 0.8]*1.0;

cnt_constraints = length(b);

A_all = A;
b_all = b;
c_all = c_delta;

for i = 1:cnt_constraints
    figure(fig_index+i*10); clf;
    A = A_all(i, :);
    b = b_all(i, :);
    set(gcf, 'color', 'w', 'position',[200,10+i*100,600,600]);
    hold on;  axis equal; axis(axis_range*0.99);
    xticks(-4:1:4);
    yticks(-4:1:4);
    set(gca, 'XAxisLocation','origin', 'YAxisLocation', 'origin', 'fontsize', 16, 'position', [0,0,1,1]);

    H = half_plane([], A, b, [0; 0], axis_range, 'c', '$\mathcal{V}$');


    n_l   = [7, 7, 7];
    k_phi = [1,  1,  1];
    clr   = {'g', 'g', 'g'};
    clr_v = {[23, 41, 77]/255, [46, 73,106]'/255, [53,122,145]'/255};
    A_L          = get_positive_basis(n_l(i));
    [~, u_L]   = reshape_b_L(A_L, A_all, c_delta, b_all, k_phi(i));
    [b_L, ~]   = reshape_b_L_u_L(A_L, A, c_delta, b, k_phi(i), u_L);

    plot(u_L(1), u_L(2), 'k.', 'MarkerSize', 20);

    H_polyhedron = half_plane([], A_L, b_L, [0; 0], axis_range, clr{i}, sprintf('$\\mathcal{V}_L$, $c_A=\\cos(2\\pi/%2d)$', n_l(i)) );
    H_polyhedron.handle_space.FaceAlpha = 0.6;
    H_polyhedron.handle_space.FaceColor = 'g';

    H_basis      = quiver(u_L(1)*ones(size(b_L,1),1), u_L(2)*ones(size(b_L,1),1), A_L(:,1).*1, A_L(:,2).*1, 1.0, 'linewidth', 2, 'color', 'k', 'MaxHeadSize',0.2, 'HandleVisibility', 'off');

    legend('Interpreter','latex', 'Location','northwest');
    xlabel('$[v^*]_1$', 'Interpreter','latex');
    ylabel('$[v^*]_2$', 'Interpreter','latex');
    legend off;
end
end







function [b_L, nu_s, c_A_c]= reshape_b_L_u_L(A_L, A_c, c_c, b_c, k_psi, nu_s)
n_l  = size(A_L, 1);
c_A  = cos(2*pi/n_l);
psi  = (b_c-A_c*nu_s)' .* max(A_L * A_c', c_A) + k_psi*max(c_A - A_L*A_c', 0);
b_L  = A_L*nu_s + min(psi, [], 2);
end



function illustrate_reshaping_step4(A, b, c_delta, fig_index)

theta   = 0;

axis_range = [-1.5, 1.0, -1.7, 0.8]*1.0;


figure(fig_index); clf;
set(gcf, 'color', 'w', 'position',[300,10,600,600]);
hold on;  axis equal; axis(axis_range*0.99);
xticks(-4:1:4);
yticks(-4:1:4);
set(gca, 'XAxisLocation','origin', 'YAxisLocation', 'origin', 'fontsize', 16, 'position', [0,0,1,1]);

H = half_plane([], A, b, [0; 0], axis_range, 'c', '$\mathcal{V}$');


n_l   = [7, 14, 27];
k_phi = [1,  4,  8];
clr   = {'g', 'g', 'g'};
clr_v = {[23, 41, 77]/255, [46, 73,106]'/255, [53,122,145]'/255};
for i = 1
    A_L          = get_positive_basis(n_l(i));
    [b_L, u_L]   = reshape_b_L(A_L, A, c_delta, b, k_phi(i));

    plot(u_L(1), u_L(2), 'k.', 'MarkerSize', 20);

    H_polyhedron = half_plane([], A_L, b_L, [0; 0], axis_range, clr{i}, sprintf('$\\mathcal{V}_L$, $c_A=\\cos(2\\pi/%2d)$', n_l(i)) );
    H_polyhedron.handle_space.FaceAlpha = 0.6-i*0.09;
    H_polyhedron.handle_space.FaceColor = 'g';

    if(i == 1)
        H_basis      = quiver(u_L(1)*ones(size(b_L,1),1), u_L(2)*ones(size(b_L,1),1), A_L(:,1).*1, A_L(:,2).*1, 1.0, 'linewidth', 2, 'color', 'k', 'MaxHeadSize',0.2, 'HandleVisibility', 'off');
    end
end

legend('Interpreter','latex', 'Location','northwest');
xlabel('$[v^*]_1$', 'Interpreter','latex');
ylabel('$[v^*]_2$', 'Interpreter','latex');
legend off;
end



function A = get_positive_basis(N)
theta = linspace(0, 2*pi, N+1)';
A     = [cos(theta(1:end-1)), sin(theta(1:end-1))];
end
function [b_L, nu_s, c_A_c]= reshape_b_L(A_L, A_c, c_c, b_c, k_psi)
n_l  = size(A_L, 1);
c_A  = cos(2*pi/n_l);
nu_s = A_c' * min(b_c, 0);
psi  = (b_c-A_c*nu_s)' .* max(A_L * A_c', c_A) + k_psi*max(c_A - A_L*A_c', 0);
b_L  = A_L*nu_s + min(psi, [], 2);
end
