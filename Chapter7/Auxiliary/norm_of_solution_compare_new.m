function [norm_original, norm_reshaped] = norm_of_solution_compare_new(X, Y, o, k_phi, v_j, v0)
%#codegen
R_s  = -log(v_j);

norm_original   = QP_with_original_feasible_set(X, Y, o, k_phi, R_s, v0);
norm_reshaped   = QP_with_reshaped_feasible_set(X, Y, o, k_phi, R_s, v0);

end

function C = QP_with_original_feasible_set(X, Y, o, k_phi, R_s, v0)
% setting of quadratic programming
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
            h(i_o, 1) = norm(p-o(:, i_o))-R_s;
            A(i_o, :) = -(p-o(:, i_o))'/norm(p-o(:, i_o));
            b(i_o, 1) = h(i_o, 1);
        end


        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A, b, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end


        if(min(h) <= -R_s + 1e-3)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A*[sol(1); sol(2)]-b > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end

function C = QP_with_reshaped_feasible_set(X, Y, o, k_phi, R_s, v0)
% setting of quadratic programming
opt    = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
[m, n] = size(X);
n_o    = size(o, 2);
h      = zeros(n_o, 1);
A_c    = zeros(n_o, 2);
b_c    = zeros(n_o, 1);
c_c    = zeros(n_o, 1);
C      = ones(m, n, 1);
A_L    = get_positive_basis(11);
for i = 1:m
    for j = 1:n
        p = double([X(i,j); Y(i,j)]);
        for i_o = 1:n_o
            h(i_o, 1) = norm(p-o(:, i_o))-R_s;
            A_c(i_o, :) = -(p-o(:, i_o))'/norm(p-o(:, i_o));
            b_c(i_o, 1) = h(i_o, 1);
        end

        [b_L, u_L]= reshape_b_L(A_L, A_c, c_c, b_c, k_phi);

        if(all(vecnorm(p-o, 2, 1) ~= 0))
            [sol, ~, ~, ~, ~] = quadprog(eye(2), -v0, A_L, b_L, [], [], [], [], zeros(2,1), opt);
            sol = [sol(1); sol(2)];
        else
            sol = [];
        end

        if(min(h) <= -R_s + 1e-3)
            C(i, j, :) = -2;
        elseif(isempty(sol) || any(A_L*[sol(1); sol(2)]-b_L > 1e-6))
            C(i, j, :) = -1;
        else
            C(i, j, :) = norm(sol);
        end
    end
end
end


function [b_L, u_L]= reshape_b_L(A_L, A, c, b, k_phi)
    n_l = size(A_L, 1);
    [b_min, idx] = min(b);

    % calculte u_L
    if(b_min <= 0)
        u_L = A(idx, :)'*b_min ./ (1-c);
    else
        u_L = [0; 0];
    end

    c_A    = cos(2*pi/n_l);
    c_A_c  = cos(acos(sqrt(1-c.^2)) + acos(c_A));              % ????
    
    phi_1 = max(A_L*A', c_A_c') .* ((b-A*u_L-c*norm(u_L))./(1+c))';
    phi_2 = k_phi*max(c_A_c' - A_L*A', 0);
    phi   = min(phi_1 + phi_2, [], 2);
    b_L   = A_L*u_L + phi;
end

function A = get_positive_basis(N)
    theta = linspace(0, 2*pi, N+1)';
    A     = [cos(theta(1:end-1)), sin(theta(1:end-1))];
end

function [h1, dh1dx] = h1_fun(x)
h1    = 1 - x;
dh1dx = -1;
end

function [h2, dh2dx] = h2_fun(x)
h2    = 1 + x;
dh2dx = 1;
end