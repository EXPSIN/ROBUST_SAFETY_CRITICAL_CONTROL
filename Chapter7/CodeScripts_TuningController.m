close all; clear all; clc;
addpath('Auxiliary/');
syms m;
assume(m, {'positive', 'integer'});
m = 4;

syms g_bar [1, m];
syms g_ubar [1, m];
syms delta_bar [1, m];

for i = 1:m
    eval(sprintf('g_bar%d=1;', i));
    eval(sprintf('g_ubar%d=1;', i));
    eval(sprintf('delta_bar%d=0;', i));
end

syms f_bar_o(s) [1, m];
for i =2:m
    eval(sprintf('f_bar_o%d(s)=0;', i));
end


syms gamma_1_2(s);
k_gamma_12   = 4.0;
gamma_1_2(s) = k_gamma_12 * s;

syms gamma_x2r_V(s);
syms k_1;
assume(k_1, {'positive', 'real'});
gamma_x2r_V(s) = 1/4 * s;
k_1 = 3.49;

k_gamma_i  = 0.249;
k_gamma_i1 = k_gamma_i;
k_gamma_ij = k_gamma_i;
syms theta;
assume(theta, {'positive', 'real'});
sigma(s) = 1e-3*s;

for i = 2:m
    fprintf('Calculating the %d-th gains.\n', i);


    eval(sprintf('syms gamma_%d_x1(s);', i));
    eval(sprintf('syms gamma_%d_x1_inv(s);', i));
    eval(sprintf('syms gamma_%d_rho0(s);', i));
    eval(sprintf('syms gamma_%d_rho0_inv(s);', i));

    for j = 1:i-1
        eval(sprintf('syms gamma_%d_%d(s);', i, j));
        eval(sprintf('syms gamma_%d_%d_inv(s);', i, j));
    end

    eval(sprintf('syms gamma_%d_%d(s);', i, i+1));
    eval(sprintf('syms gamma_%d_%d_inv(s);', i, i+1));

    eval(sprintf('gamma_%d_x1(s) = %f*s;', i, 1/0.1));
    eval(sprintf('gamma_%d_x1_inv(s) = %f*s;', i, 0.1));
    eval(sprintf('gamma_%d_rho0(s) = %f*s;', i, 1/0.1));
    eval(sprintf('gamma_%d_rho0_inv(s) = %f*s;', i, 0.1));

    for j = 1:i-1
        eval(sprintf('gamma_%d_%d(s) = %f*s;', i, j, k_gamma_ij));
        eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, j, 1/k_gamma_ij));
    end

    eval(sprintf('gamma_%d_%d(s) = %f*s;', i, i+1, k_gamma_ij));
    eval(sprintf('gamma_%d_%d_inv(s) = %f*s;', i, i+1, 1/k_gamma_ij));
    eval(sprintf('gamma_%d_2(s) = %f*s;', i, 1/1.001));
    eval(sprintf('gamma_%d_2_inv(s) = %f*s;', i, 1.001));


    variables_str{i} = 'rho_bar_0, c_x1, V_breve';
    variables_inverse_str{i} = sprintf('gamma_%d_rho0_inv(s), gamma_%d_x1_inv(s), gamma_%d_1_inv(s)', i,i,i);
    for j = 2:i+1
        variables_str{i} = [variables_str{i}, sprintf(', x_tilde_norm_%d', j)];
        if(j == i)
            variables_inverse_str{i} = [variables_inverse_str{i}, sprintf(', s')];
        else
            variables_inverse_str{i} = [variables_inverse_str{i}, sprintf(', gamma_%d_%d_inv(s)', i, j)];
        end

    end

    eval(sprintf('syms alpha_%d(%s)', i, variables_str{i}));
    eval(sprintf('syms alpha_f_%d(%s)', i, variables_str{i}));

    f_bar_o_tmp = gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2;
    for j = 3:i
        eval(sprintf(['f_bar_o_tmp = f_bar_o_tmp + ' ...
            '(g_ubar%d/(g_ubar%d-delta_bar%d) * kappa_%d(x_tilde_norm_%d) + x_tilde_norm_%d ); '], ...
            j,j,j, j-1, j-1, j));
    end

    eval(sprintf('alpha_f_%d(%s) = c_x1 + f_bar_o%d(f_bar_o_tmp);', i, variables_str{i}, i));

    eval(sprintf(['alpha_%d(%s) = (g_bar%d+delta_bar%d)*x_tilde_norm_%d + alpha_f_%d(%s) ' ...
        '+ k_1*k_prop(2, %d)*(c_x1 + (g_bar1 + delta_bar1)*(gamma_x2r_V(V_breve) + rho_bar_0 + x_tilde_norm_2) );'], ...
        i, variables_str{i}, i,i, i+1, i, variables_str{i}, i-1));

    for q = 2:i-1
        eval(sprintf(['alpha_%d(%s) = alpha_%d(%s) + k_prop(%d, %d)*(alpha_f_%d(%s) + ' ...
            '(g_bar%d+delta_bar%d)*(g_ubar%d*kappa_%d(x_tilde_norm_%d)/(g_ubar%d-delta_bar%d) + x_tilde_norm_%d ));'], ...
            i, variables_str{i}, i, variables_str{i}, q, i-1, q, variables_str{q}, q, q,q, q,q, q,q, q+1));
    end

    eval(sprintf('kappa_%d(s) = sigma(s) + 1/g_ubar%d*alpha_%d(%s);', i, i, i, variables_inverse_str{i}));

    eval(sprintf('k_%d(s) = g_ubar%d/(g_ubar%d - delta_bar%d) * (g_ubar%d/g_ubar%d*kappa_%d(s)/s + diff(kappa_%d,s));', i,i,i,i,i, i, i, i));
end

for i = 2:m
    fprintf('kappa_%d(s) = ', i);
    disp(vpa(expand(simplify(eval( sprintf('kappa_%d', i)) )), 6))
end



function res = k_prop(q, i)
res = 1;
for j = q:i-1
    eval(sprintf('x_tilde_norm_%d=evalin(''base'', ''x_tilde_norm_%d'');', j, j));
    eval(sprintf('k_%d(x_tilde_norm_%d)=evalin(''base'', ''k_%d(x_tilde_norm_%d)'');', j, j, j, j));
    eval(sprintf('res = res * k_%d(x_tilde_norm_%d);', j, j));
end
end
