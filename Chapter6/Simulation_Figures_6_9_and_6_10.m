clear all; clc; close all;

[x, u, cfg] = config();
A = [1,0];
B = 0;
H = graphic([], x, cfg);
tp=0;
theta_record_sim=zeros(2,cfg.cnt);
theta_d_record_sim=zeros(2,cfg.cnt);
u_record_sim=zeros(2,cfg.cnt);
vstar_record_sim=zeros(2,cfg.cnt);
for k = 1:cfg.cnt
    cfg.t = k * cfg.step;

    if(cfg.is_experiment == true)
        x = get_state(cfg);
        u  = controller(x, cfg);
        set_control_input(u, cfg);
    else
        [t, y] = ode15s(@(t, x)close_loop_system(x, cfg), [0, cfg.step], x);
        x = y(end, :)';
        theta_record_sim(:,k)=x(1:2,:);
        theta_d_record_sim(:,k)=x(3:4,:);
         [u vstar B A]  = controller(x, cfg);
        u_record_sim(:,k)=u;
        vstar_record_sim(:,k)=vstar;
        Bw(:,k)=B;
    end

    H = graphic(H, x, cfg);
end





function [u vstar B A]  = controller(x, cfg)
theta   = x(1:2, 1);
theta_d = x(3:4, 1);
k = cfg.t / cfg.step

obs_number_o=size(cfg.map_obs_theta);
obs_number=obs_number_o(2);
barv=cfg.c0*(cfg.barv-cfg.dr);


K=vecnorm(cfg.map_obs_theta-theta);
dis=K';

W=(cfg.map_obs_theta-theta)./vecnorm(cfg.map_obs_theta-theta);
vec=W';


reshape_o_2=phi_plus_dis(cfg,vec,dis);
reshape=alphac(cfg, reshape_o_2);




A=cfg.ctrl_basis;B=reshape;


if k<=cfg.k
   vc=(cfg.ctrl_middle_aim-theta)*min(15,30*norm(cfg.ctrl_middle_aim-theta))/(norm(cfg.ctrl_middle_aim-theta)*57.3);
else
    vc=(cfg.ctrl_aim-theta)*min(15,30*norm(cfg.ctrl_aim-theta))/(norm(cfg.ctrl_aim-theta)*57.3);
end
H=diag([2,2]);
f=-2*vc';

opt=optimoptions('quadprog', 'Display','none');
vstar=quadprog(H,f,A,B,[],[],[],[],zeros(1,2),opt);

[M, C, N] = MCN(theta, theta_d, cfg);

u = -800*(theta_d-vstar) + N + C*vstar;
end


function [x, u, cfg] = config()

x = zeros(4, 1);
x(1:2,:)=[pi/2-0.05;-1];
u = zeros(2, 1);


cfg.arm_len   = [0.25; 0.40];
cfg.arm_width = 0.07;
cfg.arm_m     = [2.00; 3.00];
cfg.arm_p0    = [0; 0];


image              = imread('Auxiliary/map_1.bmp');
try
    cfg.map_data = rgb2gray(image) < 125;
    fprintf('Image is treated as a 256-color grayscale map with threshold 125.\n');
catch
    cfg.map_data = ~image;
    fprintf('Image is treated as a binary map.\n');
end
cfg.map_p0         = [-0.8; 0.0];
cfg.map_resolution = 1200;
cfg.map_angle_interval = 2.5;


cfg.map_obs_pos        = get_obs_form_map(cfg.map_data, cfg.map_p0, cfg.map_resolution);
try
    load('Auxiliary/map_obs_theta_test_final.mat', 'map_obs_theta')
catch
    fprintf('map_obs_theta_test_final.mat not found; regenerating.\n');
    map_obs_theta  = configuration_space(cfg.arm_len, cfg.arm_p0, ...
        cfg.map_obs_pos, cfg.arm_width/2.0, cfg.map_angle_interval);
    save('Auxiliary/map_obs_theta_test_final.mat', 'map_obs_theta');
end
cfg.map_obs_theta = map_obs_theta;


cfg.step = 20e-3;
cfg.cnt  = 22*round(1/cfg.step);
cfg.t    = 0;

cfg.ctrl_basis_cnt = 75;
cfg.ctrl_basis     = positive_basis_2D(cfg.ctrl_basis_cnt);
cfg.ctrl_c_0       = cosd(360/cfg.ctrl_basis_cnt);

cfg.ctrl_d_s = deg2rad(4*sqrt(2));
cfg.ctrl_D_s = deg2rad(4*sqrt(2))+deg2rad(0.00076);


cfg.ctrl_aim =[pi/2+0.15; 0];
cfg.ctrl_middle_aim = [2.6; -1.6];

cfg.c0=cos(deg2rad(4.8));
cfg.dr=deg2rad(0.6);
cfg.deltah=1.3537e-5;
cfg.ds=deg2rad(10);
cfg.barv=deg2rad(18);
cfg.k=450;
cfg.db=deg2rad(1.04);
cfg.is_experiment = false;
if(cfg.is_experiment == true)
    cfg.ros_ip ='192.168.50.249';
    rosinit(cfg.ros_ip);
    [cfg.ros_ctrl_pub, cfg.ros_ctrl_msg] = rospublisher('/joint_states','sensor_msgs/JointState');
    cfg.ros_ctrl_msg.Name{1} = 'joint2_to_joint1';
    cfg.ros_ctrl_msg.Name{2} = 'joint3_to_joint2';
    cfg.ros_ctrl_msg.Name{3} = 'joint4_to_joint3';
    cfg.ros_ctrl_msg.Name{4} = 'joint5_to_joint4';
    cfg.ros_ctrl_msg.Name{5} = 'joint6_to_joint5';
    cfg.ros_ctrl_msg.Name{6} = 'joint6output_to_joint6';
end
end



function set_control_input(u, cfg)

u_arm = -u;

cfg.ros_ctrl_msg.Header.Stamp = rostime("now");

cfg.ros_ctrl_msg.Position = [-180, u_arm(1), u_arm(2), -90, 0, 0];

send(cfg.ros_ctrl_pub, cfg.ros_ctrl_msg);
end


function x = get_state(cfg)
x = zeros(4, 1);
end

function o_theta = configuration_space(l, p0, obs, r, angle_interval)
test_theta_1 = deg2rad(   0:angle_interval:180);
test_theta_2 = deg2rad(-180:angle_interval:180);
[theta1, theta2] = meshgrid(test_theta_1, test_theta_2);
res = false(size(theta1));

O = zeros(2, 3);
O(:, 1) = p0;

cnt_link_points = 15;
O_list_x = zeros(2*cnt_link_points, 1);
O_list_y = zeros(2*cnt_link_points, 1);

for k=1:size(theta1, 1)
    fprintf('Configuration-space generation progress: %f\n', k / size(theta1, 1));
    for m=1:size(theta1, 2)
        O(:, 2)=O(:, 1)+l(1)*[cos(theta1(k,m)); sin(theta1(k,m))];
        O_list_x(1:cnt_link_points, 1) = linspace(O(1,1), O(1,2), cnt_link_points);
        O_list_y(1:cnt_link_points, 1) = linspace(O(2,1), O(2,2), cnt_link_points);
        O(:, 3)=O(:, 2)+l(2)*[cos(theta1(k,m)+theta2(k,m)); sin(theta1(k,m)+theta2(k,m));];
        O_list_x(cnt_link_points+(1:cnt_link_points), 1) = linspace(O(1,2), O(1,3), cnt_link_points);
        O_list_y(cnt_link_points+(1:cnt_link_points), 1) = linspace(O(2,2), O(2,3), cnt_link_points);

        if(min(O(2, :)) >= -0.05)
            min_dist = min(sqrt((O_list_x - obs(1, :)).^2 + (O_list_y - obs(2, :)).^2), [], 'all');
            if(min_dist < r)
                res(k,m) = 1;
            end
        else
            res(k,m) = 1;
        end
    end
end
k=1;
size(res,1);
size(res,2);
3:1:size(res,1)-2
3:1:size(res,2)-2
ac1=3;
ac2=3;
while ac1<=size(res,1)-2
    while ac2<=size(res,2)-2
        if res(ac1,ac2)==1
            if res(ac1,ac2-1)==0
                if res(ac1-1,ac2-2)+res(ac1+1,ac2)==2&&res(ac1+1,ac2-2)+res(ac1-1,ac2)==2
                    res(ac1,ac2-1)=1;
                end
            end
            if res(ac1,ac2+1)==0
                if res(ac1+1,ac2+2)+res(ac1-1,ac2)==2&&res(ac1-1,ac2+2)+res(ac1+1,ac2)==2
                    res(ac1,ac2+1)=1;
                end
            end
            if res(ac1-1,ac2)==0
                if res(ac1-2,ac2-1)+res(ac1,ac2+1)==2&&res(ac1-2,ac2+1)+res(ac1,ac2-1)==2
                    res(ac1-1,ac2)=1;
                end
            end
            if res(ac1+1,ac2)==0
                if res(ac1+2,ac2+1)+res(ac1,ac2-1)==2&&res(ac1+2,ac2-1)+res(ac1,ac2+1)==2
                    res(ac1+1,ac2)=1;
                end
            end
        end
        ac2=ac2+1;
    end
    ac1=ac1+1;
end
o_theta = [theta1(res)'; theta2(res)'];


end

function dx = close_loop_system(x, cfg)
[u vstar]  = controller(x, cfg);
dx = m_twolinks_manipulator(x, u, cfg);
end


function basis = positive_basis_2D(basis_cnt)
if(mod(basis_cnt, 2) == 0)
    error('For 2D reshaping, basis_cnt must be odd to avoid polar-constraint matrix degeneracy.');
end
theta = linspace(0, 2*pi, basis_cnt+1)';
basis = [cos(theta(1:end-1)), sin(theta(1:end-1))];
end

function H = graphic(H, x, cfg)
k = cfg.t / cfg.step;
if(isempty(H))
    figure(1);
    hold on;
    grid on;
    axis equal;
    axis([-0.8, 0.8, -0.04, 0.9]*0.95);
    set(gcf, 'position', [0, 0, 1920, 1080], 'color', 'w');
    set(gca, 'XAxisLocation','origin', 'YAxisLocation', 'origin', 'fontsize', 14, ...
        'position',[0,0,1,1]);
    H.map = patch_im(cfg.map_data, cfg.map_p0, cfg.map_resolution);
    p_arm = joint_positions_2D(cfg.arm_p0, cfg.arm_len, x(1:2, 1));
    line_width = 0.5*cfg.arm_width/1.6*1920;
    H.arm = plot(p_arm(1, :), p_arm(2, :), 'color', 'b', 'marker', 'none', 'linewidth', line_width, ...
        'markersize', 15, 'markerfacecolor', 'c', 'markeredgecolor', 'c', 'HandleVisibility', 'off', ...
        'Color', [242,165,134,0.5*255]/255);
    H.end_effector_trajectory = animatedline('color', 'b', 'linewidth', 2, 'linestyle', '-', 'marker', 'none', 'Displayname', 'trajectory of end effector', 'HandleVisibility', 'on');


   H.joint_2_trajectory = animatedline('color', 'r', 'linewidth', 2, 'linestyle', '-.', 'marker', 'none', 'Displayname', 'trajectory of joint 2', 'HandleVisibility', 'on');

    legend('AutoUpdate','on');

    figure(2); hold on;
    axis equal;box on;set(gcf,"Color",'w');
  plot(cfg.ctrl_aim(1),cfg.ctrl_aim(2),'r*');
  plot(cfg.ctrl_middle_aim(1),cfg.ctrl_middle_aim(2),'r*');
    scatter(cfg.map_obs_theta(1, :), cfg.map_obs_theta(2, :), 300, [121,204,94]/255,'filled');
   axis([-pi/2, 3*pi/2, -pi, 3.02]);
    H.cspace_trajectory = animatedline('color', 'r', 'linewidth', 1, 'linestyle', '-', 'marker', 'none', 'Displayname', 'trajectory of joint 2', 'HandleVisibility', 'on');
    colormap(gca,"parula");


else

    p_arm = joint_positions_2D(cfg.arm_p0, cfg.arm_len, x(1:2, 1));
    H.arm.XData = p_arm(1, :);
    H.arm.YData = p_arm(2, :);

    addpoints(H.end_effector_trajectory, p_arm(1, end), p_arm(2, end));

    addpoints(H.joint_2_trajectory, p_arm(1, 2), p_arm(2, 2));

    addpoints(H.cspace_trajectory, x(1, 1), x(2, 1));

end

drawnow;
end


function h = patch_im(pic, p_m, resolution)
Z = flipud(double(pic));
Z(Z<=0) = -2;
Z(Z> 0) = -1;

xN = size(Z, 2);
yN = size(Z, 1);

x = (1:xN)/resolution+p_m(1);
y = (1:yN)/resolution+p_m(2);
[X, Y] = meshgrid(x, y);
h = surf('XData',X,'YData',Y,'ZData',Z, 'EdgeColor', 'none', 'displayname', 'obstacle', 'HandleVisibility', 'off');
surf('XData',0,'YData',0,'ZData',0, 'FaceColor', 'black', 'displayname', 'obstacle', 'HandleVisibility', 'on');
hold on;
view(2);
colorres = [ones(1,3);0,0,0];
colormap(colorres);
end

function obs = get_obs_form_map(pic, p_0, resolution)
Z = flipud(double(pic));
[X, Y]= meshgrid(1:size(Z, 2), 1:size(Z, 1));
obs_mask = Z==1;
obs(1, :) = X(obs_mask)/resolution+p_0(1);
obs(2, :) = Y(obs_mask)/resolution+p_0(2);
end

function p_arm = joint_positions_2D(p0, L, theta)
n_theta = size(theta, 1);
p_arm   = zeros(2, n_theta+1);
p_arm(:, 1) = p0;

for i = 2:n_theta+1
    p_arm(:, i) = p_arm(:, i-1) + L(i-1, 1)*[cos( sum(theta(1:i-1)) ); sin( sum(theta(1:i-1)) )];
end
end

function [M, C, N] = MCN(theta, theta_d, cfg)
l = cfg.arm_len;
m = cfg.arm_m;
g = 9.18;

M = zeros(2);
M(1,1) = m(2)*l(2)^2 + m(2)*l(1)*l(2)*cos(theta(2))*2 + (m(1)+m(2))*l(1)^2;
M(1,2) = m(2)*l(2)^2 + m(2)*l(1)*l(2)*cos(theta(2));
M(2,1) = m(2)*l(2)^2 + m(2)*l(1)*l(2)*cos(theta(2));
M(2,2) = m(2)*l(2)^2;

C = zeros(2, 2);
C(1, 2) = -m(2)*l(1)*l(2)*sin(theta(2))*theta_d(2) - 2*m(2)*l(1)*l(2)*sin(theta(1))*theta_d(1);
C(2, 1) =  m(2)*l(1)*l(2)*sin(theta(2))*theta_d(1);

N = zeros(2, 1);
N(1,1) = m(2)*l(2)*g*cos(theta(1)+theta(2)) + (m(1)+m(2))*l(1)*g*cos(theta(1));
N(2,1) = m(2)*l(2)*g*cos(theta(1)+theta(2));
end

function dx = m_twolinks_manipulator(x, u, cfg)
x(1:2,1) = atan2(sin(x(1:2,1)), cos(x(1:2,1)));
theta   = x(1:2, 1);
theta_d = x(3:4, 1);

[M, C, N] = MCN(theta, theta_d, cfg);

dx = [theta_d; M\(-C*theta_d - N + u)];
end




function reshape_o_2=phi_plus_dis(cfg,vec,dis)
md=vec*cfg.ctrl_basis';

 md_x=-0.3026*((md-cfg.c0)/0.9);
    phi_ds=max(0,md_x)-cfg.ds;
reshape_o=phi_ds+dis;
reshape_o_1=min(reshape_o(:,:));
reshape_o_2=reshape_o_1';
end



function reshape=alphac(cfg, reshape_o_2)
reshape_o_3_1=cfg.c0*0.9*reshape_o_2-cfg.dr;
reshape_o_3_2=cfg.c0*( (0.302+cfg.dr)/(cfg.c0*cfg.db/2) )*reshape_o_2-cfg.dr;
reshape_o_3_3=cfg.c0*(0.9*(reshape_o_2-cfg.db/2)+((0.302+cfg.dr)/cfg.c0))-cfg.dr;
reshape_o_3_x=reshape_o_3_1+reshape_o_3_2+reshape_o_3_3-min(min(reshape_o_3_1,reshape_o_3_2),min(reshape_o_3_1,reshape_o_3_3))-max(max(reshape_o_3_1,reshape_o_3_2),max(reshape_o_3_1,reshape_o_3_3));
reshape=min(reshape_o_3_x,cfg.c0*(cfg.barv-cfg.dr));
end
