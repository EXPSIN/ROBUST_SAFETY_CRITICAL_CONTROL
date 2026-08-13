clear all
clc
close all

o1=[0.1;0];
o2=[-0.1;0];


dr=0.004;
ds=0.096;

Kv=0.013;
cM=0.8;
dh=3.4975e-5;
c0=cos(2*pi/21);


lc=[cos(0:2*pi/21:40*pi/21);sin(0:2*pi/21:40*pi/21)];
badd=ones(21,1);
badd=badd*(cos(pi/21))*(30-dr);


qx=zeros(2,1001);
qx(2,:)=-0.5:0.001:0.5;
k=1;
opt = optimoptions('quadprog', 'Display','off');
while k<=size(qx,2)
    l1=(o1-qx(:,k))/norm(o1-qx(:,k));
    l2=(o2-qx(:,k))/norm(o2-qx(:,k));
    h1=norm(o1-qx(:,k))-ds;
    h2=norm(o2-qx(:,k))-ds;
    A=[l1';l2';lc'];
    b=[h1-dr;h2-dr;badd];
    H=[2,0;0,2];
    f=[0;-10];
    vstare1=quadprog(H,f,A,b,[], [], [], [], [0;0], opt);
    vstare1_r(:,k)=vstare1;
    k=k+1;
end


qx=zeros(2,1001);
qx(2,:)=-0.5:0.001:0.5;
k=1;
while k<=size(qx,2)
    l1=(o1-qx(:,k))/norm(o1-qx(:,k));
    l2=(o2-qx(:,k))/norm(o2-qx(:,k));
    h1c=norm(o1-qx(:,k))-ds;
    h2c=norm(o2-qx(:,k))-ds;
    m=1;
    while m<=21
        bc(m,1)= min( h1c+max((-Kv*(  l1'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0),h2c+max((-Kv*(  l2'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0));
        bc(m,1)=min(6*bc(m,1),c0*(30-dr));
        m=m+1;
    end
    m=1;
    H=[2,0;0,2];
    f=[0;-10];
    Ac=lc';
    vstarcx=quadprog(H,f,Ac,bc, [], [], [], [], [0;0], opt);
    vstarcx_r(:,k)=vstarcx;
    k=k+1;
end








figure(2)
hold on
box on;
plot(qx(2,:),vstare1_r(2,:), 'DisplayName', '$[v^*_{1}]_2$','LineWidth',2,'Color','k');
plot(qx(2,:),vstarcx_r(2,:), 'DisplayName', '$[v^*_{2}]_2$','LineWidth',2,'Color','g','LineStyle','--');

legend;
legend('Location', 'northwest', 'Interpreter','latex');


xlabel('$[q]_2$ (rad)', 'Interpreter','latex');
ylabel('$[v^*_1]_2,[v^*_2]_2$ (rad/s)', 'Interpreter','latex');

set(gcf, 'position', [50,100,800, 300], 'color', 'w');
set(gca, 'fontsize', 18);
axis([-0.5, 0.5, -1, 5.5])



qs=zeros(2,10001);
qs(2,:)=-0.5:0.0001:0.5;
k=1;
while k<=size(qs,2)
    l1=(o1-qs(:,k))/norm(o1-qs(:,k));
    l2=(o2-qs(:,k))/norm(o2-qs(:,k));
    h1s=norm(o1-qs(:,k))-ds;
    h2s=norm(o2-qs(:,k))-ds;

    kappa=250*log(2);
    bcs=10*(-1/kappa)*log ( exp(-kappa*h1s)+exp(-kappa*h2s) );
    W=-exp (-kappa*(h1s-bcs))*l1-exp (-kappa*(h2s-bcs))*l2;
    ACS=-W';

    H=[2,0;0,2];
    f=[0;-10];

    vstarcxs=quadprog(H,f,ACS,bcs, [], [], [], [], [0;0], opt);
    vstarcxs_r(:,k)=vstarcxs;
    k=k+1;
end


return;

figure(3)
hold on
box on;
xlim([-0.5 0.5]);
ylim([-0.1 5.2]);

plot(qs(2,:),vstarcxs_r(1,:), 'DisplayName', '$[v^*_{r}]_1$','LineWidth',2,'Color','b');
plot(qs(2,:),vstarcxs_r(2,:), 'DisplayName', '$[v^*_{r}]_2$','LineWidth',2,'Color','g','LineStyle','--');

legend;
legend('$[v^*_{r}]_1', '$[v^*_{r}]_2$','Location', 'northwest');


xlabel('$[q_e]_2$(rad)');
ylabel('$v^*_{r}$(rad/s)');





















