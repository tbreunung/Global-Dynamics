clear all
close all
clc
%%
mu=1;
VdP=@(t,x) [x(2); -x(1)+mu*(x(2)-x(1)^2*x(2))];
Om=0.942953987840798;
D=-1.0593773424772468/2;
Lin_LC=@(t,x) [D*(x(1)-1);Om];%[x(2);-Om^2*x(1)+2*D*x(2)];%

T=2*pi/Om;

N_IC_train_outside=80;
N_IC_train_inside=20;

N_IC_train=N_IC_train_outside+N_IC_train_inside;

phis_out=linspace(0,2*pi,N_IC_train_outside+1);
phis_out(end)=[];

phis_in=linspace(0,2*pi,N_IC_train_inside+1);
phis_in(end)=[];


ICs_train=[ 3*cos(phis_out) 0.1*cos(phis_in); 5*sin(phis_out) 0.1*sin(phis_in)];%[0.1*sin(phis)  5*sin(phis); 0.1*cos(phis) 5*cos(phis)];

opts = odeset( 'RelTol',1e-10,'AbsTol',1e-12);

VdP_data=[];
lin_osci_data=[];
traj_lin=cell(N_IC_train,1);
traj_nonlin=cell(N_IC_train,1);
figure
% [~, x_VdP_ss] = ode45(@(t,x) VdP(t,x), [0 500*T],[1;0],opts);
% %
% [~, x_VdP_ss] = ode45(@(t,x) VdP(t,x), t_vec,x_VdP_ss(end,:),opts);
%
% VdP_data=[VdP_data;x_VdP_ss];
% traj_nonlin{N_IC_train+1}=x_VdP_ss;
%
 %X0=[1  atan2(x_VdP_ss(end,1),x_VdP_ss(end,2))-Om*t_vec(end)]; %atan2(x_VdP(end,1),x_VdP(end,2))
 %[~, x] = ode45(@(t,x) Lin_LC(t,x), t_vec ,X0,opts);
 %X=[x(:,1).*sin(x(:,2)) x(:,1).*cos(x(:,2))];

 %lin_osci_data=[lin_osci_data;X];
 %traj_lin{N_IC_train+1}=X;
 %lin_LC=X;

for iter_IC=1:N_IC_train
    
    % if [-5*cos(phis(iter_IC)) -3*sin(phis(iter_IC))]*VdP(0,ICs_train(:,iter_IC))>0
    %chck=false;
    %while chck==false
    %if vecnorm(VdP_data-ICs_train(:,iter_IC).',2,2)>0.01
%     chck=false;
%     while chck==false
%         [~, x_VdP] = ode45(@(t,x) VdP(t,x), t_vec,ICs_train(:,iter_IC),opts);
%         
%         if max((x_VdP(:,1).^2/9+x_VdP(:,2).^2/25))>1
%             ICs_train(:,iter_IC)=ICs_train(:,iter_IC)-0.05*[3*cos(phis(iter_IC)); 5*sin(phis(iter_IC))];
%             
%         else
%             chck=true;
%         end
%     end
%   
    if norm(ICs_train(:,iter_IC))<1
        t_vec=(0:0.001:2)*T;
    else
        t_vec=(0:0.001:1)*T;
    end
    [~, x_VdP] = ode45(@(t,x) VdP(t,x), t_vec,ICs_train(:,iter_IC),opts);
    
     if max((x_VdP(:,1).^2/9+x_VdP(:,2).^2/25))<=1
    %
    
    
%     min_dist=zeros(length(t_vec),1);
%     ss_idx=zeros(length(t_vec),1);
%     for iter_t=1:length(t_vec)
%         [min_dist(iter_t), ss_idx(iter_t)]=min(vecnorm(x_VdP_ss-x_VdP(iter_t,:),2,2));
%         t_idxs(iter_t)=iter_t;
%     end
%     t_idx=find(min_dist<0.01,1);
%     r_tmp=sqrt(x_VdP(end,1)^2+x_VdP(end,1)^2);
%     x_VdP= x_VdP(1:t_idx,:);
%     
    t_sim=(0:0.001:1)*T;
    [~, x_VdP_tmp] = ode45(@(t,x) VdP(t,x), t_sim,x_VdP(end,:),opts);
    
    
    t_idx=find(diff(sign(x_VdP_tmp(:,1)))==2);
    t_new=[t_vec  t_vec(end)+t_sim(2:t_idx)];
    %x_VdP=[x_VdP;x_VdP_tmp(2:t_idx,:)];
    
    % x_VdP=[sqrt(x_VdP(:,1).^2+x_VdP(:,2).^2)  atan2(x_VdP(:,1),x_VdP(:,2))];
    %
    % x_VdP=x_VdP-x_VdP_ss;
    subplot(2,2,1)
    plot(x_VdP(:,1),x_VdP(:,2))
    hold on
    %plot(x(1,1),x(1,2),'x')
    
    subplot(2,2,2)
    plot(t_vec,x_VdP(:,1))
    hold on
    
    
    traj_nonlin{iter_IC}=x_VdP;
    
    
    VdP_data=[VdP_data;x_VdP];
    
    
    %sqrt(ICs_train(1,iter_IC)^2+ICs_train(2,iter_IC)^2)-sqrt(x_VdP_tmp(t_idx,1)^2+x_VdP_tmp(t_idx,2)^2)+1
    %1+abs(atan2(x_VdP(1,1),x_VdP(1,2))-atan2(x_VdP(end,1),x_VdP(end,2)))
    X0=[sqrt(ICs_train(1,iter_IC)^2+ICs_train(1,iter_IC)^2) atan2(x_VdP_tmp(t_idx,1),x_VdP_tmp(t_idx,2))-Om*t_new(end)]; %atan2(x_VdP(end,1),x_VdP(end,2))
    %X0=[ sqrt(ICs_train(1,iter_IC)^2+ICs_train(1,iter_IC)^2) atan2(lin_LC(ss_idx(t_idx),1),lin_LC(ss_idx(t_idx),2))-Om*t_vec(t_idx)];%
     
    [~, x] = ode45(@(t,x) Lin_LC(t,x), t_new,X0,opts);
    
    x=x(1:length(t_vec),:);
    
    X=[x(:,1).*sin(x(:,2)) x(:,1).*cos(x(:,2))];
    
    subplot(2,2,3)
    plot(X(:,1),X(:,2))
    hold on
    subplot(2,2,4)
    plot(t_vec,X(:,1))
    hold on
    lin_osci_data=[lin_osci_data;X];
    traj_lin{iter_IC}=X;
    drawnow
     end
    
end

subplot(2,2,3)
xlabel('$x_1$','Fontsize',12,'Interpreter','latex')
ylabel('$x_2$','Fontsize',12,'Interpreter','latex')
grid on

subplot(2,2,1)
xlabel('$y_1$','Fontsize',12,'Interpreter','latex')
ylabel('$y_2$','Fontsize',12,'Interpreter','latex')
grid on

subplot(2,2,2)
ylabel('$x_1$','Fontsize',12,'Interpreter','latex')
xlabel('$t$','Fontsize',12,'Interpreter','latex')
grid on

subplot(2,2,4)
ylabel('$y_1$','Fontsize',12,'Interpreter','latex')
xlabel('$t$','Fontsize',12,'Interpreter','latex')
grid on

%%
% figure
% plot3(lin_osci_data(1:10:end,1),lin_osci_data(1:10:end,2),0*lin_osci_data(1:10:end,2),'x')
% hold on
% plot3(VdP_data(1:10:end,1),VdP_data(1:10:end,2),VdP_data(1:10:end,2)./VdP_data(1:10:end,2),'s')
% % %plot3([VdP_data(:,1).'; lin_osci_data(:,1).'],[VdP_data(:,2).'; lin_osci_data(:,2).'],VdP_data(:,2)./VdP_data(:,2),'s')
% %
% plot3([lin_osci_data(1:10:end,1).'; VdP_data(1:10:end,1).'  ],[ lin_osci_data(1:10:end,2).'; VdP_data(1:10:end,2).' ],[zeros(size(lin_osci_data(1:10:end,1).')); ones(size(lin_osci_data(1:10:end,1).'))],'k')
%%
net_lin2nonlin = feedforwardnet([40 40 40]);%
%net_lin2nonlin.layers{1}.transferFcn = 'logsig';
%net_lin2nonlin.layers{1}.transferFcn = 'radbas';
%net_lin2nonlin.layers{2}.transferFcn = 'purelin';

%net_lin2nonlin.layers{1}.transferFcn = 'poslin';
%net_lin2nonlin.layers{2}.transferFcn = 'poslin';
%net_lin2nonlin.layers{3}.transferFcn = 'poslin';
%
%%

%net.trainParam.showWindow = false;
net_lin2nonlin.trainParam.showCommandLine= true;
net_lin2nonlin.trainParam.epochs=1000;
net_lin2nonlin.trainParam.max_fail = 20;

%net.inputs{1}.size=2;
%f_r=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,1));
%f_phi=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,2));

net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',VdP_data.','useParallel','yes');

%int1_lin2nonlin=scatteredInterpolant(lin_osci_data(:,1),lin_osci_data(:,2),VdP_data(:,1),'nearest');
%int2_lin2nonlin=scatteredInterpolant(lin_osci_data(:,1),lin_osci_data(:,2),VdP_data(:,2),'nearest');

%net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',VdP_data.','useGPU','yes');

%%
net_nonlin2lin= feedforwardnet([40 40 40]);%
% net_nonlin2lin.layers{1}.transferFcn = 'logsig';
% net_nonlin2lin.layers{2}.transferFcn = 'radbas';
% net_nonlin2lin.layers{3}.transferFcn = 'purelin';

% net_nonlin2lin.layers{1}.transferFcn = 'poslin';
% net_nonlin2lin.layers{2}.transferFcn = 'poslin';
% net_nonlin2lin.layers{3}.transferFcn = 'poslin';


%%
%net.trainParam.showWindow = false;
net_nonlin2lin.trainParam.showCommandLine= true;
net_nonlin2lin.trainParam.epochs=1000;
net_nonlin2lin.trainParam.max_fail = 20;

net_nonlin2lin = train(net_nonlin2lin,VdP_data.',lin_osci_data.','useParallel','yes');

%int1_nonlin2lin=scatteredInterpolant(VdP_data(:,1),VdP_data(:,2),lin_osci_data(:,1),'nearest');
%int2_nonlin2lin=scatteredInterpolant(VdP_data(:,1),VdP_data(:,2),lin_osci_data(:,2),'nearest');


%%




%%
save Linearize_VdP_D_1
%%

