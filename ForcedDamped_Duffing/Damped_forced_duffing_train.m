clear all
close all
clc
%%
Om=1.3;
T=2*pi/Om;
a=0.1;
D=0.02;
Duff_forced=@(t,x)  [x(2); -x(1)-x(1)^3-D*x(2)+a*cos(x(3));Om];

%Om=0.942953987840798;
D_lin=0.01;
w_lin=1.3;
Lin_osci=@(t,x) [x(2);-w_lin^2*x(1)-2*D_lin*x(2);Om;0];
opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
N_hidden=40;
train_GPU=false;
%%
t_train=(0:0.01:1)*T;
t_train(end)=[];
load('Duff_forced_damped_ICs.mat')
ICs=[IC_high; IC_low];

num_ICs_train=length(ICs(:,1));
% phis=linspace(0,2*pi,num_ICs_train+1);
% phis(end)=[];
%
% X1=x_per_high(1)+0.05*sin(phis);%2*(rand(num_ICs_train,1)-0.5);
% X2=x_per_high(2)+0.05*cos(phis);%2*(rand(num_ICs_train,1)-0.5);

x0_low=[-0.1483    0.0075];
[~, x_PO_low] = ode45(@(t,x) Duff_forced(t,x), t_train,[x0_low 0],opts);
x0_high=[0.9911    0.4076];
[~, x_PO_high] = ode45(@(t,x) Duff_forced(t,x), t_train,[x0_high 0],opts);

Duff_data=[];
lin_osci_data=[];
traj_lin=cell(num_ICs_train,1);
traj_nonlin=cell(num_ICs_train,1);

for iter_IC=1:num_ICs_train
     
   
    
    
    [~, x] = ode45(@(t,x) Duff_forced(t,x), t_train,ICs(iter_IC,:),opts);
    
    
    %T_mean=2*abs(mean(diff(t((abs(diff(sign(x(:,1))))==2)))))
    
    if iter_IC<length(IC_high(:,1))+1
        %x(:,1:2)=x(:,1:2)-x_PO_low(:,1:2);
        Es=0.*x(:,1);
        X0=[ICs(iter_IC,1:2) 0  0]; %-x0_low
    %elseif norm(x(end,1:2)-x0_high)<0.1  
        %x=x-x_PO_high;
        %X0=[IC(1:2)-x0_high 0  1];
    else
        %x(:,1:2)=x(:,1:2)-x_PO_high(:,1:2);
        X0=[ICs(iter_IC,1:2) 0  1];%-x0_high
        Es=ones(size(x(:,1)));
    end
    
    
    
    traj_nonlin{iter_IC}=[x Es];
    Duff_data=[Duff_data;[x Es]];
    
    [t, x] = ode45(@(t,x) Lin_osci(t,x), t_train ,X0,opts);
    lin_osci_data=[lin_osci_data;x];
    traj_lin{iter_IC}=x;
    disp(['Progress:' num2str(round(iter_IC/num_ICs_train,2))])
    
end

%%
net_lin2nonlin = feedforwardnet([N_hidden N_hidden N_hidden]);%
%net_lin2nonlin.layers{1}.transferFcn = 'logsig';
%net_lin2nonlin.layers{2}.transferFcn = 'radbas';
%net_lin2nonlin.layers{3}.transferFcn = 'purelin';

%net_lin2nonlin.layers{1}.transferFcn = 'poslin';
%net_lin2nonlin.layers{2}.transferFcn = 'poslin';
%net_lin2nonlin.layers{3}.transferFcn = 'poslin';
%


%net.trainParam.showWindow = false;
net_lin2nonlin.trainParam.showCommandLine= true;
net_lin2nonlin.trainParam.epochs=2000;
net_lin2nonlin.trainParam.max_fail = 20;

%net.inputs{1}.size=2;
%f_r=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,1));
%f_phi=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,2));
%
if train_GPU==true
    net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',Duff_data.','useGPU','yes');
else
    net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',Duff_data.','useParallel','yes');    
end

%%
net_nonlin2lin= feedforwardnet([N_hidden N_hidden N_hidden]);%
%net_nonlin2lin.layers{1}.transferFcn = 'logsig';
%net_nonlin2lin.layers{2}.transferFcn = 'radbas';
%net_nonlin2lin.layers{3}.transferFcn = 'purelin';

% net_nonlin2lin.layers{1}.transferFcn = 'poslin';
% net_nonlin2lin.layers{2}.transferFcn = 'poslin';
% net_nonlin2lin.layers{3}.transferFcn = 'poslin';



%net.trainParam.showWindow = false;
net_nonlin2lin.trainParam.showCommandLine= true;
net_nonlin2lin.trainParam.epochs=2000;
net_nonlin2lin.trainParam.max_fail = 20;

%net.inputs{1}.size=2;
%f_r=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,1));
%f_phi=griddedInterpolant(pendulum_data(:,2),lin_osci_data(:,2));
%
if train_GPU==true
    net_nonlin2lin =  train(net_nonlin2lin,Duff_data.',lin_osci_data.','useGPU','yes');
else 
    net_nonlin2lin = train(net_nonlin2lin,Duff_data.',lin_osci_data.','useParallel','yes');
end
%%
id=length(dir('*mat'))+1;
save(['Duff_forced_damp_data_' num2str(id)],'net_lin2nonlin','net_nonlin2lin','lin_osci_data','Duff_data',...
    'traj_lin','traj_nonlin','t_train','ICs','x_PO_low','x_PO_high','x0_low','x0_high',...
    'T','Duff_forced','Lin_osci')
