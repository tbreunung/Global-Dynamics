clear all
close all
clc
%%


Pendulum=@(t,x) [x(2);-sin(x(1))];

Lin_osci=@(t,x) [x(2);-x(1)];

figure
N_IC_train=50;
IC_pos_train=linspace(0.01,pi-0.01,N_IC_train);

opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
tol=10^-3;

pendulum_data=[];
lin_osci_data=[];
T_pendulum=[];

cmap=[linspace(0,1, N_IC_train)', zeros(N_IC_train, 1),fliplr(linspace(0, 1, N_IC_train))'];


for iter_IC=1:N_IC_train
    x0=[IC_pos_train(iter_IC);0];
    
    [t, x] = ode45(@(t,x) Pendulum(t,x), [0:0.001:100],x0,opts);
    x_per=extract_period(x, tol);
    subplot(2,2,3)
    plot(x_per(:,1),x_per(:,2),'Color',cmap(iter_IC,:))
    hold on
    subplot(2,2,4)
    plot(t(1:length(x_per(:,1))),x_per(:,1),'Color',cmap(iter_IC,:))
    hold on
    
    
    
    phis_tmp=atan2(x_per(:,1),x_per(:,2));
    jump_idx=find(abs(diff(phis_tmp))>0.1);
    
    for iter_jmp=1:length(jump_idx)
        phis_tmp(jump_idx(iter_jmp)+1:end)=2*pi+phis_tmp(jump_idx(iter_jmp)+1:end);
        
        
    end
    rs_pendulum{iter_IC}=sqrt(x_per(:,1).^2+x_per(:,2).^2);
    phis_pendulum{iter_IC}=phis_tmp;
    T_per_pendulum{iter_IC}=t(length(x_per(:,1)));
    
    pendulum_data=[pendulum_data;rs_pendulum{iter_IC}  phis_pendulum{iter_IC}];
    
    T_pendulum=[T_pendulum; repmat(t(length(x_per(:,1))),length(x_per(:,1)),1)];
    
    [t, x] = ode45(@(t,x) Lin_osci(t,x), [0:0.001:T_per_pendulum{iter_IC}],x0,opts);
    %x_per=x;
    x_per=extract_period(x, tol);
    subplot(2,2,1)
    plot(x_per(:,1),x_per(:,2),'Color',cmap(iter_IC,:))
    hold on
    subplot(2,2,2)
    plot(t(1:length(x_per(:,1))),x_per(:,1),'Color',cmap(iter_IC,:))
    hold on
    
    
    
    phis_tmp=atan2(x_per(:,1),x_per(:,2));
    jump_idx=find(abs(diff(phis_tmp))>0.1);
    
    for iter_jmp=1:length(jump_idx)
        phis_tmp(jump_idx(iter_jmp)+1:end)=2*pi+phis_tmp(jump_idx(iter_jmp)+1:end);
        
        
    end
    phis_lin{iter_IC}=phis_tmp;
    T_per_lin{iter_IC}=t(length(x_per(:,1)));
    rs_lin{iter_IC}=sqrt(x_per(:,1).^2+x_per(:,2).^2);
    
    N=length(rs_pendulum{iter_IC});
    rs_lin{iter_IC}=interp1((0:length(rs_lin{iter_IC})-1)./length(rs_lin{iter_IC}).*2.*pi,rs_lin{iter_IC},(0:N-1)./N.*2.*pi).';
    phis_lin{iter_IC}=interp1((0:length(phis_lin{iter_IC})-1)./length(phis_lin{iter_IC}).*2.*pi,phis_lin{iter_IC},(0:N-1)./N.*2.*pi).';
    
    lin_osci_data=[lin_osci_data;rs_lin{iter_IC}  phis_lin{iter_IC}];
    drawnow
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
net_lin2nonlin = feedforwardnet([20 20 20]);%

net_lin2nonlin.trainParam.showCommandLine= true;
net_lin2nonlin.trainParam.epochs=1000;

net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',pendulum_data.','useParallel','yes');
%%
net_nonlin2lin = feedforwardnet([20 20 20]);%

net_nonlin2lin.trainParam.showCommandLine= true;
net_nonlin2lin.trainParam.epochs=1000;

net_nonlin2lin = train(net_nonlin2lin,pendulum_data.',lin_osci_data.','useParallel','yes');

%%
net_T = feedforwardnet([20 20 20]);%

net_T.trainParam.showCommandLine= true;
net_T.trainParam.epochs=1000;

net_T= train(net_T,lin_osci_data.',T_pendulum.','useParallel','yes');
%%

for iter=1:N_IC_train
T_tmp(iter)=T_per_duff{iter}(1);
end
f_T_pendulum=griddedInterpolant(IC_pos_train,T_tmp);

%%
save('Linearize_pendulum_data_new')
    

%%

function x_per=extract_period(x, tol)

id=find( diff(sign(x(:,2)))==-2,1);

x_per=x(1:id+1,:);
end
