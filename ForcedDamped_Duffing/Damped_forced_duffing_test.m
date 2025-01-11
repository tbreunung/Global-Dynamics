clear all
close all
clc
%%
load('Duff_forced_damp_data.mat')
opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
my_green=[143 209 41]/255;
my_blue=[7 149 214]/255;
%%
N_IC_test=10;

 

IC_test=[2*rand(N_IC_test,1) 1.6*rand(N_IC_test,1)]-[0.8 1];

N_per_test=50;

t_test=(0:0.01:N_per_test)*T;
test_traj_duff=zeros(N_IC_test,length(t_test),4);
test_traj_lin_osci=zeros(N_IC_test,length(t_test),4);

y4=NaN(N_IC_test,1);
cmap=NaN(N_IC_test,3);
parfor iter_IC=1:N_IC_test
    
    IC=[IC_test(iter_IC,1) IC_test(iter_IC,2)  0 ];
    
   
    
    [~, x] = ode45(@(t,x) Duff_forced(t,x), t_test,IC,opts);
    
    if x(end,2)<0.1
        Es=ones(size(x(:,1)));
        cmap(iter_IC,:)=my_blue;
    else
        Es=zeros(size(x(:,1)));
         cmap(iter_IC,:)=my_green;

    end
    
    test_traj_duff(iter_IC,:,:)=[x Es];
    
    
    
    %IC_test_lin(3)=0;
    IC_test_lin=net_nonlin2lin([IC Es(1)].');
    [t, x] = ode45(@(t,x) Lin_osci(t,x), t_test ,IC_test_lin.',opts);
    %x(:,3)=mod(x(:,3),2*pi);
    N_steps=length(t_test);
    out_nn = zeros(N_steps,4);
    N_disp=floor(N_steps/100);
    
    for iter_t=1:N_steps
        out_nn(iter_t,:) = net_lin2nonlin( x(iter_t,:).' );
        
         if out_nn(iter_t,3)>2*pi 
             IC_test_lin=net_nonlin2lin(out_nn(iter_t,:).'-[0;0;2*pi;0]);
             [t, x_tmp] = ode45(@(t,x) Lin_osci(t,x), t_test,IC_test_lin.',opts);
             t_idx=length(x_tmp(iter_t:end,1));
             x(iter_t:end,:)=x_tmp(1:t_idx,:);
         end
        
        if floor(iter_t/N_disp)*N_disp==iter_t
            disp(['Progress: ' num2str(round(iter_t/N_steps*100,2)) ' %'])
        end
    end
    test_traj_lin_osci(iter_IC,:,:)=out_nn;
    
    
end

%%
%cmap=[linspace(0,1, N_IC_test)', zeros(N_IC_test, 1),fliplr(linspace(0, 1, N_IC_test))'];

fig_ps=figure;
fig_ts=figure;
for iter_IC=1:N_IC_test
    
    figure(fig_ps)
    plot3(test_traj_duff(iter_IC,:,3),test_traj_duff(iter_IC,:,1),test_traj_duff(iter_IC,:,2),'Color',cmap(iter_IC,:))
    
    hold on
      
    
    plot3(t_test,test_traj_lin_osci(iter_IC,:,1).',test_traj_lin_osci(iter_IC,:,2).','-.','Color',cmap(iter_IC,:))
    
    figure(fig_ts)
    subplot(2,1,1)
    plot(t_test,test_traj_duff(iter_IC,:,1),'Color',cmap(iter_IC,:))
    hold on
    plot(t_test,test_traj_lin_osci(iter_IC,:,1).','-.','Color',cmap(iter_IC,:))
    
    
    subplot(2,1,2)
    plot(t_test,test_traj_duff(iter_IC,:,2),'Color',cmap(iter_IC,:))
    hold on
   
     plot(t_test,test_traj_lin_osci(iter_IC,:,2).','-.','Color',cmap(iter_IC,:))
     
end
figure(fig_ps)
xlabel('Angle $x_3$','Fontsize',12,'Interpreter','latex')
ylabel('Postion $x_1$','Fontsize',12,'Interpreter','latex')
zlabel('Velocity $x_2$','Fontsize',12,'Interpreter','latex')
grid on

figure(fig_ts)
subplot(2,1,1)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Position $x_1$','Fontsize',12,'Interpreter','latex')
%legend('Nonlinear Oscillator', 'Transformed Linear Oscillator')

subplot(2,1,2)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Velocity $x_2$','Fontsize',12,'Interpreter','latex')
%legend('Nonlinear Oscillator', 'Transformed Linear Oscillator')
%%
figure
for iter_IC=1:N_IC_test
    
    %subplot(2,1,1)
    %lin_osci=[test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2));test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2))];
    hold on
    plot(t_test,vecnorm(squeeze(test_traj_duff(iter_IC,:,1:2)).'-squeeze(test_traj_lin_osci(iter_IC,:,1:2)).'),'Color',cmap(iter_IC,:))
    
%     subplot(2,1,2)
%     plot(t_test,abs(squeeze(test_traj_duff(iter_IC,:,1))-squeeze(test_traj_lin_osci(iter_IC,:,1))))%,'Color',cmap(iter_IC,:))
%     hold on
end
%subplot(2,1,1)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Relative error $|x_{NL}(t)-x_{Lin}(t)|/|x_{NL}(t)|$','Fontsize',12,'Interpreter','latex')


%%


