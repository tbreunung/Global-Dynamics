load('Linearize_pendulum_data_new.mat')

%%
N_IC_test=10;
Es=2*sort(rand(N_IC_test,1))-1;
xmax=acos(-Es);
xmin=-acos(-Es);
IC_pos_test=(xmax-xmin).*rand(N_IC_test,1)+xmin;
IC_vel_test=sign(rand(N_IC_test,1)-0.5).*sqrt(2*Es+2*cos(IC_pos_test));
%%
%IC_pos_test=1.9*pi.*sort(rand(N_IC_test,1))-0.95*pi;

t_test=[0:0.01:50];
test_traj_pendulum=zeros(N_IC_test,length(t_test),2);
test_traj_lin_osci=zeros(N_IC_test,length(t_test),2);



parfor iter_IC=1:N_IC_test
    %x0=[IC_pos_test(iter_IC);0];
    x0=[IC_pos_test(iter_IC);IC_vel_test(iter_IC)];
    [~, x] = ode45(@(t,x) Pendulum(t,x), t_test,x0,opts);
    
    test_traj_pendulum(iter_IC,:,:)=x;
    r_pendulum=sqrt(x(1,1).^2+x(1,2).^2);
    phi_pendulum=mod(atan2(x(1,1),x(1,2))-pi/2,2*pi)+pi/2%atan2(x(1,1),x(1,2));
    
    x0_lin2=[r_pendulum*sin(phi_pendulum); r_pendulum*cos(phi_pendulum) ];
    x0_lin=net_nonlin2lin([r_pendulum(1);phi_pendulum(1)]);
    x0_lin=[x0_lin(1)*sin(x0_lin(2)); x0_lin(1)*cos(x0_lin(2))];
    [t, x] = ode45(@(t,x) Lin_osci(t,x), t_test,x0_lin,opts);
    rs_lin_test=sqrt(x(:,1).^2+x(:,2).^2);
    phis_tmp=atan2(x(:,1),x(:,2));
    jump_idx=find(abs(diff(phis_tmp))>0.1);
    
    for iter_jmp=1:length(jump_idx)
        phis_tmp(jump_idx(iter_jmp)+1:end)=2*pi+phis_tmp(jump_idx(iter_jmp)+1:end);
        
        
    end
    phis_lin_test=phis_tmp;
    %T_pendulum_test=f_T_pendulum(IC_pos_test(iter_IC));
    T_pendulum_test=net_T([rs_lin_test mod(phis_lin_test-pi/2,2*pi)+pi/2 ].').';
    phis_lin_test=(phis_lin_test-phis_lin_test(1))./T_pendulum_test*2*pi+phis_lin_test(1);
    phis_lin_test=mod(phis_lin_test-pi/2,2*pi)+pi/2;
    
    N_steps=length(rs_lin_test);
    out_nn = zeros( N_steps,2);
    N_disp=floor(N_steps/100);
    
    for iter_t=1:length(t)
        out_nn(iter_t,:) = net_lin2nonlin( [rs_lin_test(iter_t);phis_lin_test(iter_t)] );
        
        if floor(iter_t/N_disp)*N_disp==iter_t
            disp(['Progress: ' num2str(round(iter_t/N_steps*100,2)) ' %'])
        end
    end
    test_traj_lin_osci(iter_IC,:,:)=out_nn;
end

%%
cmap=[linspace(0,1, N_IC_test)', zeros(N_IC_test, 1),fliplr(linspace(0, 1, N_IC_test))'];

figure
for iter_IC=1:N_IC_test
    plot(test_traj_pendulum(iter_IC,:,1),test_traj_pendulum(iter_IC,:,2),'Color',cmap(iter_IC,:))
    
    hold on
    plot(test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2)),test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2)),'-.','Color',cmap(iter_IC,:))
    plot(test_traj_pendulum(iter_IC,1,1),test_traj_pendulum(iter_IC,1,2),'o','Color',cmap(iter_IC,:))
    
    
 end
legend('Pendulum', 'Transformed Linear Oscillator','Fontsize',12,'Interpreter','latex')

xlabel('Postion','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
%%
cmap=[linspace(0,1, N_IC_test)', zeros(N_IC_test, 1),fliplr(linspace(0, 1, N_IC_test))'];

figure
for iter_IC=1:N_IC_test
   subplot(2,1,1)
    plot(t_test,test_traj_pendulum(iter_IC,:,1),'Color',cmap(iter_IC,:))
    hold on
    
    plot(t_test,test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2)),'-.','Color',cmap(iter_IC,:))
    
    
    subplot(2,1,2)
    plot(t_test,test_traj_pendulum(iter_IC,:,2),'Color',cmap(iter_IC,:))
    hold on
    
    plot(t_test,test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2)),'-.','Color',cmap(iter_IC,:))
end
subplot(2,1,1)
xlabel('Time','Fontsize',12,'Interpreter','latex')
ylabel('Position','Fontsize',12,'Interpreter','latex')
legend('Pendulum', 'Transformed Linear Oscillator','Fontsize',12,'Interpreter','latex')
subplot(2,1,2)
xlabel('Time','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
legend('Pendulum', 'Transformed Linear Oscillator','Fontsize',12,'Interpreter','latex')
%%
figure
for iter_IC=1:N_IC_test
    %subplot(2,1,1)
    lin_osci=[test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2));test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2))];
    hold on
    plot(t_test,vecnorm(squeeze(test_traj_pendulum(iter_IC,:,:)).'-lin_osci)./vecnorm(squeeze(test_traj_pendulum(iter_IC,:,:)).'),'Color',cmap(iter_IC,:))
    %
   % subplot(2,1,2)
   % plot(t_test,abs(squeeze(test_traj_pendulum(iter_IC,:,1))-lin_osci(1,:)),'Color',cmap(iter_IC,:))
   % hold on
end
%subplot(2,1,1)
xlabel('Time ','Fontsize',12,'Interpreter','latex')
ylabel('Relative error $|x_{NL}(t)-x_{Lin}(t)|/|x_{NL}(t)|$','Fontsize',12,'Interpreter','latex')
%subplot(2,1,2)
%xlabel('time units')
%ylabel('Position error |x_{NL}(t)-x_{Lin}(t)|')