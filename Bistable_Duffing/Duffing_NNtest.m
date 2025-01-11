load('Linearize_duff_data_new.mat')
    


%%
N_IC_test=10;
E1=-0.25*sort(rand(N_IC_test,1));
E2=-0.25*sort(rand(N_IC_test,1));
E3=2*sort(rand(N_IC_test,1));
Es=[E1; E2; E3];
xmax=[sqrt(1+sqrt(1+4*E1));  -sqrt(1-sqrt(1+4*E2)); sqrt(1+sqrt(1+4*E3))];
xmin=[sqrt(1-sqrt(1+4*E1));  -sqrt(1+sqrt(1+4*E2)); -sqrt(1+sqrt(1+4*E3))];
IC_pos_test=(xmax-xmin).*rand(3*N_IC_test,1)+xmin;
IC_vel_test=sign(rand(3*N_IC_test,1)-0.5).*sqrt(2*Es+IC_pos_test.^2-0.5*IC_pos_test.^4);
%%

%IC_pos_test=[0.95.*sort(rand(N_IC_test,1));  -0.95.*sort(rand(N_IC_test,1)); sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%IC_pos_test=1;%[(sqrt(2)-0.05).*sort(rand(N_IC_test,1));  -(sqrt(2)-0.05).*sort(rand(N_IC_test,1)); sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%IC_pos_test=1.7;%[sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%sign(rand(N_IC_test,1)-0.5).*
t_test=[0:0.01:50];
test_traj_duff=zeros(3*N_IC_test,length(t_test),2);
test_traj_lin_osci=zeros(3*N_IC_test,length(t_test),3);



parfor iter_IC=1:3*N_IC_test%
    x0=[IC_pos_test(iter_IC);IC_vel_test(iter_IC)];
    
    [~, x] = ode45(@(t,x) Duffing(t,x), t_test,x0,opts);
    
    test_traj_duff(iter_IC,:,:)=x;
    r_duff=sqrt(x(:,1).^2+x(:,2).^2);
    
    if 0.5*x(1,2).^2-0.5*x(1,1).^2+0.25*x(1,1).^4<0
        E_duff=sign(x(:,1));
        phi_duff=atan2(x(:,1),x(:,2));
    else
        E_duff=0.*x(:,1);
        phi_duff=mod(atan2(x(:,1),x(:,2))-pi/2,2*pi)+pi/2;
    end
    
    x0_lin2=[r_duff(1)*sin(phi_duff(1)); r_duff(1)*cos(phi_duff(1)); E_duff(1)];
    x0_lin=net_nonlin2lin([r_duff(1);phi_duff(1); E_duff(1)]);
    x0_lin=[x0_lin(1)*sin(x0_lin(2)); x0_lin(1)*cos(x0_lin(2));x0_lin(3)];
    [t, x] = ode45(@(t,x) Lin_osci(t,x), t_test,x0_lin,opts);
    rs_lin_test=sqrt(x(:,1).^2+x(:,2).^2);
    phis_tmp=atan2(x(:,1),x(:,2))+2*pi;
    jump_idx=find(abs(diff(phis_tmp))>0.1);
    
    for iter_jmp=1:length(jump_idx)
        phis_tmp(jump_idx(iter_jmp)+1:end)=2*pi+phis_tmp(jump_idx(iter_jmp)+1:end);
        
        
    end
    if x(1,3)<-0.5
        phis0=-pi/2;
    else
        phis0=pi/2;
    end
    
    phis_lin_test=phis_tmp;
    T_duff_test=f_T_duff(IC_pos_test(iter_IC));
    T_duff_test=net_T([rs_lin_test mod(phis_lin_test-phis0,2*pi)+phis0  x(:,3)].').';

    %phis_lin_test=(phis_lin_test-phis0)./T_pendulum_test.*2*pi+phis0;
    phis_lin_test=(phis_lin_test-phis_lin_test(1))./T_duff_test.*2*pi+phis_lin_test(1);

    phis_lin_test=mod(phis_lin_test-phis0,2*pi)+phis0;
    
    N_steps=length(rs_lin_test);
    out_nn = zeros( N_steps,3);
    N_disp=floor(N_steps/100);
    
    for iter_t=1:length(t)
        out_nn(iter_t,:) = net_lin2nonlin( [rs_lin_test(iter_t);phis_lin_test(iter_t); x(iter_t,3)] );
        
        if floor(iter_t/N_disp)*N_disp==iter_t
            disp(['Progress: ' num2str(round(iter_t/N_steps*100,2)) ' %'])
        end
    end
    test_traj_lin_osci(iter_IC,:,:)=out_nn;
end

%%
my_green=[143 209 41]/255;
my_blue=[7 149 214]/255;
my_pink=[218 112 214 ]/255;
cmap=[repmat(my_green,N_IC_test,1); repmat(my_blue,N_IC_test,1);repmat(my_pink,N_IC_test,1)];

figure
for iter_IC=1:3*N_IC_test
    
     
    plot(test_traj_duff(iter_IC,:,1),test_traj_duff(iter_IC,:,2),'Color',cmap(iter_IC,:))
    
    hold on
    plot(test_traj_duff(iter_IC,1,1),test_traj_duff(iter_IC,1,2),'o','Color',cmap(iter_IC,:))
    plot(test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2)),test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2)),'-.','Color',cmap(iter_IC,:))
end

xlabel('Postion $x_1$','Fontsize',12,'Interpreter','latex')
ylabel('Velocity $x_2$','Fontsize',12,'Interpreter','latex')
figure
for iter_IC=1:3*N_IC_test
    subplot(3,1,1+floor((iter_IC-1)/N_IC_test))
    
    plot(t_test,test_traj_duff(iter_IC,:,1),'Color',cmap(iter_IC,:))
    hold on
    
    plot(t_test,test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2)),'-.','Color',cmap(iter_IC,:))
end

subplot(3,1,1)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Position $x_1$','Fontsize',12,'Interpreter','latex')
%legend('Duffing Oscillator', 'Transformed Linear Oscillator')
subplot(3,1,2)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Position $x_1$','Fontsize',12,'Interpreter','latex')
%legend('Duffing Oscillator', 'Transformed Linear Oscillator')
subplot(3,1,3)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Position $x_1$','Fontsize',12,'Interpreter','latex')
%legend('Duffing Oscillator', 'Transformed Linear Oscillator')
%%
figure
for iter_IC=1:3*N_IC_test
    subplot(3,1,1+floor((iter_IC-1)/N_IC_test))
    lin_osci=[test_traj_lin_osci(iter_IC,:,1).*sin(test_traj_lin_osci(iter_IC,:,2));test_traj_lin_osci(iter_IC,:,1).*cos(test_traj_lin_osci(iter_IC,:,2))];
    hold on
    plot(t_test,vecnorm(squeeze(test_traj_duff(iter_IC,:,:)).'-lin_osci)./vecnorm(lin_osci),'Color',cmap(iter_IC,:))
    
    
end
subplot(3,1,1)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
%ylabel('Relative error |x_{NL}(t)-x_{Lin}(t)|')
subplot(3,1,2)
xlabel('Time  $t$','Fontsize',12,'Interpreter','latex')
ylabel('Relative error $|x_{NL}(t)-x_{Lin}(t)|/x_{NL}$','Fontsize',12,'Interpreter','latex')
subplot(3,1,3)
xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
%ylabel('Relative error |x_{NL}(t)-x_{Lin}(t)|')
