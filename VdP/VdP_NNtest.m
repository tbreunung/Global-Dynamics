clear all
close all
clc
%%
load('Linearize_VdP.mat')






%%
N_IC_test=10;



IC_test= zeros(N_IC_test,2);%[ 3*cos(phis_test)  5*sin(phis_test) ];%[rs_test.*sin(phis_test) rs_test.*cos(phis_test)];

t_test=(0:0.01:5)*T;
test_traj_VdP=zeros(N_IC_test,length(t_test),2);
test_traj_lin_osci=zeros(N_IC_test,length(t_test),2);



parfor iter_IC=1:N_IC_test
    chck=false;
    while chck==false
        phis_test=2*pi.*rand(1);
        rs_test=rand(1)*2.9+0.1;
        
        IC_test_tmp= [ rs_test*cos(phis_test)  rs_test*sin(phis_test) ];
        [~, x] = ode45(@(t,x) VdP(t,x), t_test,IC_test_tmp,opts);
        if max((x(:,1).^2/9+x(:,2).^2/25))<=1
            chck=true;
        IC_test(iter_IC,:)=IC_test_tmp;
        end
    end
        test_traj_VdP(iter_IC,:,:)=x;
        %X0=[10*sqrt(IC_test(iter_IC,1)^2+IC_test(iter_IC,2)^2)   atan2(IC_test(iter_IC,1),IC_test(iter_IC,2))];
        [~,idx]=min(vecnorm(VdP_data-IC_test(iter_IC,:),2,2));
        
        IC_test_lin=lin_osci_data(idx,:);
        %net_nonlin2lin(IC_test(iter_IC,:).');%
        %IC_test_lin=[int1_nonlin2lin(IC_test(iter_IC,:)); int2_nonlin2lin(IC_test(iter_IC,:))];
        IC_test_lin=[sqrt(IC_test_lin(1)^2+IC_test_lin(2)^2); atan2(IC_test_lin(1),IC_test_lin(2))];
        [t, x] = ode45(@(t,x) Lin_LC(t,x), t_test,IC_test_lin.',opts);
        X=[x(:,1).*sin(x(:,2)) x(:,1).*cos(x(:,2))];
        N_steps=length(t_test);
        out_nn = zeros( N_steps,2);
        N_disp=floor(N_steps/100);
        
        for iter_t=1:length(t)
            
            out_nn(iter_t,:) = net_lin2nonlin( X(iter_t,:).' );
            %[int1_lin2nonlin(X(iter_t,:));int2_lin2nonlin(X(iter_t,:))];
            %
            
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
    plot(test_traj_VdP(iter_IC,:,1),test_traj_VdP(iter_IC,:,2))%,'Color',cmap(iter_IC,:))
    
    hold on
    %plot(test_traj_duff(iter_IC,:,1),test_traj_duff(iter_IC,1,2),'s')
    
    if get(gca,'ColorOrderIndex')>1
        set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
    else
        set(gca,'ColorOrderIndex',7)
        
    end
    plot(test_traj_lin_osci(iter_IC,:,1),test_traj_lin_osci(iter_IC,:,2),'-.')%,'Color',cmap(iter_IC,:))
    
    figure(fig_ts)
    subplot(2,1,1)
    plot(t_test,test_traj_VdP(iter_IC,:,1))%,'Color',cmap(iter_IC,:))
    hold on
    if get(gca,'ColorOrderIndex')>1
        set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
    else
        set(gca,'ColorOrderIndex',7)
    end
    plot(t_test,test_traj_lin_osci(iter_IC,:,1),'-.')%,'Color',cmap(iter_IC,:))
    
    subplot(2,1,2)
    plot(t_test,test_traj_VdP(iter_IC,:,2))%,'Color',cmap(iter_IC,:))
    hold on
    if get(gca,'ColorOrderIndex')>1
        set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
    else
        set(gca,'ColorOrderIndex',7)
    end
    plot(t_test,test_traj_lin_osci(iter_IC,:,2),'-.')%,'Color',cmap(iter_IC,:))
    %title(num2str(iter_IC))

end
figure(fig_ps)
xlabel('Postion $x_1$','Fontsize',12,'Interpreter','latex')
ylabel('Velocity $x_2$','Fontsize',12,'Interpreter','latex')
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
    
    plot(t_test,vecnorm(squeeze(test_traj_VdP(iter_IC,:,:)).'-squeeze(test_traj_lin_osci(iter_IC,:,:)).')./vecnorm(squeeze(test_traj_VdP(iter_IC,:,:)).'));%,'Color',cmap(iter_IC,:))
    hold on
%     subplot(2,1,2)
%     plot(t_test,abs(squeeze(test_traj_VdP(iter_IC,:,1))-squeeze(test_traj_lin_osci(iter_IC,:,1))));%,'Color',cmap(iter_IC,:))
%     hold on
end

xlabel('Time $t$','Fontsize',12,'Interpreter','latex')
ylabel('Relative error |x_{NL}(t)-x_{Lin}(t)|./|x_{NL}|')


