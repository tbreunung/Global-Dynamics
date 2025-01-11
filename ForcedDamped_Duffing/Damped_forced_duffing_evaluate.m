clear all
close all
clc
%%
load('Duff_forced_damp_data.mat')

%%
N_plt=10;
num_ICs=length(ICs(:,1));
plt_idx=round(linspace(1,num_ICs,N_plt));

plot_data=cell(N_plt,1);
N_dim=length(traj_nonlin{1}(1,:));
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_lin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,N_dim);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_lin2nonlin( traj_lin{plt_idx(iter_IC)}(jj,:).' );
        %out_nn(jj,:) = [f_r(phis_pendulum{iter_IC}(jj));f_phi(phis_pendulum{iter_IC}(jj))];
        %ynn(jj) = net(ynn(jj-delay_steps:jj-1));
        if floor(jj/N_disp)*N_disp==jj
            disp(['Progress: ' num2str(round(jj/N_steps*100,2)) ' %'])
        end
    end
    plot_data{iter_IC}=out_nn;
end

%%
%cmap= [linspace(0,1, N_plt)', zeros(N_plt, 1),fliplr(linspace(0, 1, N_plt))'];



for iter_IC=1:N_plt
    figure
    for iter_dim=1:N_dim
        subplot(N_dim,1,iter_dim)
        plot(t_train,traj_nonlin{plt_idx(iter_IC)}(:,iter_dim))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(t_train,plot_data{iter_IC}(:,iter_dim),'--')%,'Color',cmap(iter_IC,:))
    
    end
     drawnow
    
end
%%




plt_idx=round(linspace(1,num_ICs,N_plt));

N_dim=length(traj_lin{1}(1,:));
plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_nonlin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,N_dim);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_nonlin2lin( traj_nonlin{plt_idx(iter_IC)}(jj,:).' );
        %out_nn(jj,:) = [f_r(phis_pendulum{iter_IC}(jj));f_phi(phis_pendulum{iter_IC}(jj))];
        %ynn(jj) = net(ynn(jj-delay_steps:jj-1));
        if floor(jj/N_disp)*N_disp==jj
            disp(['Progress: ' num2str(round(jj/N_steps*100,2)) ' %'])
        end
    end
    plot_data{iter_IC}=out_nn;
end

%%

for iter_IC=1:N_plt
    figure
    for iter_dim=1:N_dim
        subplot(N_dim,1,iter_dim)
        plot(t_train,traj_lin{plt_idx(iter_IC)}(:,iter_dim))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(t_train,plot_data{iter_IC}(:,iter_dim),'--')%,'Color',cmap(iter_IC,:))
    end
    drawnow
    
    
end

%%

