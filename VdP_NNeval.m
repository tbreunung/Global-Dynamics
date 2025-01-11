load('Linearize_VdP.mat')
%%

N_plt=min(10,N_IC_train);

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_lin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,2);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_lin2nonlin( traj_lin{plt_idx(iter_IC)}(jj,:).' );
        %[int1_lin2nonlin(traj_lin{plt_idx(iter_IC)}(jj,:));int2_lin2nonlin(traj_lin{plt_idx(iter_IC)}(jj,:))];
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

figure
for iter_IC=1:N_plt
    if ~isempty(traj_nonlin{plt_idx(iter_IC)})
        
        subplot(2,1,1)
        plot(linspace(0,1,length(traj_nonlin{plt_idx(iter_IC)}(:,1))),traj_nonlin{plt_idx(iter_IC)}(:,1))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--')%,'Color',cmap(iter_IC,:))
        drawnow
        
        subplot(2,1,2)
        plot(linspace(0,1,length(traj_nonlin{plt_idx(iter_IC)}(:,2))),traj_nonlin{plt_idx(iter_IC)}(:,2))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--')%,'Color',cmap(iter_IC,:))
        drawnow
    end
    
end

%%

N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_nonlin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,2);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_nonlin2lin( traj_nonlin{plt_idx(iter_IC)}(jj,:).' );
        %[int1_nonlin2lin(traj_nonlin{plt_idx(iter_IC)}(jj,:));int2_nonlin2lin(traj_nonlin{plt_idx(iter_IC)}(jj,:))];
        if floor(jj/N_disp)*N_disp==jj
            disp(['Progress: ' num2str(round(jj/N_steps*100,2)) ' %'])
        end
    end
    plot_data{iter_IC}=out_nn;
end

%%
cmap= [linspace(0,1, N_plt)', zeros(N_plt, 1),fliplr(linspace(0, 1, N_plt))'];
figure
for iter_IC=1:N_plt
    
    
    if ~isempty(traj_nonlin{plt_idx(iter_IC)})
        subplot(2,1,1)
        plot(linspace(0,1,length(traj_lin{plt_idx(iter_IC)}(:,1))),traj_lin{plt_idx(iter_IC)}(:,1))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--')%,'Color',cmap(iter_IC,:))
        drawnow
        
        subplot(2,1,2)
        plot(linspace(0,1,length(traj_nonlin{plt_idx(iter_IC)}(:,2))),traj_lin{plt_idx(iter_IC)}(:,2))%,'Color',cmap(iter_IC,:))
        hold on
        if get(gca,'ColorOrderIndex')>1
            set(gca,'ColorOrderIndex',get(gca,'ColorOrderIndex')-1)
        end
        plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--')%,'Color',cmap(iter_IC,:))
        title(num2str(iter_IC))
        drawnow
    end
    
end
