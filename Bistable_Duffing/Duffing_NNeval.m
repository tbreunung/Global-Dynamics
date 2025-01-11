load('Linearize_duff_data_new.mat')
    
%%
N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_lin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,3);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_lin2nonlin( [rs_lin{plt_idx(iter_IC)}(jj);phis_lin{plt_idx(iter_IC)}(jj);Es_lin{plt_idx(iter_IC)}(jj)] );
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

  subplot(3,1,1)
    plot(linspace(0,1,length(rs_duff{plt_idx(iter_IC)})),rs_duff{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(3,1,2)
    plot((1:length(phis_duff{plt_idx(iter_IC)}))./length(phis_duff{plt_idx(iter_IC)}),phis_duff{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(3,1,3)
    plot(linspace(0,1,length(Es_duff{plt_idx(iter_IC)})),Es_duff{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,3))),plot_data{iter_IC}(:,3),'--','Color',cmap(iter_IC,:))
    
    drawnow
end

%%

N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_duff{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,3);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_nonlin2lin([rs_duff{plt_idx(iter_IC)}(jj);phis_duff{plt_idx(iter_IC)}(jj);Es_duff{plt_idx(iter_IC)}(jj)] );
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

  subplot(3,1,1)
    plot(linspace(0,1,length(rs_lin{plt_idx(iter_IC)})),rs_lin{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(3,1,2)
    plot((1:length(phis_lin{plt_idx(iter_IC)}))./length(phis_lin{plt_idx(iter_IC)}),phis_lin{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(3,1,3)
    plot(linspace(0,1,length(Es_lin{plt_idx(iter_IC)})),Es_lin{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,3))),plot_data{iter_IC}(:,3),'--','Color',cmap(iter_IC,:))

    drawnow
end
%%
N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_lin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,1);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_T([rs_lin{plt_idx(iter_IC)}(jj);phis_lin{plt_idx(iter_IC)}(jj);Es_lin{plt_idx(iter_IC)}(jj)] );
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

   
    plot(linspace(0,1,length(T_per_duff{plt_idx(iter_IC)})),T_per_duff{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    
    drawnow
end