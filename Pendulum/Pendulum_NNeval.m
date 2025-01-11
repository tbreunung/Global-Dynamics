load('Linearize_pendulum_data_new.mat')
    
%%
N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_lin{plt_idx(iter_IC)});
    out_nn = zeros( N_steps,2);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_lin2nonlin( [rs_lin{plt_idx(iter_IC)}(jj);phis_lin{plt_idx(iter_IC)}(jj)] );
        %out_nn(jj,:) = [f_r(phis_pendulum{iter_IC}(jj));f_phi(phis_pendulum{iter_IC}(jj))];
        %ynn(jj) = net(ynn(jj-delay_steps:jj-1));
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

  subplot(2,1,1)
    plot(linspace(0,1,length(rs_pendulum{plt_idx(iter_IC)})),rs_pendulum{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(2,1,2)
    plot((1:length(phis_pendulum{plt_idx(iter_IC)}))./length(phis_pendulum{plt_idx(iter_IC)}),phis_pendulum{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--','Color',cmap(iter_IC,:))
    drawnow
end
subplot(2,1,1)
xlabel('time')
set(gca,'Xtick',0:0.25:1)
set(gca,'Xticklabel',{'0 T', 'T/4', 'T/2', '3T/4', 'T'})
ylabel('Raidus r')

subplot(2,1,2)
xlabel('time')
set(gca,'Xtick',0:0.25:1)
set(gca,'Xticklabel',{'0 T', 'T/4', 'T/2', '3T/4', 'T'})
ylabel('Angle \phi')

%%
N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_pendulum{plt_idx(iter_IC)});
    out_nn = zeros( N_steps,2);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_nonlin2lin( [rs_pendulum{plt_idx(iter_IC)}(jj);phis_pendulum{plt_idx(iter_IC)}(jj)] );
        %out_nn(jj,:) = [f_r(phis_pendulum{iter_IC}(jj));f_phi(phis_pendulum{iter_IC}(jj))];
        %ynn(jj) = net(ynn(jj-delay_steps:jj-1));
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

  subplot(2,1,1)
    plot(linspace(0,1,length(rs_lin{plt_idx(iter_IC)})),rs_lin{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    subplot(2,1,2)
    plot((1:length(phis_lin{plt_idx(iter_IC)}))./length(phis_lin{plt_idx(iter_IC)}),phis_pendulum{plt_idx(iter_IC)},'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,2))),plot_data{iter_IC}(:,2),'--','Color',cmap(iter_IC,:))
    drawnow
end
subplot(2,1,1)
xlabel('time')
set(gca,'Xtick',0:0.25:1)
set(gca,'Xticklabel',{'0 T', 'T/4', 'T/2', '3T/4', 'T'})
ylabel('Raidus r')

subplot(2,1,2)
xlabel('time')
set(gca,'Xtick',0:0.25:1)
set(gca,'Xticklabel',{'0 T', 'T/4', 'T/2', '3T/4', 'T'})
ylabel('Angle \phi')
%%
N_plt=10;

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(rs_lin{plt_idx(iter_IC)});
    out_nn = zeros(N_steps,1);
    N_disp=floor(N_steps/100);
    for jj = 1:N_steps
        
        out_nn(jj,:) = net_T([rs_lin{plt_idx(iter_IC)}(jj);phis_lin{plt_idx(iter_IC)}(jj)] );
        %out_nn(jj,:) = [f_r(phis_pendulum{iter_IC}(jj));f_phi(phis_pendulum{iter_IC}(jj))];
        %ynn(jj) = net(ynn(jj-delay_steps:jj-1));
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

   
    plot([0 1],T_per_pendulum{plt_idx(iter_IC)}.*[1 1],'Color',cmap(iter_IC,:))
    hold on
    plot(linspace(0,1,length(plot_data{iter_IC}(:,1))),plot_data{iter_IC}(:,1),'--','Color',cmap(iter_IC,:))
    drawnow
    
    
    drawnow
end


