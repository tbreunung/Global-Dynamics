load('Linearize_VdP.mat')
    
%%
N_plt=10;%length(traj_lin);

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_lin{plt_idx(iter_IC)});
    out_nn = zeros( N_steps,2);
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
figure
x_in=[];
x_out=[];

for iter_IC=1:N_plt
    x_in=[x_in; NaN(1,2) ;traj_lin{plt_idx(iter_IC)}];
    
    x_out=[x_out; NaN(1,2); plot_data{iter_IC}];
    
end

x_out_interp=x_out(min(~isnan(x_in),[],2),:);

x_in_interp=x_in(min(~isnan(x_in),[],2),:);
[XX1,XX2]=ndgrid(linspace(min(x_in(:,1)),max(x_in(:,1)),100),linspace(min(x_in(:,2)),max(x_in(:,2))),100);
x1_out_interp=griddata(x_in_interp(:,1),x_in_interp(:,2),x_out_interp(:,1),XX1,XX2,'cubic');
x2_out_interp=griddata(x_in_interp(:,1),x_in_interp(:,2),x_out_interp(:,2),XX1,XX2,'cubic');

subplot(2,2,1)
plot3(x_in(:,1),x_in(:,2),x_out(:,1),'Color',[0.8 0.8 0.8])
hold on


surf(XX1,XX2,x1_out_interp,'EdgeColor','None');


xlabel('Position $y_1$','Interpreter','latex','FontSize',12)
ylabel('Velocity $y_2$','Interpreter','latex','FontSize',12)
zlabel('Position $x_1$','Interpreter','latex','FontSize',12)
grid on
%colormap summer
colorMap = [linspace(0,1,256)', zeros(256,1) linspace(1,0,256)'];
colormap(colorMap);
%lighting gouraud
%axis equal
subplot(2,2,2)
plot3(x_in(:,1),x_in(:,2),x_out(:,2),'Color',[0.8 0.8 0.8])

hold on
surf(XX1,XX2,x2_out_interp,'EdgeColor','None')
xlabel('Position $y_1$','Interpreter','latex','FontSize',12)
ylabel('Velocity $y_2$','Interpreter','latex','FontSize',12)
zlabel('Velocity $x_2$','Interpreter','latex','FontSize',12)
grid on
%axis equal
colormap(colorMap);
%lighting gouraud
drawnow
%%
N_plt=10;%length(traj_nonlin);

plt_idx=round(linspace(1,N_IC_train,N_plt));

plot_data=cell(N_plt,1);
parfor iter_IC=1:N_plt
    
    N_steps=length(traj_nonlin{plt_idx(iter_IC)});
    out_nn = zeros( N_steps,2);
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

%figure
x_in=[];
x_out=[];
for iter_IC=1:N_plt

   x_in=[x_in;NaN(1,2) ; traj_nonlin{plt_idx(iter_IC)}];
    
   x_out=[x_out; NaN(1,2) ; plot_data{iter_IC} ];
  
end


[XX1,XX2]=ndgrid(linspace(min(x_in(:,1)),max(x_in(:,1)),100),linspace(min(x_in(:,2)),max(x_in(:,2))),100);

x_out_interp=x_out(min(~isnan(x_in),[],2),:);

x_in_interp=x_in(min(~isnan(x_in),[],2),:);

x1_out_interp=griddata(x_in_interp(:,1),x_in_interp(:,2),x_out_interp(:,1),XX1,XX2,'cubic');
x2_out_interp=griddata(x_in_interp(:,1),x_in_interp(:,2),x_out_interp(:,2),XX1,XX2,'cubic');


subplot(2,2,3)
plot3(x_in(:,1),x_in(:,2),x_out(:,1),'Color',[0.8 0.8 0.8])
hold on
surf(XX1,XX2,x1_out_interp,'EdgeColor','None')
xlabel('Position $x_1$','Interpreter','latex','FontSize',12)
ylabel('Velocity $x_2$','Interpreter','latex','FontSize',12)
zlabel('Position $y_1$','Interpreter','latex','FontSize',12)
grid on
colormap(colorMap);
%lightangle(-45,30)

%lighting gouraud
%axis equal
subplot(2,2,4)
plot3(x_in(:,1),x_in(:,2),x_out(:,2),'Color',[0.8 0.8 0.8])
hold on
surf(XX1,XX2,x2_out_interp,'EdgeColor','None')
xlabel('Position $x_1$','Interpreter','latex','FontSize',12)
ylabel('Velocity $x_2$','Interpreter','latex','FontSize',12)
zlabel('Velocity $y_2$','Interpreter','latex','FontSize',12)
grid on
colormap(colorMap);
%axis equal
%colormap summer
%lighting gouraud


