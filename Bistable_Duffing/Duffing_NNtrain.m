clear all
close all
clc
%%
Duffing=@(t,x) [x(2); x(1)-x(1)^3 ];

Lin_osci=@(t,x) [x(2);-x(1);0];


N_IC_train=50;
IC_pos_train=[linspace(-1+0.05,-0.05,floor(N_IC_train/3))  linspace(+0.05,1-0.05,floor(N_IC_train/3)) linspace(sqrt(2)+0.05,2,N_IC_train-2*floor(N_IC_train/3))];
%IC_pos_train=[linspace(sqrt(2)+0.05,2,N_IC_train-2*floor(N_IC_train/3))];
my_green=[143 209 41]/255;
my_blue=[7 149 214]/255;
my_pink=[218 112 214 ]/255;
cmap=[repmat(my_green,floor(N_IC_train/3),1); repmat(my_blue,floor(N_IC_train/3),1);repmat(my_pink,N_IC_train-2*floor(N_IC_train/3),1)];

opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
tol=2*10^-3;
t_vec=0:0.001:300;
duff_data=[];
lin_osci_data=[];
T_duff=[];
fig_all=figure;
for iter_IC=1:N_IC_train
    
    x0=[IC_pos_train(iter_IC);0];
    
    [t, x] = ode45(@(t,x) Duffing(t,x), t_vec,x0,opts);
    figure(fig_all)
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

    
    rs_duff{iter_IC}=sqrt(x_per(:,1).^2+x_per(:,2).^2);
    phis_duff{iter_IC}=phis_tmp;
    T_per_duff{iter_IC}= repmat(t(length(x_per(:,1))),length(x_per(:,1)),1);
    
    T_duff=[T_duff; repmat(t(length(x_per(:,1))),length(x_per(:,1)),1)];
    if abs(IC_pos_train(iter_IC))<sqrt(2)
        Es=sign(x_per(:,1));
    else
        Es=0.*x_per(:,1);
    end
    duff_data=[duff_data;rs_duff{iter_IC}  phis_duff{iter_IC} Es];
    Es_duff{iter_IC}=Es;
    x0=[IC_pos_train(iter_IC);0; Es(1)];
    [t, x] = ode45(@(t,x) Lin_osci(t,x), t_vec ,x0,opts);
    %x_per=x;
    x_per=extract_period(x, tol);
    
    
    figure(fig_all)
    subplot(2,2,3)
    plot3(x_per(:,1),x_per(:,2),x_per(:,3),'Color',cmap(iter_IC,:))
    hold on
    subplot(2,2,4)
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
    Es_lin{iter_IC}=x_per(:,3);
    
    N=length(rs_duff{iter_IC});
    rs_lin{iter_IC}=interp1((0:length(rs_lin{iter_IC})-1)./length(rs_lin{iter_IC}).*2.*pi,rs_lin{iter_IC},(0:N-1)./N.*2.*pi).';
    phis_lin{iter_IC}=interp1((0:length(phis_lin{iter_IC})-1)./length(phis_lin{iter_IC}).*2.*pi,phis_lin{iter_IC},(0:N-1)./N.*2.*pi).';
    Es_lin{iter_IC}=interp1((0:length(Es_lin{iter_IC})-1)./length(Es_lin{iter_IC}).*2.*pi,Es_lin{iter_IC},(0:N-1)./N.*2.*pi).';
    lin_osci_data=[lin_osci_data;rs_lin{iter_IC}  phis_lin{iter_IC} Es_lin{iter_IC}];
    drawnow
end


subplot(2,2,3)
xlabel('$x_1$','Fontsize',12,'Interpreter','latex')
ylabel('$x_2$','Fontsize',12,'Interpreter','latex')
zlabel('$x_3$','Fontsize',12,'Interpreter','latex')
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
net_lin2nonlin = train(net_lin2nonlin,lin_osci_data.',duff_data.','useParallel','yes');
%%
net_nonlin2lin = feedforwardnet([40 40 40]);%

net_nonlin2lin.trainParam.showCommandLine= true;
net_nonlin2lin.trainParam.epochs=5000;
net_nonlin2lin = train(net_nonlin2lin,duff_data.',lin_osci_data.','useParallel','yes');

%%
net_T = feedforwardnet([20 20 20]);%

net_T.trainParam.showCommandLine= true;
net_T.trainParam.epochs=1000;

net_T= train(net_T,lin_osci_data.',T_duff.','useParallel','yes');
%%

for iter=1:N_IC_train
T_tmp(iter)=T_per_duff{iter}(1);
end
f_T_duff=griddedInterpolant(IC_pos_train,T_tmp);
%%
save('Linearize_duff_data_new')
    


%%

function x_per=extract_period(x, tol)
id_start=find( vecnorm((x-x(1,:)).')./norm(x(1,:))>tol,1);
id=find( vecnorm((x(id_start:end,:)-x(1,:)).')./norm(x(1,:))<tol,1);

x_per=x(1:id_start+id-1,:);
end
