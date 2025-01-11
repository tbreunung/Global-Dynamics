load('Linearize_duff_data_new.mat')
    


%%
N_IC_test=50;
 
IC_pos_test=linspace(sqrt(2),0,N_IC_test+2);
IC_pos_test(1)=[];
IC_pos_test(end)=[];
IC_vel_test=zeros(N_IC_test,1);
%%


t_test=[0:0.01:50];
T_duffing_test=zeros(N_IC_test,1);

T_linear_test=zeros(N_IC_test,1);

%par
for iter_IC=1:N_IC_test%
    x0=[IC_pos_test(iter_IC);IC_vel_test(iter_IC)];
    
    [~, x] = ode45(@(t,x) Duffing(t,x), t_test,x0,opts);
    idx=find(diff(sign(x(:,2)))==2)+1;
    T_duffing_test(iter_IC)=mean(diff(t_test(idx)));
    
    
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
    
    T_linear_test(iter_IC)=net_T([rs_lin_test(1) mod(phis_lin_test(1)-phis0,2*pi)+phis0  x(1,3)].').';

    %phis_lin_test=(phis_lin_test-phis0)./T_pendulum_test.*2*pi+phis0;
     
end

%%
my_green=[143 209 41]/255;
my_blue=[7 149 214]/255;
my_pink=[218 112 214 ]/255;
cmap=[repmat(my_green,N_IC_test,1); repmat(my_blue,N_IC_test,1);repmat(my_pink,N_IC_test,1)];

figure

plot(-IC_pos_test.^2/2+IC_pos_test.^4/4,2*pi./T_duffing_test,'Linewidth',2,'Color',my_green)
hold on
plot(-IC_pos_test.^2/2+IC_pos_test.^4/4,2*pi./T_linear_test,'Linewidth',2,'Color',my_blue)
 
grid on
xlabel('Hamiltonian Energy $E=\dot{x}^2/2-x^2/2+x^4/4$','Fontsize',12,'Interpreter','latex')
ylabel('Frequency','Fontsize',12,'Interpreter','latex')
lg=legend('Bistable Duffing oscillator','Transformed Linear Oscillator');
set(lg,'Fontsize',12,'Interpreter','latex')

%%

