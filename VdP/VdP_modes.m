clear all
close all
clc
%%
load('Linearize_VdP.mat')


%%

ICs_train=[ 3*cos(phis_out) 0.1*cos(phis_in); 5*sin(phis_out) 0.1*sin(phis_in)];

[Rs,Phis]=ndgrid(linspace(0,5),linspace(0,2*pi));
X=Rs(:).*cos(Phis(:));
V=Rs(:).*sin(Phis(:));

Es=(X.^2/3+V.^2/25);
X(Es>1)=[];
V(Es>1)=[];
figure
plot(X,V,'x')

%%

A=[1 0; 0 0];

[U,~]=eig(A);
%U=[-1i.*U(:,1) U(:,2)];

U_inv=inv(U);



%%
N_steps=length(X);
phis=zeros(N_steps,2);
N_disp=floor(length(X)/100);



parfor iter_IC=1:N_steps
    chck=false;   
    x0_lin=net_nonlin2lin([X(iter_IC);V(iter_IC)]);%
    %IC_test_lin=[int1_nonlin2lin(IC_test(iter_IC,:)); int2_nonlin2lin(IC_test(iter_IC,:))];
    x0_lin=[sqrt(x0_lin(1)^2+x0_lin(2)^2); atan2(x0_lin(1),x0_lin(2))];
    
    phis(iter_IC,:)=U_inv*x0_lin;
    if floor(iter_IC/N_disp)*N_disp==iter_IC
        disp(['Progress: ' num2str(round(iter_IC/N_steps*100,2)) ' %'])
    end
end

%%

[X_grd,V_grd]=ndgrid(linspace(-3,3),linspace(-5,5));

figure

subplot(2,1,1)
phi_1=scatteredInterpolant(X,V,phis(:,1),'linear','none');
surf(X_grd,V_grd,phi_1(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter
xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\varphi_1$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,1,2)
phi_2=scatteredInterpolant(X,V,phis(:,2),'linear','none');
surf(X_grd,V_grd,phi_2(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\varphi_2$','Fontsize',12,'Interpreter','latex')
colorbar



