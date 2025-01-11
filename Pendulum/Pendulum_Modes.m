load('Linearize_pendulum_data_new.mat')

%%
[X,V]=ndgrid(linspace(-pi,pi),linspace(-2,2));
X=X(:);
V=V(:);
Es=0.5.*V.^2-cos(X);
X(Es>1)=[];
V(Es>1)=[];
figure
plot(X,V,'x')
%%


A=[0 1; -1 0];

[U,~]=eig(A);
%U=[-1i.*U(:,1) U(:,2)];

U_inv=inv(U);
r_pendulum=sqrt(X.^2+V.^2);
phi_pendulum=mod(atan2(X,V)-pi/2,2*pi)+pi/2;

N_steps=length(r_pendulum);
phis=zeros(N_steps,2);
N_disp=floor(N_steps/100);

for iter=1:N_steps
    x0_lin=net_nonlin2lin([r_pendulum(iter);phi_pendulum(iter)]);
    x0_lin=[x0_lin(1)*sin(x0_lin(2)); x0_lin(1)*cos(x0_lin(2))];
    phis(iter,:)=U_inv*x0_lin;
    if floor(iter/N_disp)*N_disp==iter
        disp(['Progress: ' num2str(round(iter/N_steps*100,2)) ' %'])
    end
end
%%

cmap=[linspace(0,1, 100)', zeros(100, 1),fliplr(linspace(0, 1, 100))'];

figure
[X_grd,V_grd]=ndgrid(linspace(-pi,pi),linspace(-2,2));
subplot(2,2,1)
phi_abs_1=scatteredInterpolant(X,V,abs(phis(:,1)),'linear','none');
surf(X_grd,V_grd,phi_abs_1(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap(cmap)
%winter
xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$|\varphi_1|$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,2)
phi_angle_1=scatteredInterpolant(X,V,angle(phis(:,1)),'linear','none');
surf(X_grd,V_grd,phi_angle_1(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap(cmap)

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\angle \varphi_1$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,3)
phi_abs_2=scatteredInterpolant(X,V,abs(phis(:,2)),'linear','none');
surf(X_grd,V_grd,phi_abs_2(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap(cmap)

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$|\varphi_2|$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,4)
phi_angle_2=scatteredInterpolant(X,V,angle(phis(:,2)),'linear','none');
surf(X_grd,V_grd,phi_angle_2(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap(cmap)

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$ \angle \varphi_2$','Fontsize',12,'Interpreter','latex')
colorbar


%%
figure
[X_grd,V_grd]=ndgrid(linspace(-pi,pi),linspace(-2,2));
subplot(2,1,1)
phi_abs=scatteredInterpolant(X,V,abs(phis(:,1)).^2+abs(phis(:,2)).^2,'linear','none');
surf(X_grd,V_grd,phi_abs(X_grd,V_grd)./max(phi_abs(X_grd,V_grd)),'EdgeColor','none')
view(2)
colormap winter
xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\varphi_1^2+\varphi_2^2$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,1,2)
phi_angle=scatteredInterpolant(X,V,mod(angle(phis(:,1))-angle(phis(:,2)),2*pi),'linear','none');
surf(X_grd,V_grd,phi_angle(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter
xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\angle \varphi_1-\angle \varphi_2$','Fontsize',12,'Interpreter','latex')

colorbar


%%
