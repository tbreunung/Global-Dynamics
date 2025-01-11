load('Linearize_duff_data_new.mat')
    


%%

[X,V]=ndgrid(linspace(-2,2),linspace(-2,2));
X=X(:);
V=V(:);
Es=0.5.*V.^2-0.5*X.^2+0.25.*X.^4;
X(Es>2)=[];
V(Es>2)=[];
X3=sign(X);
Es=0.5.*V.^2-0.5*X.^2+0.25.*X.^4;
X3(Es>0)=0;
figure
plot3(X,V,X3,'x')

%%

A=[0 1; -1 0];

[U,~]=eig(A);
%U=[-1i.*U(:,1) U(:,2)];

U_inv=inv(U);

%%
%IC_pos_test=[0.95.*sort(rand(N_IC_test,1));  -0.95.*sort(rand(N_IC_test,1)); sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%IC_pos_test=1;%[(sqrt(2)-0.05).*sort(rand(N_IC_test,1));  -(sqrt(2)-0.05).*sort(rand(N_IC_test,1)); sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%IC_pos_test=1.7;%[sign(rand(N_IC_test,1)-0.5).*((2-sqrt(2)).*sort(rand(N_IC_test,1))+sqrt(2))];
%sign(rand(N_IC_test,1)-0.5).*

N_steps=length(X);
phis=zeros(N_steps,3);
N_disp=floor(length(X)/100);

parfor iter_IC=1:N_steps
    x0=[X(iter_IC);V(iter_IC)];
    
    r_duff=sqrt(X(iter_IC).^2+V(iter_IC).^2);
    
    if 0.5*V(iter_IC).^2-0.5*X(iter_IC).^2+0.25*X(iter_IC).^4<0
         
        phi_duff=atan2(X(iter_IC),V(iter_IC));
    else
         
        phi_duff=mod(atan2(X(iter_IC),V(iter_IC))-pi/2,2*pi)+pi/2;
    end
    
    
    x0_lin=net_nonlin2lin([r_duff;phi_duff; X3(iter_IC)]);
    x0_lin=[x0_lin(1)*sin(x0_lin(2)); x0_lin(1)*cos(x0_lin(2));x0_lin(3)];
    
    phis(iter_IC,:)=[U_inv*x0_lin(1:2);X3(iter_IC)];
    
    if floor(iter_IC/N_disp)*N_disp==iter_IC
        disp(['Progress: ' num2str(round(iter_IC/N_steps*100,2)) ' %'])
    end
    
    
end

%%
my_green=[143 209 41]/255;
my_blue=[7 149 214]/255;
my_pink=[218 112 214 ]/255;


[X_grd,V_grd]=ndgrid(linspace(-2,2),linspace(-2,2));


figure

subplot(2,2,1)
phi_abs_1=scatteredInterpolant(X,V,log(abs(phis(:,1))),'linear','none');
surf(X_grd,V_grd,phi_abs_1(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter
xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\ln(|\varphi_1|)$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,2)
phi_angle_1=scatteredInterpolant(X,V,angle(phis(:,1)),'linear','none');
surf(X_grd,V_grd,phi_angle_1(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\angle \varphi_1$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,3)
phi_abs_2=scatteredInterpolant(X,V,log(abs(phis(:,2))),'linear','none');
surf(X_grd,V_grd,phi_abs_2(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$\ln(|\varphi_2|)$','Fontsize',12,'Interpreter','latex')
colorbar

subplot(2,2,4)
phi_angle_2=scatteredInterpolant(X,V,angle(phis(:,2)),'linear','none');
surf(X_grd,V_grd,phi_angle_2(X_grd,V_grd),'EdgeColor','none')
view(2)
colormap winter

xlabel('Position','Fontsize',12,'Interpreter','latex')
ylabel('Velocity','Fontsize',12,'Interpreter','latex')
title('$ \angle \varphi_2$','Fontsize',12,'Interpreter','latex')
colorbar