function [out] = Algo18_GPAS(bRie,btie,qij,pattern,patternX,patternY,K)
% GPA hand-eye calibration for single markers
%
% Input:
% bRie (3x3xNumber_of_Views): the rotation matrix of robot pose from effector to base,
% btie (3xNumber_of_Views): the translation vector of robot pose  from effector to base (unit: m),
% qij (2xNumber_of_Views): the 2D pixel,
% K (3x3): the camera intrinsic.
%
% Output:
% eRc (3x3): the rotation part of hand-eye pose from camera to effector,
% etc (3x1): the translation part of hand-eye pose from camera to effecotor (unit: m),
% p (3x1): the marker positon in the base frame, 
% runtime (1x1):  the running time (unit: seconds).
%
% Reference: Jin, G., Yu, X., Chen, Y., Li, J. (2023), Simultaneous hand-eye and target parameter estimation
%            by solving 2d-3d generative point alignment problem, submitted to IEEE Trans. Instrum. Meas.
% E-mail: jingumin@sjtu.edu.cn

mark=tic;





if ndims(qij)==3
% qij=squeeze(qij(:,:,size(qij,3)/2));
if mod(patternX,2)==1 %奇数
    numU=patternX*patternY/2;
else
    numU=patternX*patternY/2+ceil(patternY/2);
end
qij=squeeze(qij(:,:,numU));
end
% Data preparation.
n=size(qij,2);
eRib=pagetranspose(bRie);
etib=-squeeze(pagemtimes(eRib,reshape(btie,3,1,n)));


rnti1=toc(mark);

% To start initialization, first recover the rotation.
C=zeros(2*n,39);
for i=1:n
    C(2*i-1:2*i,:)=kron([reshape(eRib(:,:,i),9,1)',[etib(:,i);1]'],([-eye(2),qij(:,i)]*K));
end
[~,~,V]=svd(C);
[Uh,~,Vh]=svd(reshape(V(28:36,39),3,3));
Rini=sign(det(Uh*Vh'))*Uh*Vh';

% Recover translation based on rotation.
D=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),...
    reshape(qij,2,1,n)),K),cat(2,pagemtimes(Rini,eRib),repmat(eye(3),[1,1,n]))));
d=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),...
    reshape(qij,2,1,n)),K),pagemtimes(Rini,reshape(etib,3,1,n))));
z=-inv(D'*D)*D'*d;
tini=z(4:6);

% Recover the marker position
pini=z(1:3);


% Refinement preparation.
Xk=[Rini,tini;0,0,0,1];
pk=pini;
Rk=Rini;
tk=tini;
k_num=1;
k_step=100;
iter_th=100;
step_th=1e-4;

while (step_th<k_step && k_num<iter_th)
    k_num=k_num+1;

    % Calculate the coefficients of the normal equation through Jacobi.
    alpha=reshape(pagemtimes(Rk,(pagemtimes(eRib,pk)+reshape(etib,3,1,n)))+tk,3,n);
    Ja_mat=[K(1,1)./alpha(3,:),zeros(1,n),-K(1,1).*alpha(1,:)./alpha(3,:).^2;...
        zeros(1,n),K(2,2)./alpha(3,:),-K(2,2).*alpha(2,:)./alpha(3,:).^2];
    Ja=pagetranspose(reshape(reshape(Ja_mat,2*n,3)',3,2,n));
    JaJX=cat(2,Ja,-pagemtimes(Ja,reshape([zeros(1,n);alpha(3,:);-alpha(2,:);...
        -alpha(3,:);zeros(1,n);alpha(1,:);...
        alpha(2,:);-alpha(1,:);zeros(1,n)],3,3,n)));
    JaJp=pagemtimes(pagemtimes(Ja,Rk),eRib);
    L=sum(pagemtimes(pagetranspose(JaJX),JaJX),3);
    M=squeeze(sum(reshape(pagemtimes(pagetranspose(JaJp),JaJX),3,6,n),3));
    N=squeeze(sum(reshape(pagemtimes(pagetranspose(JaJp),JaJp),3,3,n),3));%3x3xm
    f=K*alpha;
    qsubf=reshape((qij-f(1:2,:)./f(3,:)),2,1,n);
    oX=sum(pagemtimes(pagetranspose(JaJX),qsubf),3);
    op=sum(reshape(pagemtimes(pagetranspose(JaJp),qsubf),3,1,n),3);

    % Calculate the update by matrix inversion.
    delta=inv([L,M';M,N])*[oX;op];
    deltaX=delta(1:6,1);
    deltap=delta(7:9,1);

    % Update.
    Xk=expo(deltaX)*Xk;
    Rk=Xk(1:3,1:3);
    tk=Xk(1:3,4);
    pk=pk+deltap;
    k_step=norm(delta);

end
eXc=inv(Xk);
eRc=eXc(1:3,1:3);
etc=eXc(1:3,4);
p=pk;
rnti2=toc(mark);
out.eRc=eRc;
out.etc=etc;
out.p=p;
out.rnti2=rnti2;
out.rnti1=rnti1;
end

function s = expo(v)
%Exponential mapping
phi=v(1:3);rho=v(4:6);
rh=norm(rho,2);
L=cos(rh)*eye(3)+((1-cos(rh))/rh/rh)*rho*rho'+(sin(rh)/rh)*skewsys(rho);
l=(sin(rh)/rh)*phi+((rh-sin(rh))/rh/rh/rh)*rho*rho'*phi+((1-cos(rh))/rh/rh)*skewsys(rho)*phi;
s=[L,l;0,0,0,1];
end

function s = skewsys(v)
%Skew-symmetric mapping
s=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function mat=page2mat(page)
%Convert page to mat
mat=reshape(pagetranspose(page),size(page,2),size(page,1)*size(page,3))';
end