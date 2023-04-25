function [out] = Algo17_GPAP(bRie,btie,qij,pattern,patternX,patternY,K)
% GPA hand-eye calibration for pattern
%
% Input:
% bRie (3x3xNumber_of_Views): the rotation matrix of robot pose from effector to base,
% btie (3xNumber_of_Views): the translation vector of robot pose  from effector to base (unit: m),
% qij (2xNumber_of_ViewsxNumber_of_Markers): the 2D pixel,
% K (3x3): the camera intrinsic,
% iniNum (1x1): the selected initialization number,
% pattern (3xNumber_of_Markers): the pattern position (unit: m).
%
% Output:
% eRc (3x3): the rotation part of hand-eye pose from camera to effector,
% etc (3x1): the translation part of hand-eye pose from camera to effecotor (unit: m),
% Ry (3x3): the rotation part of pattern pose from pattern to base,
% ty (3x1): the translation part of pattern pose from pattern to base (unit: m).
% runtime (1x1):  the running time (unit: seconds).
%
% Reference: Jin, G., Yu, X., Chen, Y., Li, J. (2023), Simultaneous hand-eye and target parameter estimation
%            by solving 2d-3d generative point alignment problem, submitted to IEEE Trans. Instrum. Meas.
% E-mail: jingumin@sjtu.edu.cn
% 20230331

mark=tic;
% Data preparation.
m=size(qij,3);

if mod(patternX,2)==1 %奇数
    numU=patternX*patternY/2;
else
    numU=patternX*patternY/2+ceil(patternY/2);
end
iniNum=numU;
n=size(qij,2);
eRib=pagetranspose(bRie);
etib=-squeeze(pagemtimes(eRib,reshape(btie,3,1,n)));%求bRie is 3x3xn and btie is 3xn

rnti1=toc(mark);
% To start initialization, first recover the rotation.
C=zeros(2*n,39);
for i=1:n
    C(2*i-1:2*i,:)=kron([reshape(eRib(:,:,i),9,1)',[etib(:,i);1]'],([-eye(2),qij(:,i,iniNum)]*K));
end
[~,~,V]=svd(C);
[Uh,~,Vh]=svd(reshape(V(28:36,end),3,3));
Rini=sign(det(Uh*Vh'))*Uh*Vh';

% Recover translation based on rotation.
D=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),...
    reshape(qij(:,:,iniNum),2,1,n)),K),cat(2,pagemtimes(Rini,eRib),repmat(eye(3),[1,1,n]))));
d=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),...
    reshape(qij(:,:,iniNum),2,1,n)),K),pagemtimes(Rini,reshape(etib,3,1,n))));
z=-inv(D'*D)*D'*d;
tini=z(4:6);

% Perform one iteration using a single-point data for better initialization.
alpha_init=reshape(pagemtimes(Rini,(pagemtimes(eRib,z(1:3))+reshape(etib,3,1,n)))+tini,3,n);
Ja_init_mat=[K(1,1)./alpha_init(3,:),zeros(1,n),-K(1,1).*alpha_init(1,:)./alpha_init(3,:).^2;...
    zeros(1,n),K(2,2)./alpha_init(3,:),-K(2,2).*alpha_init(2,:)./alpha_init(3,:).^2];
Ja_init=pagetranspose(reshape(reshape(Ja_init_mat,2*n,3)',3,2,n));
JaJX_init=cat(2,Ja_init,-pagemtimes(Ja_init,reshape([zeros(1,n);alpha_init(3,:);-alpha_init(2,:);...
    -alpha_init(3,:);zeros(1,n);alpha_init(1,:);...
    alpha_init(2,:);-alpha_init(1,:);zeros(1,n)],3,3,n)));
JaJp_init=pagemtimes(pagemtimes(Ja_init,Rini),eRib);
L_init=sum(pagemtimes(pagetranspose(JaJX_init),JaJX_init),3);
M_init=squeeze(sum(reshape(pagemtimes(pagetranspose(JaJp_init),JaJX_init),3,6,n),3));
N_init=squeeze(sum(reshape(pagemtimes(pagetranspose(JaJp_init),JaJp_init),3,3,n),3));%3x3xm
f_init=K*alpha_init;
qsubf_init=reshape((reshape(qij(:,:,iniNum),2,n)-f_init(1:2,:)./f_init(3,:)),2,1,n);
oX_init=sum(pagemtimes(pagetranspose(JaJX_init),qsubf_init),3);
op_init=sum(reshape(pagemtimes(pagetranspose(JaJp_init),qsubf_init),3,1,n),3);
delta_init=[L_init,M_init';M_init,N_init]\[oX_init;op_init];
deltaX=delta_init(1:6,1);
Xk=expo(deltaX)*[Rini,tini;0,0,0,1];
Rini=Xk(1:3,1:3);
tini=Xk(1:3,4);

% Recover marker positions
pini=zeros(3,m);
for j=1:m
    E=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),reshape(qij(:,:,j),2,1,n)),K*Rini),eRib));
    e=page2mat(pagemtimes(pagemtimes(cat(2,-repmat(eye(2),[1,1,n]),reshape(qij(:,:,j),2,1,n)),K),(pagemtimes(Rini,reshape(etib,3,1,n))+tini)));
    pini(:,j)=-inv(E'*E)*E'*e;
end

% Recover pattern pose Y.
mean_p=mean(pini,2);
mean_r=mean(pattern,2);
W=sum(pagemtimes(pini-mean_p,pagetranspose(pattern-mean_r)),3);
[Uw,~,Vw]=svd(W);
RY=Uw*diag([1,1,det(Uw)*det(Vw)])*Vw';
tY=mean_p-RY*mean_r;
Yini=[RY,tY;0,0,0,1];

% Refinement preparation.
Yk=Yini;
Rk=Rini;
tk=tini;
Xk=[Rk,tk;0,0,0,1];
k_num=1;
step=100;
iter_th=100;
step_th=1e-4;
while (step_th<step && k_num<iter_th)
    k_num=k_num+1;

    % Calculate the coefficients of the normal equation through Jacobi.
    Ri_nm=repmat(eRib,[1,1,m]);
    ti_nm=repmat(reshape(etib,3,1,n),[1,1,m]);
    pattern_nm=reshape(repmat(reshape(pattern,3,1,m),[1,n]),3,1,m*n);
    beta=reshape(pagemtimes(RY,pattern_nm)+tY,3,m*n);
    alpha=reshape(pagemtimes(Rk,(pagemtimes(Ri_nm,pagemtimes(RY,pattern_nm)+tY)+ ti_nm))+ tk,3,m*n);
    Ja_mat=[K(1,1)./alpha(3,:),zeros(1,m*n),-K(1,1).*alpha(1,:)./alpha(3,:).^2;...
        zeros(1,m*n),K(2,2)./alpha(3,:),-K(2,2).*alpha(2,:)./alpha(3,:).^2];
    Ja=pagetranspose(reshape(reshape(Ja_mat,2*m*n,3)',3,2,m*n));
    JaJX=cat(2,Ja,-pagemtimes(Ja,reshape([zeros(1,m*n);alpha(3,:);-alpha(2,:);...
        -alpha(3,:);zeros(1,m*n);alpha(1,:);...
        alpha(2,:);-alpha(1,:);zeros(1,m*n)],3,3,m*n)));
    JaJY=pagemtimes(pagemtimes(pagemtimes(Ja,Rk),Ri_nm), cat(2,repmat(eye(3),[1,1,m*n]),-reshape([zeros(1,m*n);beta(3,:);-beta(2,:);...
        -beta(3,:);zeros(1,m*n);beta(1,:);...
        beta(2,:);-beta(1,:);zeros(1,m*n)],3,3,m*n)));
    f=K*alpha;
    qsubf=reshape((reshape(qij,2,n*m)-f(1:2,:)./f(3,:)),2,1,m*n);
    L=sum(pagemtimes(pagetranspose(JaJX),JaJX),3);
    M=sum(pagemtimes(pagetranspose(JaJY),JaJX),3);
    N=sum(pagemtimes(pagetranspose(JaJY),JaJY),3);
    oX=sum(pagemtimes(pagetranspose(JaJX),qsubf),3);
    oY=sum(pagemtimes(pagetranspose(JaJY),qsubf),3);

    % Calculate the update by matrix inversion.
    A=[L,M';M,N];
    a=[oX;oY];
    delta=inv(A'*A)*A'*a;
    deltaX=delta(1:6,1);
    deltaY=delta(7:end,1);

    % Update.
    Xk=expo(deltaX)*Xk;
    Yk=expo(deltaY)*Yk;
    Rk=Xk(1:3,1:3);
    tk=Xk(1:3,4);
    RY=Yk(1:3,1:3);
    tY=Yk(1:3,4);
    step=norm(delta);
end
Ry=Yk(1:3,1:3);
ty=Yk(1:3,4);
eXc=inv(Xk);
eRc=eXc(1:3,1:3);
etc=eXc(1:3,4);
rnti2=toc(mark);
out.eRc=eRc;
out.etc=etc;
out.Ry=Ry;
out.ty=ty;
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