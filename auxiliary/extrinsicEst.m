function [cRpList,ctpList] = extrinsicEst(qij,pattern,K)
cRpList=zeros(3,3,size(qij,2));
ctpList=zeros(3,size(qij,2));
for i=1:size(qij,2)
    imagePoints=squeeze(qij(:,i,:));  %2xm
    [cRpList(:,:,i),ctpList(:,i)] = estPNP(imagePoints,pattern,K);
end
end


function [cRp,ctp] = estPNP(imagePoints,worldPoints,K)
m=size(imagePoints,2);
fx=K(1,1);
fy=K(2,2);
cx=K(1,3);
cy=K(2,3);

Ahomo=zeros(2*m,9);
for j=1:m
    Ahomo(2*j-1:2*j,:)=[kron([worldPoints(1:2,j)',1],[1,0,-(imagePoints(1,j)/fx-cx/fx)]);...
        kron([worldPoints(1:2,j)',1],[0,1,-(imagePoints(2,j)/fy-cy/fy)])];
end

[~,~,V]=svd(Ahomo);
alpha=V(:,end);

Xh=[alpha(1),alpha(4),alpha(7);...
    alpha(2),alpha(5),alpha(8);...
    alpha(3),alpha(6),alpha(9)];

lamda1=1/norm(Xh(:,1));
lamda2=1/norm(Xh(:,2));
lamda=(lamda1+lamda2)/2;
if alpha(9)*lamda<0
    lamda=-lamda;
end
Xh=Xh*lamda;
Rh=[Xh(:,1),Xh(:,2),cross(Xh(:,1),Xh(:,2))];

tini=Xh(:,3);

[U,~,V]=svd(Rh);
if det(U*V')>0
    Rini=U*V';
else
    Rini=-U*V';
end

para.imagePoints=imagePoints;
para.worldPoints=worldPoints;
para.K=K;
para.x0=[dcm2rod(Rini)';tini];
options = optimoptions('lsqnonlin','Display','off','StepTolerance',1e-4,'MaxIterations',100,'Algorithm','levenberg-marquardt','ScaleProblem','jacobian');
[x,~,~,~,~] =lsqnonlin(@(x)loss_lm(x,para),para.x0,[],[],options);
cRp=rod2dcm([x(1),x(2),x(3)]);
ctp=[x(4);x(5);x(6);];
end

function [s] = homo(v)
s=v(1:end-1,1)/v(end);
end

function s = expo(v)
phi=v(1:3);rho=v(4:6);
rh=norm(rho,2);
L=cos(rh)*eye(3)+((1-cos(rh))/rh/rh)*rho*rho'+(sin(rh)/rh)*skewsys(rho);
l=(sin(rh)/rh)*phi+((rh-sin(rh))/rh/rh/rh)*rho*rho'*phi+((1-cos(rh))/rh/rh)*skewsys(rho)*phi;
s=[L,l;0,0,0,1];
end

function s = skewsys(v)
s=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function [cost]=loss_lm(x,para)
imageP=para.imagePoints;
Points=para.worldPoints;
inK=para.K;
m=size(imageP,2);
R_op=rod2dcm([x(1),x(2),x(3)]);
t_op=[x(4);x(5);x(6);];
cost=zeros(2*m,1);
for j=1:m
    p_temp=R_op*Points(:,j)+t_op;
    cost((j*2-1):(j*2),1)=imageP(:,j)-homo(inK*p_temp);
end
end


% function [rod] = dcm22rod(R)
% R=[R(1,1),R(1,2),R(1,3);...
%     R(2,1),R(2,2),R(2,3);...
%     R(3,1),R(3,2),R(3,3);];
% fi=acos((trace(R)-1)/2);
% if real(fi)<1e-10
%     skewrod=zeros(3);
% else
%     skewrod=(fi/2/sin(fi))*(R-R');
% end
% rod=[skewrod(3,2);skewrod(1,3);skewrod(2,1)];
% end
%
%
% function [out] = rod22dcm(rod)
% in=[rod(1);rod(2);rod(3)];
% sita=norm(in);
% if sita<1e-10
%     out=eye(3);
% else
%     in_st=in/sita;
%     out=cos(sita)*eye(3)+(1-cos(sita))*(in_st*in_st')+sin(sita)*skew(in_st);
% end
% end