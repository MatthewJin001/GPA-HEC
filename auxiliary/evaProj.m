function [resultProj,resultRec2] = evaProj(bRie,btie,qij,K,pattern,result_eRc,result_etc,debug)

m=size(qij,3);
n=size(qij,2);
eRib=pagetranspose(bRie);
etib=-squeeze(pagemtimes(eRib,reshape(btie,3,1,n)));
cTe=inv([result_eRc,result_etc;0,0,0,1]);
RX=cTe(1:3,1:3);
tX=cTe(1:3,4);

if debug
    disp('Initialization begins.');
end

Rini=RX;
tini=tX;
pini=zeros(3,m);
H=zeros(2*n,3);h=zeros(2*n,1);
for j=1:m
    for i=1:n
        H(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*Rini*eRib(:,:,i);
        h(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*(Rini*etib(:,i)+tini);
    end
    pini(:,j)=-inv(H'*H)*H'*h;
end

%求解 pj=Yrj
W=zeros(3,3);
mean_p=mean(pini,2);
mean_r=mean(pattern,2);
for j=1:m
    W=W+(pini(:,j)-mean_p)*(pattern(:,j)-mean_r)';
end
[Uw Sw Vw]=svd(W);
RY=Uw*diag([1,1,det(Uw)*det(Vw)])*Vw';
tY=mean_p-RY*mean_r;
% Yini=[RY,tY;0,0,0,1];


if debug
    disp('Initialization finishs.');
end
if debug
    disp('Solve Y begins.');
end

para.qij=qij;
para.Ri=eRib;
para.ti=etib;
para.RX=RX;
para.tX=tX;
para.pattern=pattern;
para.K=K;
para.Y0=[dcm2rod(RY)';tY];
options = optimoptions('lsqnonlin','Display','off','StepTolerance',1e-4,'MaxIterations',100,'Algorithm','levenberg-marquardt');
[x,resnorm,residual,exitflag,output] =lsqnonlin(@(x)lossY(x,para),para.Y0,[],[],options);
RY=rod2dcm([x(1),x(2),x(3)]);
tY=[x(4);x(5);x(6);];

if debug
    disp('Solve Y finishs.');
end

resultProj=0;
for j=1:m
    for i=1:n
        temp_p1=RY*pattern(:,j)+tY;
        temp_p2=RX*(eRib(:,:,i)*temp_p1+etib(:,i))+tX;
        resultProj=resultProj+norm([qij(:,i,j)-homo(K*temp_p2)])^2; 
    end
end
resultProj=sqrt(resultProj/m/n);


if debug
    disp('Solve p begins.');
end

% para.qij=qij;
% para.Ri=eRib;
% para.ti=etib;
% para.RX=RX;
% para.tX=tX;
% para.RY=RY;
% para.tY=tY;
% para.K=K;
% para.p0=reshape(pattern,[],1);
% options = optimoptions('lsqnonlin','Display','iter','StepTolerance',1e-9,'MaxIterations',100,'Algorithm','levenberg-marquardt');
% [x,resnorm,residual,exitflag,output] =lsqnonlin(@(x)loss_p(x,para),para.p0,[],[],options);
% rec_p=reshape(x,3,[]);
% rec_p(:,1:10)
% if debug
%     disp('Solve p finishs.');
% end
% 
% resultRec=0;
% for j=1:m
%         resultRec=resultRec+norm([pattern(:,j)-rec_p(:,j)])^2; 
% end
% resultRec=sqrt(resultRec/m);


% rec_p2=zeros(3,m);
% H=zeros(2*n,3);h=zeros(2*n,1);
% for j=1:m
%     for i=1:n
%         H(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*RX*eRib(:,:,i)*RY;
%         h(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*(RX*eRib(:,:,i)*tY+RX*etib(:,i)+tX);
%     end
%     rec_p2(:,j)=-inv(H'*H)*H'*h;
% end
% 
% if debug
%     disp('Solve p2 finishs.');
% end
% 
% resultRec2=0;
% for j=1:m
%         resultRec2=resultRec2+norm([pattern(:,j)-rec_p2(:,j)])^2; 
% end
% resultRec2=sqrt(resultRec2/m)*1000;



rec_p2=zeros(3,m);
H=zeros(2*n,3);h=zeros(2*n,1);
for j=1:m
    for i=1:n
        H(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*RX*eRib(:,:,i);
        h(2*i-1:2*i,:)=[-eye(2) qij(:,i,j)]*K*(RX*etib(:,i)+tX);
    end
    rec_p2(:,j)=-inv(H'*H)*H'*h;
end

if debug
    disp('Solve p2 finishs.');
end

resultRec2=0;
for j=1:m
    for i=1:n        
        temp_p2=RX*(eRib(:,:,i)*rec_p2(:,j)+etib(:,i))+tX;
        resultRec2=resultRec2+norm([qij(:,i,j)-homo(K*temp_p2)])^2; 
    end
end
resultRec2=sqrt(resultRec2/m/n);

end









function cost=loss_p(x,para)
K=para.K;
qij=para.qij;
Ri=para.Ri;
ti=para.ti;
RX=para.RX;
tX=para.tX;
RY=para.RY;
tY=para.tY;
m=size(qij,3);
n=size(qij,2);

op_p=reshape(x,3,m);

cost=zeros(2*m*n,1);
for j=1:m
    for i=1:n
        temp_p1=RY*op_p(:,j)+tY;
        temp_p2=RX*(Ri(:,:,i)*temp_p1+ti(:,i))+tX;
        cost((2*n*j-2*n+2*i-1):(2*n*j-2*n+2*i),1)=qij(:,i,j)-homo(K*temp_p2); 
    end
end

end


function cost=lossY(x,para)
K=para.K;
qij=para.qij;
Ri=para.Ri;
ti=para.ti;
RX=para.RX;
tX=para.tX;
pattern=para.pattern;
m=size(qij,3);
n=size(qij,2);

opRY=rod2dcm([x(1),x(2),x(3)]);
optY=[x(4);x(5);x(6);];

cost=zeros(2*m*n,1);
for j=1:m
    for i=1:n
        temp_p1=opRY*pattern(:,j)+optY;
        temp_p2=RX*(Ri(:,:,i)*temp_p1+ti(:,i))+tX;
        cost((2*n*j-2*n+2*i-1):(2*n*j-2*n+2*i),1)=qij(:,i,j)-homo(K*temp_p2); 
    end
end

end

function s = homo(v)
s=v(1:end-1,1)/v(end);
end
