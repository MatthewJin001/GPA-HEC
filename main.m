
clc;clear;

currentFolder = pwd;
addpath ([currentFolder,'\comparison\'])
addpath ([currentFolder,'\dataset\'])
addpath ([currentFolder,'\auxiliary\'])
addpath ([currentFolder,'\method\'])

flagTabbAli=true;
foldname='dataTabb';
load([foldname,'.mat'])

% numUse=100;
% ppi=ppi(:,1:numUse,:);
% Ri=Ri(:,:,1:numUse);
% ti=ti(:,1:numUse);

if flagTabbAli
    RiEval=Ri;tiEval=ti;ppiEval=ppi;KEval=K;patternEval=pattern;
else
    load dataEval.mat;
    K=KEval;
end

Method={

        @Algo1_Tsai;...       %1989
        @Algo2_Park;...       %1994
        @Algo3_Horaud;...     %1995
        @Algo4_Liang;...      %2008
        @Algo5_Li;...         %2010
        @Algo6_Shah;...       %2013
        @Algo7_TabbZ1;...     %2017
        @Algo8_TabbZ2;...    %2017
        @Algo9_TabbR;...     %2017
        @Algo10_AliX1;...     %2019
        @Algo11_AliX2;...     %2019
        @Algo12_AliR1;...     %2019
        @Algo13_AliR2;...     %2019
        @Algo14_Zhao;...      %2019
        @Algo15_Wu;...       %2019    
        @Algo16_Sarabandi;...%2022
        @Algo17_GPAS;   % this paper
        @Algo18_GPAP;   % this paper
        @Algo19_GPAM;}; % this paper



numAlg=size(Method,1);
RAlg=zeros(3,3,numAlg);
tAlg=zeros(3,numAlg);
timeAlg=zeros(1,numAlg);
timeDAlg=zeros(1,numAlg);
ProjAlg=zeros(1,numAlg);
RecAlg=zeros(1,numAlg);

for idx=1:1e8
    ;
end

%Calibration
for idxAlg=1:numAlg
    [out] = Method{idxAlg}(Ri,ti,ppi,pattern,patternX,patternY,K);
    RAlg(:,:,idxAlg)=out.eRc;
    tAlg(:,idxAlg)=out.etc;
    timeDAlg(idxAlg)=out.rnti1;
    timeAlg(idxAlg)=out.rnti2;
    
end




%Evaluation
for idxAlg=1:numAlg
    [ProjAlg(idxAlg),RecAlg(idxAlg)] = evaProj(RiEval,tiEval,ppiEval,KEval,patternEval,RAlg(:,:,idxAlg),tAlg(:,idxAlg),false);
end

%Show and save the results
Time=timeAlg';
TimeD=timeDAlg';
Proj=ProjAlg';
Rec=RecAlg';
tAlg=tAlg*1000;tx=tAlg(1,:)';ty=tAlg(2,:)';tz=tAlg(3,:)';
Rx=zeros(numAlg,1);Ry=zeros(numAlg,1);Rz=zeros(numAlg,1);
for idxAlg=1:numAlg
    [Rx(idxAlg),Ry(idxAlg),Rz(idxAlg)]=dcm2angle(RAlg(:,:,idxAlg));
end
Rx=Rx*57.3;Ry=Ry*57.3;Rz=Rz*57.3;
for idxAlg=1:numAlg
    strList=strsplit(func2str(Method{idxAlg}),'_');
    Method{idxAlg}=strList{2};
end
range1='A1';gap=2;range2=['A',num2str(numAlg+1+1+gap)];
T = table(Method,tx,ty,tz,Rx,Ry,Rz);
writetable(T,'result.xlsx','Sheet',foldname,'Range',range1)
T = table(Method,Time,TimeD,Proj,Rec)
writetable(T,'result.xlsx','Sheet',foldname,'Range',range2)





