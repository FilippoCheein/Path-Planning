% img=zeros(50,60);
% kRep=100;
% endPos=[20 30];
% kAttr=1E-3;

function Ffield=imgToF1(img,kRep,endPos,kAttr,rObj)
%     constants and initialization
    [yMax,xMax]=size(img);
    dField=bwdist(img==0);
    k=100;
    dFieldNaN=(dField/k)+1;
    
%     Repulse force at every pixel
    Frep=kRep*((1./dFieldNaN-1/rObj).^2);
    Frep(dFieldNaN>rObj)=0;
    max(Frep,[],'all');

%     Attractive force at every pixel
    
    [x,y]=meshgrid(1:xMax,1:yMax);
    Fattr=kAttr*((y-endPos(1)).^2+(x-endPos(2)).^2);
    
%     Combine
    Ffield=Fattr+Frep;
end