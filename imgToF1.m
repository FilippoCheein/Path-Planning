function Ffield=imgToF1(img,kRep,endPos,kAttr,rObj)
%     Ffield=imgToF1(img,kRep,endPos,kAttr,rObj)
%     inputs:
%       img=grayscale image representation of 2D-environment
%         all black pixels (I = 0) are obstacles
%       kRep=scaling constant for repulsive (obstacle) force
%       endPos=row vector containing the desired goal point
%         endPos goes [y x] NOT [x y]
%       kAttr=scaling constant for attractive (goal) force
%       rObj=the radius of influence for every given object pixel
%     outputs:
%       Ffield=combined field map

%     constants and initialization
    [yMax,xMax]=size(img);
%     bwdist's output is the same as the inputted image, with each pixel
%     value being replaced with the distance to the closest non-zero pixel,
%     so in this case distance to the closest object
    dField=bwdist(img==0);
%     in order to overcome divide by zero issues, scale the distance field
%     and add one
    k=100;
    dFieldNaN=(dField/k)+1;
    
%     calculate the repulsive force at every point, based on the distance
%     field and how large an influence every object point has
    Frep=kRep*((1./dFieldNaN-1/rObj).^2);
%     zero-out any point's distance who is larger than rObj
    Frep(dFieldNaN>rObj)=0;

%     calculate the attractive force at every point, based on the current
%     co-ordinates and a parabaloid
%     use meshgrid to create matrix representations of the row and column
%     vectors of x and y, sized off of our input image size
    [x,y]=meshgrid(1:xMax,1:yMax);
%     generate a parabaloid with vertex/minimum being the 
    Fattr=kAttr*((y-endPos(1)).^2+(x-endPos(2)).^2);
    
%     Add attractive and repulsive to get total force at every point
    Ffield=Fattr+Frep;
end