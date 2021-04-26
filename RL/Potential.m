%%% Outdated intitial RL code that has been superceded by timNN path
function Ufield = Potential(State,Ragent,kRep,kAttr,rObj,n)

    Operator = State(1);
    End = State(2);
    Obstacle = State(3);
    
    img = zeros(100,'logical');
    img(Obstacle(2),Obstacle(1)) = 1;
    [yMax,xMax]=size(img);
    
%     use meshgrid to create matrix representations of the row and column
%     vectors of x and y, sized off of our input image size
    [x,y]=meshgrid(1:xMax,1:yMax);
%     bwdist's output is the same as the inputted image, with each pixel
%     value being replaced with the distance to the closest non-zero pixel,
%     so in this case distance to the closest object
    dFieldObj=bwdist(img==0);
%     in order to overcome divide by zero issues, scale the distance field
%     and add one
    k=100;
    dFieldObj=(dFieldObj/k)+1;
    
%     calculate the n-th order euclidean distance from every point to the
%     goal, to be used in repulsive potential calculation
    if n~=0
        dFieldEnd=(abs(x-endPos(2)).^n+abs(y-endPos(1)).^n).^1/n;
    else
        dFieldEnd=1;
    end
    
%     calculate the repulsive potential at every point, based on the
%     distance field and how large an influence every object point has and
%     now also the n-th order euclidean distance to the goal
    Urep=kRep*((1./dFieldObj-1/rObj).^2).*dFieldEnd;
%     zero-out any point's distance who is larger than rObj
    Urep(dFieldObj>rObj)=0;

%     calculate the attractive potential at every point, based on the 
%     current co-ordinates and a parabaloid
%     generate a parabaloid with vertex/minimum being the end point
    Uattr=kAttr*((y-endPos(1)).^2+(x-endPos(2)).^2);
    
%     Add attractive and repulsive to get total potential at every point
    Ufield=Uattr+Urep;
end