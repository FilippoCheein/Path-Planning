function subset=subsetBounded(unbounded,pos,radius)
%     subset=subsetBounded(unbounded,pos,radius)
%     inputs:
%       unbounded=padded potential field
%       pos=center point of bounds, no co-ord translation needed
%       radius=radius of bounding box
%     outputs:
%       subset=bounded potential field, with padding

%     calculate un-shifted bounding box
    yLower=pos(1)-radius;
    yUpper=pos(1)+radius;
    xLower=pos(2)-radius;
    xUpper=pos(2)+radius;

%     shift bounding box due to padding of potential field
    yRange=yLower+4:yUpper+4;
    xRange=xLower+4:xUpper+4;
%     crop
    subset=unbounded(yRange,xRange);
end