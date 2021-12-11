function c = cuboid(size, position, orientation)
c =  [-size(1)/ 2, -size(2) / 2, -size(3) / 2;
    -size(1) / 2, -size(2) / 2, size(3) / 2;
    -size(1) / 2, +size(2) / 2, -size(3) / 2;
    -size(1) / 2, +size(2) / 2, size(3) / 2;
    size(1) / 2, -size(2) / 2, -size(3) / 2;
    size(1) / 2, -size(2) / 2, size(3) / 2;
    size(1) / 2, +size(2) / 2, -size(3) / 2;
    size(1) / 2, +size(2) / 2, size(3) / 2];
for i = 1:length(c)
    c(i,:) = quatrotate([orientation(end), orientation(1:3)],c(i,:)) + position;
end
end