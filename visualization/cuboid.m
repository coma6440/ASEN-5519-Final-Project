function c = cuboid(size, position, orientation)
c =  [-size(1)/ 2, -size(2) / 2, -size(3) / 2;
    -size(1) / 2, -size(2) / 2, size(3) / 2;
    -size(1) / 2, +size(2) / 2, -size(3) / 2;
    -size(1) / 2, +size(2) / 2, size(3) / 2;
    size(1) / 2, -size(2) / 2, -size(3) / 2;
    size(1) / 2, -size(2) / 2, size(3) / 2;
    size(1) / 2, +size(2) / 2, -size(3) / 2;
    size(1) / 2, +size(2) / 2, size(3) / 2];
    q = quaternion(orientation([4,1:3]));
    for i = 1:length(c)
        c(i,:) = rotatepoint(q, c(i,:)) + position;
    end
%     c = quatrotate(orientation([4,1:3]),c) + position;
end