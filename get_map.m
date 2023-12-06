% Create a new 3D occupancy map with a resolution of 0.1
map3D = occupancyMap3D(10);
map3D.FreeThreshold = 0.2;

[xGround,yGround,zGround] = meshgrid(0:0.1:5,0:0.1:5,0:0.1:5);
xyzGround = [xGround(:) yGround(:) zGround(:)];
occval = 0;
setOccupancy(map3D,xyzGround,occval)
% Define the center of the obstacle
obstacleCenter = [1, 1, 1];

% Define the size of the unit cube (in meters)
cubeSize = [1, 1, 1];

% Calculate the corner of the cube
corner = obstacleCenter - cubeSize / 2;

%Set the occupancy of the cells corresponding to the unit cube
for x = corner(1):0.1:(corner(1) + cubeSize(1))
    for y = corner(2):0.1:(corner(2) + cubeSize(2))
        for z = corner(3):0.1:(corner(3) + cubeSize(3))
            setOccupancy(map3D, [x, y, z], 1);
        end
    end
end
inflate(map3D,0.2)

