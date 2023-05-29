clear;
clc;
addpath('PLY/')
tic
%Load all images
pc0 = pcread('D0.ply');
pc1 = pcread('D1.ply');
pc2 = pcread('D2.ply');
pc3 = pcread('D3.ply');
pc4 = pcread('D4.ply');

figure
pcshow(pc1);
axis on;
axis equal;
title('Point Cloud Registration 1', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');

figure
pcshow(pc2);
axis on;
axis equal;
title('Point Cloud Registration 2', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');

figure
pcshow(pc4);
axis on;
axis equal;
title('Point Cloud Registrated', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');
%% Registration 1
[~,ptCloud] = pcregistericp(pc1,pc0,Metric="planeToPlane");
%% Registration 2
[~,ptCloud2] = pcregistericp(pc2,ptCloud,Metric="planeToPlane");
%% Registration 3
[~,ptCloud3] = pcregistericp(pc3,ptCloud2,Metric="planeToPlane");
%% Registration 4
[~,ptCloud4] = pcregistericp(pc4,ptCloud3,Metric="planeToPlane");
%% Denoise

ptCloud5 = pcdenoise(ptCloud4);

figure 
pcshow(ptCloud4);
axis on;
axis equal;
title('Noisy Point Cloud', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');

figure 
pcshow(pc4);
axis on;
axis equal;
title('Registered Point Cloud', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');


%% Visualize

figure;
pcshow(ptCloud5,ColorSource="Z")
axis on;
axis equal;
title('Raw Point Cloud Data of Block A', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');

%% 
% Fit a line through the data.
x = ptCloud5.Location(:,1);
y = ptCloud5.Location(:,2);
z = ptCloud5.Location(:,3);

xyz = ptCloud5.Location;
centroid = mean(xyz);

% Subtract the centroid from the point cloud
centered_points = xyz - centroid;

% Compute the covariance matrix of the point cloud
cov_matrix = cov(centered_points);

% Compute the eigenvectors and eigenvalues of the covariance matrix
[eig_vectors, eig_values] = eig(cov_matrix);

% Extract the eigenvectors corresponding to the three largest eigenvalues
[~, sort_idx] = sort(diag(eig_values), 'descend');
eig_vectors = eig_vectors(:,sort_idx);

% Compute the rotation angles around the X, Y, and Z axes
angle_x = double(atan2d(eig_vectors(2,1), eig_vectors(1,1)));
angle_y = double(atan2d(eig_vectors(3,1), sqrt(eig_vectors(3,2)^2 + eig_vectors(3,3)^2)));
angle_z = double(atan2d(eig_vectors(3,2), eig_vectors(3,3)));

% Compute the rotation matrix
rot_x = makehgtform('xrotate', deg2rad(angle_x));
rot_y = makehgtform('yrotate', deg2rad(angle_y));
rot_z = makehgtform('zrotate', deg2rad(angle_z));
rot_matrix = rot_z(1:3, 1:3) * rot_y(1:3, 1:3) * rot_x(1:3, 1:3); %

% % Compute the rotation matrix to align the point cloud with its XY plane
% normal = eig_vectors%(:,1); % Change this
% angle2 = atan2d(normal(3), normal(2)); % and this
% rota = makehgtform('xrotate', deg2rad(-angle2));
% rot_matrix2 = rota(1:3, 1:3);


% Apply the rotation to the point cloud
rotated_points1 = (centered_points * rot_matrix') %+ centroid;
%rotated_points2 = (centered_points * rot_matrix' * rot_matrix2') %+ centroid;

% coefficients1 = polyfit(x, y, 1);
% coefficients2 = polyfit(x, z, 1);
% coefficients3 = polyfit(y, z, 1);
% 
% fittedy = polyval(coefficients1, x);
% fittedz = polyval(coefficients2, x);
% fittedx = polyval(coefficients3, y);
% 
% % Then subtract the fitted values and add the vertical offset.
% rotatedy = y - fittedy;
% rotatedz = z - fittedz;
% rotatedx = x - fittedz;
%% 

points1 =[rotated_points1(:,1), rotated_points1(:,2), rotated_points1(:,3)];
newpc1 = pointCloud(points1)

% points2 =[rotated_points2(:,1), rotated_points2(:,2), rotated_points2(:,3)];
% newpc2 = pointCloud(points2)

figure
pcshow(newpc1)
axis on;
xlabel('x');
ylabel('y');
zlabel('z');
view(0,90)

% figure
% pcshow(newpc2)
% axis on;
% xlabel('x');
% ylabel('y');
% zlabel('z');
% view(0,90)

%% Second rotation

% model = pcfitplane(newpc1, 0.01, [1 1 0]);
% normal_vector = model.Normal;
% ang = rad2deg(atan(dot(normvec,%[0 0 0])));
ang = deg2rad(11);
rotation_matrix = [1 0 0;
                   0 cos(ang) -sin(ang);
                   0 sin(ang) cos(ang)];
rotated_point_cloud = newpc1.Location * rotation_matrix;
pc = pointCloud(rotated_point_cloud)
figure
pcshow(pc)
axis on;
xlabel('x');
ylabel('y');
zlabel('z');

%% Removing unwanted object

xmin = pc.XLimits(1);
xmax = 100;
ymin = -60;
ymax = 24;
zmin = pc.ZLimits(1);
zmax = pc.ZLimits(2);

roi= [xmin xmax ymin ymax zmin zmax];
indices = findPointsInROI(pc, roi);
ROI = select(pc,indices);
figure
pcshow(ROI)
axis on;
axis equal;
title('Adjusted Point Cloud Data of Block A', 'Color','black');
set(gca, 'color', 'white');
set(gcf, 'color', 'white');
ax = gca;
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
xlabel('x');
ylabel('y');
zlabel('z');
view(0,90)
%% Filtering



%% Convert to mesh

mesh = pc2surfacemesh(ROI, "poisson", 5)
mesh2 = pc2surfacemesh(ROI, "poisson", 3)
mesh3 = pc2surfacemesh(ROI, "poisson", 8)

%% Show mesh

surfaceMeshShow(mesh, Title="Surface Mesh of Block A with Depth 5", BackgroundColor="white" )
surfaceMeshShow(mesh2, Title="Surface Mesh of Block A with Depth 3", BackgroundColor="white" )
surfaceMeshShow(mesh3, Title="Surface Mesh of Block A with Depth 7.5", BackgroundColor="white" )

%% Patch obj
figure 
patch('Faces', mesh.Faces, 'Vertices', mesh.Vertices)
axis equal
title('Surface Mesh of Block A with Depth 5')
colorbar;
xlabel('X');
ylabel('Y');
zlabel('Z');


%% Transform mesh into origin position
faces = double(mesh.Faces);
vertices = double(mesh.Vertices);

%Translation vector
minX = min(vertices(:,1));
minY = min(vertices(:,2));
minZ = min(vertices(:,3));
centroid = mean(vertices);
translationVec = -[minX, minY, (minZ + 7.5)];

% Apply the translation to the mesh vertices and faces
vertices2 = repmat(translationVec, size(vertices,1), 1) + vertices;

figure 
patch('Faces', faces, 'Vertices', vertices, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
grid on

figure
patch('Faces', faces, 'Vertices', vertices2, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
grid on


%% Create uniform spaced points on surface mesh

xm = vertices2(:,1);
ym = vertices2(:,2);
zm = vertices2(:,3);

% Create scattered interpolant object
F = scatteredInterpolant(xm, ym, zm, 'natural');

% Define the resolution for the grid
grid_resolution = 5;

% Create a grid in the x-y plane
x_range = min(xm):grid_resolution:max(xm);
y_range = min(ym):grid_resolution:max(ym);
[x_grid, y_grid] = meshgrid(x_range, y_range);

% Interpolate z-coordinates of the grid points
z_grid = F(x_grid, y_grid);


% Visualize the original surface mesh and the generated grid points
figure;
hold on;
patch('Faces', faces, 'Vertices', vertices2, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
scatter3(x_grid(:), y_grid(:), z_grid(:), 'r', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
title('Uniformly spaced grid-like points on the surface');
hold off;


%% Waypoints

% Convert the x_grid, y_grid, and z_grid matrices into column vectors
x_col = reshape(x_grid, [], 1);
y_col = reshape(y_grid, [], 1);
z_col = reshape(z_grid, [], 1);

% Concatenate the column vectors horizontally to create a single matrix
waypoints = [x_col, y_col, z_col];

figure
hold on
axis equal
axis on
xlabel('X')
ylabel('Y')
zlabel('Z')
scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 10, 'r', 'filled');
hold off

%% 2D Waypoints for Validation
% Sort the points by y-coordinate
sortedPoints = sortrows(waypoints, 2);

% Select 2D coordinates
ind = waypoints(:,2) == 5;
points2D = waypoints(ind, [1,2,3]);

figure
scatter3(points2D(:,1),points2D(:,2),points2D(:,3), 'r', 'filled')

%% Compare to actual function
x = 1:200;
y = 1;
[X,Y] = meshgrid(x,5,y);
Z = zeros(size(X));
for i = 1:length(x)
    if  x(i) >= 25 & x(i) <= 175     
        Z(:,i) = 7.5*sin((2*pi/100)*X(:,i) - pi/2);
    else
        Z(:,i) = 0 ; 
    end
end

figure
plot3(X,Y,Z)
hold on
scatter3(points2D(:,1),points2D(:,2),points2D(:,3), 'r', 'filled')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('');
hold off

%% Evaluating coordinates to its actual function
xv = linspace(0, size(waypoints, 1));
z_actual = 7.5*sin((2*pi/100)*xv - pi/2);
z_actual = z_actual'
z_actual_evaluated = 7.5*sin((2*pi/100)*waypoints(:, 1) - pi/2);
z_diff = z_actual_evaluated - waypoints(:, 3);

figure;
plot(xv, z_actual,'r', waypoints(:, 1), waypoints(:, 3), 'ro');
xlabel('x');
ylabel('z');
title('Comparison of actual and generated values');

%% Create surface mesh from actual geometry
% Define the x and y axes
x = 1:200;
y = 1:100;
[X,Y] = meshgrid(x,y);
% Evaluate z values based on two functions
Z = zeros(size(X));
%Define range of Z
for i = 1:length(x)
    if  x(i) >= 25 & x(i) <= 175     
        Z(:,i) = 7.5*sin((2*pi/100)*X(:,i) - pi/2);
    else
        Z(:,i) = 0 ; 
    end
end

figure
hold on
surf(X,Y,Z)
patch('Faces', faces, 'Vertices', vertices, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
axis on
axis equal
xlabel('x');
ylabel('y');
zlabel('z')
hold off

elapsedTime=toc;
disp(['Elapsed time; ' num2str(elapsedTime) ' seconds'])

% %% Compute error metric for Z axis
% 
% pcmesh = pointCloud(mesh.Vertices);
% pcactual = pointCloud([X(:), Y(:), Z(:)]);
% 
% % Perform the registration (alignment) between the two point clouds
% tform = pcregrigid(pcmesh, pcactual, 'Verbose', false, 'Extrapolate', true);
% 
% % Transform the reconstructed point cloud using the registration result
% alignedpc = pctransform(pcmesh, tform);
% 
% % Compute the error metric (e.g., RMSE) between the transformed reconstructed point cloud and the actual sinusoidal surface point cloud
% distances = pdist2(alignedpc.Location, pcactual.Location, 'euclidean', 'Smallest', 1);
% rmse = sqrt(mean(distances.^2));
% 
% % Display the results
% fprintf('Root Mean Squared Error: %f\n', rmse);
% %% Compute error metric for Z axis
% 
% pcmesh = pointCloud(mesh.Vertices);
% pcactual = pointCloud([X(:), Y(:), Z(:)]);
% 
% % Perform the registration (alignment) between the two point clouds
% tform = pcregrigid(pcmesh, pcactual, 'Verbose', false, 'Extrapolate', true);
% 
% % Transform the reconstructed point cloud using the registration result
% alignedpc = pctransform(pcmesh, tform);
% 
% % Compute the error metric (e.g., RMSE) between the transformed reconstructed point cloud and the actual sinusoidal surface point cloud
% distances = pdist2(alignedpc.Location, pcactual.Location, 'euclidean', 'Smallest', 1);
% rmse = sqrt(mean(distances.^2));
% 
% % Display the results
% fprintf('Root Mean Squared Error: %f\n', rmse);
% 
% %% Compute error metric for Z axis
% 
% save("waypoints_D.m","waypoints")
% writematrix(waypoints,'waypointsD')