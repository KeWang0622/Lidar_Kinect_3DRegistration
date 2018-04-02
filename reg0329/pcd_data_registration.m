%% Ke Wang 2017 0328 Rough Registrationg Using Hesai Lidar and Kinect
clc;clear;close all

ptCloud_kinect = pcread('output_0329.pcd');
figure;
A = [-1 0 0 0; ...
     0 -1 0 0; ...
     0  0 1 0; ...
     0  0 0 1];
B = [0 -1 0 0; ...
     1 0 0 0; ...
     0  0 1 0; ...
     0  0 0 1];
 t = 90;
 C = [1 0 0 0; ...
     0  cosd(t) -sind(t) 0; ...
     0  sind(t) cosd(t) 0; ...
     0  0 0 1];
 m = -40;
 C1 = [cosd(m) 0 -sind(m) 0; ...
     0  1 0 0; ...
     sind(m)  0 cosd(m) 0; ...
     0  0 0 1];
D = [1 0 0 0; ...
     0 1 0 0; ...
     0  0 1 0; ...
     -0.61  -0.08 0.61 1];
t0 = affine3d(D * C1 * C* B*A);
ptCloud_kinect = pctransform(ptCloud_kinect,t0);
pcshow(ptCloud_kinect);
%%
ptCloud_Lidar = pcread('pcd_lidar.pcd');
figure;
pcshow(ptCloud_Lidar);

gridSize = 0.02;
fixed = pcdownsample(ptCloud_Lidar, 'gridAverage', gridSize);
gridSize = 0.02;
moving = pcdownsample(ptCloud_kinect, 'gridAverage', gridSize);
figure;pcshow(fixed);


figure;pcshow(moving);


%% Processing
A = moving.Location;
figure(); plot(A(:,3));
%moving.Location = A(1:24050,:);
%moving.Color = moving.Color(1:24050,:);
%moving.cound = 24050
n = 15000;
move = pointCloud(A(1:n,:));
move.Color = moving.Color(1:n,:);
moving = move;
figure();
pcshow(moving);
%%
tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloud_kinect,tform);

figure;pcshow(ptCloudAligned);

%%
mergeSize = 0.005;
ptCloudScene = pcmerge_wk(ptCloud_Lidar, ptCloudAligned, mergeSize);

figure;pcshow(ptCloudScene)
pcwrite(ptCloudScene,'object3d.pcd','Encoding','ascii');
