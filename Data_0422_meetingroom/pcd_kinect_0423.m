%% Ke Wang 2017 0328 Rough Registrationg Using Hesai Lidar and Kinect
clc;clear;close all

ptCloud_kinect = pcread('/home/wk/Data_kinect/Scene_1_Large/0000_cloud.pcd');
ptCloud_kinect1 = pcread('/home/wk/Data_kinect/Scene_5/0004_cloud.pcd');
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
 m = 180;
 C1 = [cosd(m) 0 -sind(m) 0; ...
     0  1 0 0; ...
     sind(m)  0 cosd(m) 0; ...
     0  0 0 1];
D = [1 0 0 0; ...
     0 1 0 0; ...
     0  0 1 0; ...
     -0.04  0.37 0.41 1];
t0 = affine3d(D * C1 * C* B*A);
ptCloud_kinect = pctransform(ptCloud_kinect,t0);
ptCloud_kinect1 = pctransform(ptCloud_kinect1,t0);
pcshow(ptCloud_kinect);
figure;
pcshow(ptCloud_kinect1);
%%
ptCloud_Lidar = pcread('Data_0422/pcd_lidar_0422_1.pcd');
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
%%
%moving.Location = A(1:24050,:);
%moving.Color = moving.Color(1:24050,:);
%moving.cound = 24050
n = 8509;
move = pointCloud(A(1:n,:));
move.Color = moving.Color(1:n,:);
mvloc = move.Location;
mvcolor = move.Color;
index_ex = [];
for i = 1:n
    x0 = mvloc(i,1);
    y0 = mvloc(i,2);
    if x0 > 6
        index_ex = [index_ex,i];
    end
end
n_i = size(index_ex,2);
seq = 1:n;
for i = 1: n_i
    seq = seq(seq~=index_ex(i));
end
mvloc = mvloc(seq,:);
mvcolor = mvcolor(seq,:);
move = pointCloud(mvloc);
move.Color = mvcolor;
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
ptCloudScene1 = pcmerge_wk(ptCloud_Lidar, ptCloud_kinect1, mergeSize);
ptCloudScene2 = pcmerge_wk(ptCloud_Lidar, ptCloud_kinect, mergeSize);
h = figure; set(h,'color',[1,1,1]);pcshow(ptCloudScene)
h1 = figure; set(h1,'color',[1,1,1]);pcshow(ptCloudScene1)
h2 = figure; set(h2,'color',[1,1,1]);pcshow(ptCloudScene2)
pcwrite(ptCloudScene,'object3d.pcd','Encoding','ascii');

