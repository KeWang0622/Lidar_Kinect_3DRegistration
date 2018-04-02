%% Set a threshold to filter the point of a higer z value.
A = moving.Location;
figure(); plot(A(:,3));
moving.Location = A(1:24050,:);
moving.Color = moving.Color(1:24050,:);
moving.cound = 24050
move = pointCloud(A(1:24050,:));
move.Color = moving.Color(1:24050,:);