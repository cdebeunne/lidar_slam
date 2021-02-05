% initializing the datapath

datapath = uigetdir('Select the data folder of velodyne points');
folder = uigetdir('select the folder for the point clouds');
numberOfScans = length(dir([datapath '/*.bin']));
counter = 0;
traj = {};

% creating the scan array

disp('creating the scan array...');
for i=1:numberOfScans 
    fileID = datapath;
    fileID = strcat(fileID, '/');
    numberOfZeros = 10 - length(int2str(counter));
    for i=1:numberOfZeros
        fileID = strcat(fileID, '0');
    end
    fileID = strcat(fileID, int2str(counter), '.bin');
    counter = counter + 1;
    try
        fid = fopen(fileID, 'r');
        data = fread(fid,'float');
        fclose(fid);
        data = reshape(data, 4, []);
        data = single(data');
        traj{counter} = data(:, 1:3);
    catch
        numberOfScans = numberOfScans + 1;
    end
end

% creating the timestamp array

disp('creating the timestamp array...');
lenPath = length(datapath);
tspath = datapath(1:lenPath-5);
fid = fopen(strcat(tspath, '/timestamps.txt'));
txt = textscan(fid,'%s %f32 :  %f32 : %f64');
fclose(fid);
time = [];
time(1) = 0;

for i=1:length(txt{1})-1
    t0 = txt{3}(i)*60+txt{4}(i);
    t1 = txt{3}(i+1)*60+txt{4}(i+1);
    time(i+1) = time(i) + (t1-t0);
end

% save the data in a mat file

disp('saving the data...');
% save('KITTI_VEL_SCAN.mat', 'traj', 'time');
for i=1:length(traj)
    ptCloud = pointCloud(traj{i});
    name = strcat(folder, '/message', int2str(i), '.pcd');
    pcwrite(ptCloud,name,'Encoding','binary');
end