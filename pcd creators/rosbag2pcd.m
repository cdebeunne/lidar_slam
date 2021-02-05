[filename,pathname] = uigetfile('*.bag','Select the rosbag file');
bag = rosbag(strcat(pathname, filename));
folder = uigetdir('select the folder for the point clouds');

for i=1:bag.NumMessages
    msg = readMessages(bag, i);
    ptCloud = readXYZ(msg{1});
    ptCloud = pointCloud(ptCloud);
    name = strcat(folder, '/message', int2str(i), '.pcd');
    pcwrite(ptCloud,name,'Encoding','binary');  
end
