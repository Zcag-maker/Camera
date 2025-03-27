%% 清理环境
clear; clc; close all;

%% 初始化 ROS2 节点与订阅者
node = ros2node("fusion_node");

% 摄像头订阅者（话题：/multisense/left/image_rect）
imageSub = ros2subscriber(node, "/multisense/left/image_rect", "sensor_msgs/Image", ...
    "Reliability", "besteffort", "Durability", "volatile", "History", "keeplast", "Depth", 10);

% 激光雷达订阅者（话题：/velodyne_points）
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

%% 参数设置

% ----- 激光雷达参数 -----
minRange = 0.5;
maxRange = 5;
minAngle = -180;
maxAngle = 180;
clusterDistanceThreshold = 0.2; % 单位：米
minClusterPoints = 10;          % 聚类最少点数（过滤噪声）
minClusterSize = 0.1;           % 聚类最小尺寸（米）

% ----- 占用图与世界坐标参数（摄像头与激光雷达统一） -----
resolution = 0.05;               % m/像素
xWorldLimits = [-1, 3];          % X 轴范围（单位：米）
yWorldLimits = [-1, 1];          % Y 轴范围（单位：米）
outputCols = round(diff(xWorldLimits)/resolution);
outputRows = round(diff(yWorldLimits)/resolution);
outputRef = imref2d([outputRows, outputCols], xWorldLimits, yWorldLimits);

% ----- 相机参数 -----
% 内参（像素单位），已根据MultiSense S7 2MP调整到图像尺寸1024×544
K = [595,   0, 512;
     0, 590, 272;
     0,   0,   1];
% 外参：相机位置 C = (0,0,0.8)（米），pitch=-30°（转为弧度）
theta = -30 * pi/180;
R = [cos(theta), 0, sin(theta);
     0,          1, 0;
     -sin(theta),0, cos(theta)];
C = [0; 0; 0.8];
t_extr = -R * C;
% 构造单应矩阵 H（仅针对地面平面 Z=0）
H = K * [R(:,1:2), t_extr];
tform = projective2d(H');

%% 创建实时显示窗口（2行4列子图）
hFig = figure('Name', '数据融合：摄像头与激光雷达', 'NumberTitle', 'off');

while isvalid(hFig)
    overallTimer = tic;
    
    %% 1. 摄像头图像接收及处理
    try
        tPreReceive = tic;
        imgMsg = receive(imageSub, 5);  % 最多等待 5秒
        rawImg = rosReadImage(imgMsg);   % 单通道灰度图，尺寸1024×544
        tReceive = toc(tPreReceive);
    catch ME
        warning('接收摄像头图像失败：%s', ME.message);
        continue;
    end
    
    % 双边滤波：降噪并保留边缘信息
    tPreBlur = tic;
    blurredImg = imbilatfilt(rawImg);
    tBlur = toc(tPreBlur);
    
    % 动态阈值检测（根据图像平均亮度线性确定阈值）
    tPreThreshold = tic;
    avgBrightness = mean(rawImg(:));
    if avgBrightness <= 13
        greythreshold = 12;
    elseif avgBrightness >= 150
        greythreshold = 175;
    else
        greythreshold = 1.0788 * avgBrightness + 4.0248;
    end
    extractGraph = rawImg < greythreshold;
    tThreshold = toc(tPreThreshold);
    
    % 形态学处理：增强车道线连续性
    tPreMorph = tic;
    laneLineMask = processEdges(extractGraph);
    tMorph = toc(tPreMorph);
    
    % 生成边缘叠加图（取反后车道区域显示为红色）
    tPreOverlay = tic;
    overlay = createOverlay(laneLineMask);
    tOverlay = toc(tPreOverlay);
    
    % 占用图生成（摄像头）：透视变换到地面平面
    tPreOccCam = tic;
    redChannel = overlay(:,:,1);
    occupancyInputCam = redChannel > 200;
    [occupancyMapCam, ~] = imwarp(occupancyInputCam, tform, 'OutputView', outputRef);
    occupancyMapCam = occupancyMapCam > 0.5;
    tOccCam = toc(tPreOccCam);
    
    %% 2. 激光雷达点云接收及处理
    lidarAvailable = ~isempty(pointsSubscriber.LatestMessage);
    if lidarAvailable
        tPreLidar = tic;
        msgLidar = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msgLidar);
        
        % 计算每个点的距离与角度
        distances = sqrt(xyzPoints(:,1).^2 + xyzPoints(:,2).^2 + xyzPoints(:,3).^2);
        angles = atan2d(xyzPoints(:,2), xyzPoints(:,1));
        
        % 过滤点云（基于距离和角度）
        validIndices = (distances >= minRange) & (distances <= maxRange) & ...
                       (angles >= minAngle) & (angles <= maxAngle);
        filteredPoints = xyzPoints(validIndices, :);
        
        % 聚类并筛选障碍物
        obstaclePoints = [];
        if ~isempty(filteredPoints)
            pcLidar = pointCloud(filteredPoints);
            [labels, numClusters] = pcsegdist(pcLidar, clusterDistanceThreshold);
            if numClusters > 0
                for i = 1:numClusters
                    clusterIndices = (labels == i);
                    clusterPoints = pcLidar.Location(clusterIndices, :);
                    % 计算包围盒尺寸
                    minVals = min(clusterPoints, [], 1);
                    maxVals = max(clusterPoints, [], 1);
                    clusterSize = maxVals - minVals;
                    if (size(clusterPoints,1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                        obstaclePoints = [obstaclePoints; clusterPoints];
                    end
                end
            end
        end
        
        % 生成激光雷达占用图（基于障碍物点云）
        if ~isempty(obstaclePoints)
            xEdges = xWorldLimits(1):resolution:xWorldLimits(2);
            yEdges = yWorldLimits(1):resolution:yWorldLimits(2);
            occupancyMapLidar = histcounts2(obstaclePoints(:,1), obstaclePoints(:,2), xEdges, yEdges);
            occupancyMapLidar = occupancyMapLidar' > 0;  % 转置并二值化
        else
            occupancyMapLidar = false(outputRows, outputCols);
        end
        tLidar = toc(tPreLidar);
    else
        occupancyMapLidar = false(outputRows, outputCols);
        tLidar = 0;
    end
    
    %% 3. 数据融合：合并摄像头与激光雷达的占用图
    % 融合策略：采用逻辑或运算——若任一传感器检测到占用，则该区域标记为占用
    fusedOccupancyMap = occupancyMapCam | occupancyMapLidar;
    
    overallTime = toc(overallTimer);
    
    %% 4. 可视化显示（2行4列子图）
    clf(hFig);
    
    % (1) 摄像头原始图像
    subplot(2,4,1);
    imshow(rawImg, []);
    title(sprintf('原始图像\n(接收: %.3f秒)', tReceive));
    
    % (2) 双边滤波结果
    subplot(2,4,2);
    imshow(blurredImg, []);
    title(sprintf('双边滤波\n(%.3f秒)', tBlur));
    
    % (3) 阈值检测结果
    subplot(2,4,3);
    imshow(extractGraph, []);
    title(sprintf('阈值检测 (<%d)\n(%.3f秒)', round(greythreshold), tThreshold));
    
    % (4) 形态学处理结果
    subplot(2,4,4);
    imshow(laneLineMask, []);
    title(sprintf('形态学处理\n(%.3f秒)', tMorph));
    
    % (5) 边缘叠加图（车道区域为红色）
    subplot(2,4,5);
    imshow(overlay);
    title(sprintf('边缘叠加图\n(%.3f秒)', tOverlay));
    
    % (6) 摄像头占用图（透视变换后）
    subplot(2,4,6);
    imshow(occupancyMapCam, outputRef);
    title(sprintf('摄像头占用图\n(%.3f秒)', tOccCam));
    
    % (7) 信息显示（处理时长、平均亮度、动态阈值及激光雷达处理时延）
    subplot(2,4,7);
    infoText = sprintf(['整帧处理耗时: %.3f秒\n' ...
                        '摄像头接收: %.3f秒\n滤波: %.3f秒\n阈值: %.3f秒\n形态学: %.3f秒\n' ...
                        '平均亮度: %d\n动态阈值: %d\n激光雷达处理: %.3f秒'], ...
                        overallTime, tReceive, tBlur, tThreshold, tMorph, round(avgBrightness), round(greythreshold), tLidar);
    text(0.1, 0.5, infoText, 'FontSize', 12);
    axis off;
    
    % (8) 数据融合占用图（摄像头与激光雷达）
    subplot(2,4,8);
    imshow(fusedOccupancyMap, outputRef);
    title('数据融合占用图');
    
    drawnow;
    
    disp(['整帧处理耗时: ', num2str(overallTime), '秒 | 摄像头接收: ', num2str(tReceive), '秒 | 激光雷达处理: ', num2str(tLidar), '秒']);
end

%% 辅助函数：形态学处理（增强车道线区域连续性）
function enhancedEdges = processEdges(edgeImg)
    se = strel('rectangle', [1, 2]);
    marker = imerode(edgeImg, se);
    reconstructedEdges = imreconstruct(marker, edgeImg);
    enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2, 6]));
end

%% 辅助函数：生成边缘叠加图
function overlay = createOverlay(laneLineMask)
    overlay = zeros([size(laneLineMask), 3], 'uint8');
    overlay(:,:,1) = 255 * uint8(~laneLineMask);  % 红色通道
    overlay(:,:,2) = 0;
    overlay(:,:,3) = 0;
end
