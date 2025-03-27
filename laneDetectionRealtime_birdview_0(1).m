function laneDetectionRealtime_dynamicThresholdWithOccupancyMap
% 基于摄像头图像的车道线检测、动态阈值调整及占用图生成
%
% 处理流程：
% 1. 对单通道灰度图进行双边滤波降噪；
% 2. 根据图像平均亮度动态（线性插值）确定阈值：
%       当 avgBrightness <= 100 时，阈值取55；
%       当 avgBrightness >= 200 时，阈值取95；
%       否则： greythreshold = 55 + 0.4*(avgBrightness - 100)
% 3. 阈值检测后进行形态学处理，得到车道候选区域；
% 4. 生成边缘叠加图：取反后车道区域显示为红色 [255,0,0]，背景为黑色；
% 5. 利用内参和外参（相机位置 (0,0,0.8)，pitch=-30°）构造单应矩阵，
%    并使用自定义 OutputView（假设地面范围 x∈[-1.5,0.5] 米、y∈[-1,1] 米，分辨率 0.05 m/像素）
%    对 overlay 中红色部分进行透视变换生成占用图（车道区域值为1，其余为0）；
% 6. 采用 2×4 布局显示：原始图、滤波图、阈值检测结果、形态学处理结果、边缘叠加图、占用图及实时文本信息。

%% 初始化 ROS2 节点与订阅者
node = ros2node("test_node");
imageSub = ros2subscriber(node, "/multisense/left/image_rect", "sensor_msgs/Image", ...
    "Reliability", "besteffort", "Durability", "volatile", "History", "keeplast", "Depth", 10);

%% 创建实时显示窗口（2行4列子图）
hFig = figure('Name', '实时车道检测、动态阈值及占用图', 'NumberTitle', 'off');

while isvalid(hFig)
    overallTimer = tic;
    
    %% 1. 图像接收
    try
        tPreReceive = tic;
        imgMsg = receive(imageSub, 5);  % 最多等待 5秒
        rawImg = rosReadImage(imgMsg);   % 单通道灰度图，尺寸1024×544
        tReceive = toc(tPreReceive);
    catch ME
        warning('接收图像失败：%s', ME.message);
        continue;
    end
    
    %% 2. 双边滤波：降噪并保留边缘信息
    tPreBlur = tic;
    blurredImg = imbilatfilt(rawImg);
    tBlur = toc(tPreBlur);
    
    %% 3. 动态阈值检测（采用线性插值）
    tPreThreshold = tic;
    avgBrightness = mean(rawImg(:));
    if avgBrightness <= 100
        greythreshold = 55;
    elseif avgBrightness >= 200
        greythreshold = 95;
    else
        greythreshold = 55 + 0.4*(avgBrightness - 100);
    end
    extractGraph = rawImg < greythreshold;
    tThreshold = toc(tPreThreshold);
    
    %% 4. 形态学处理：增强车道线区域连续性
    tPreMorph = tic;
    laneLineMask = processEdges(extractGraph);
    tMorph = toc(tPreMorph);
    
    %% 5. 生成边缘叠加图
    % 假设 laneLineMask 中车道线区域为 false，
    % 取反后使车道区域显示为红色 [255,0,0]，背景为黑色
    tPreOverlay = tic;
    overlay = createOverlay(laneLineMask);
    tOverlay = toc(tPreOverlay);
    
    %% 6. 占用图生成（透视变换到地面平面）
    tPreOcc = tic;
    % 自定义 OutputView：假设地面范围 x∈[-1.5,0.5] 米，y∈[-1,1] 米，分辨率 0.05 m/像素
    xWorldLimits = [-1.5, 0.5];
    yWorldLimits = [-1, 1];
    pixelExtent = 0.05;
    outputCols = round(diff(xWorldLimits)/pixelExtent);
    outputRows = round(diff(yWorldLimits)/pixelExtent);
    outputRef = imref2d([outputRows, outputCols], xWorldLimits, yWorldLimits);
    
    % 设置相机内参（像素单位），基于 MultiSense S7 2MP 参数调整到当前图像尺寸1024×544
    K = [595,   0, 512;
         0, 590, 272;
         0,   0,   1];
    
    % 设置相机外参：相机位置 C = (0,0,0.8)（单位：米），欧拉角 (roll, pitch, yaw) = (0, -30, 0)
    theta = -30 * pi/180;  % -30°转为弧度
    % 绕 y 轴旋转（pitch 旋转）
    R = [cos(theta), 0, sin(theta);
         0,          1, 0;
         -sin(theta),0, cos(theta)];
    r1 = R(:,1);
    r2 = R(:,2);
    C = [0; 0; 0.8];
    t_extr = -R * C;
    
    % 构造单应矩阵 H：对于地面平面 (Z=0) 的点 [X; Y; 1] 有：
    % s*[u; v; 1] = K * [r1, r2, t_extr] * [X; Y; 1]
    H = K * [r1, r2, t_extr];
    H_inv = inv(H);
    tform = projective2d(H_inv');
    
    % 从 overlay 中提取红色部分（车道区域）：直接取红色通道 > 200 得到二值图
    redChannel = overlay(:,:,1);
    occupancyInput = redChannel > 200;
    
    % 透视变换 occupancyInput 到地面平面
    [occupancyMap, ~] = imwarp(occupancyInput, tform, 'OutputView', outputRef);
    occupancyMap = occupancyMap > 0.5;
    tOcc = toc(tPreOcc);
    
    overallTime = toc(overallTimer);
    
    %% 7. 显示各处理阶段结果（2行4列布局）
    clf(hFig);
    
    subplot(2,4,1);
    imshow(rawImg, []);
    title(sprintf('原始图像\n(接收: %.3f秒)', tReceive));
    
    subplot(2,4,2);
    imshow(blurredImg, []);
    title(sprintf('双边滤波\n(%.3f秒)', tBlur));
    
    subplot(2,4,3);
    imshow(extractGraph, []);
    title(sprintf('阈值检测 (<%d)\n(%.3f秒)', round(greythreshold), tThreshold));
    
    subplot(2,4,4);
    imshow(laneLineMask, []);
    title(sprintf('形态学处理\n(%.3f秒)', tMorph));
    
    subplot(2,4,5);
    imshow(overlay);
    title(sprintf('边缘叠加图\n(%.3f秒)', tOverlay));
    
    subplot(2,4,6);
    imshow(occupancyMap, outputRef);
    title(sprintf('占用图\n(%.3f秒)', tOcc));
    
    subplot(2,4,7);
    txt = sprintf(['整帧处理耗时: %.3f 秒\n' ...
                   '接收: %.3f 秒\n滤波: %.3f 秒\n阈值: %.3f 秒\n形态学: %.3f 秒\n' ...
                   '平均亮度: %d\n动态阈值: %d'], ...
                   overallTime, tReceive, tBlur, tThreshold, tMorph, round(avgBrightness), round(greythreshold));
    text(0.1, 0.5, txt, 'FontSize', 12);
    axis off;
    
    subplot(2,4,8);
    % 可选：显示 occupancyInput（透视前的二值图），便于调试
    imshow(occupancyInput);
    title('透视前二值图');
    
    drawnow;
    
    disp(['整帧处理耗时: ', num2str(overallTime), ' 秒  |  图像接收耗时: ', num2str(tReceive), ' 秒']);
end
end

%% 辅助函数：形态学处理（增强车道线区域连续性）
function enhancedEdges = processEdges(edgeImg)
    % 对输入二值图像进行腐蚀、重构和膨胀操作，增强车道线区域连续性
    se = strel('rectangle', [1, 2]);
    marker = imerode(edgeImg, se);
    reconstructedEdges = imreconstruct(marker, edgeImg);
    enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2, 6]));
end

%% 辅助函数：生成边缘叠加图
function overlay = createOverlay(laneLineMask)
    % 生成颜色叠加图：
    % 假设 laneLineMask 中车道线区域为 false，
    % 取反后使车道区域显示为红色 [255,0,0]，背景为黑色
    overlay = zeros([size(laneLineMask), 3], 'uint8');
    overlay(:,:,1) = 255 * uint8(~laneLineMask);  % 红色通道
    overlay(:,:,2) = 0;
    overlay(:,:,3) = 0;
end
