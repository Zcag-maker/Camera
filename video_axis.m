%% ========================== 1. 连接 ROS2 ==========================
clear;
clc;

% 创建 ROS2 节点
ros2node = ros2node("matlab_node");

disp("正在连接 ROS2...");

%% ========================== 2. 订阅相机参数 ==========================
disp("订阅相机参数...");
cameraSub = ros2subscriber(ros2node, "/multisense/left/camera_info");

% 获取相机参数
cameraMsg = receive(cameraSub, 2); % 等待 2 秒
K = reshape(cameraMsg.K, [3, 3])'; % 3x3 内参矩阵

disp("相机内参矩阵 K:");
disp(K);

%% ========================== 3. 订阅 TF 变换 ==========================
disp("订阅相机外参...");
tfSub = ros2subscriber(ros2node, "/tf");

% 读取 TF 变换
tfMsg = receive(tfSub, 2);

% 提取旋转矩阵 R 和 平移向量 t
R = quat2rotm([tfMsg.transforms.transform.rotation.w, ...
               tfMsg.transforms.transform.rotation.x, ...
               tfMsg.transforms.transform.rotation.y, ...
               tfMsg.transforms.transform.rotation.z]);

t = [tfMsg.transforms.transform.translation.x;
     tfMsg.transforms.transform.translation.y;
     tfMsg.transforms.transform.translation.z];

disp("相机旋转矩阵 R:");
disp(R);
disp("相机平移向量 t:");
disp(t);

%% ========================== 4. 订阅图像 ==========================
disp("订阅 ROS2 相机图像...");
imageSub = ros2subscriber(ros2node, "/multisense/left/image_raw");

%% ========================== 5. 处理视频流 ==========================
hFig = figure('Name', '车道检测 & Image to World', 'NumberTitle', 'off', ...
              'MenuBar', 'none', 'ToolBar', 'none');

frameCount = 0;

while true
    % 获取 ROS2 图像
    imgMsg = receive(imageSub, 2);
    frame = rosReadImage(imgMsg);
    frameCount = frameCount + 1;

    if mod(frameCount, 5) == 0  % 每5帧处理一次，减少计算量
        %% **5.1 预处理**
        grayImg = rgb2gray(frame);  % 转换为灰度图像
        blurImg = imbilatfilt(grayImg);  % 双边滤波去噪

        %% **5.2 Canny 边缘检测**
        cannyEdges = edge(blurImg, 'Canny', [0.2, 0.4]);

        %% **5.3 形态学处理**
        se = strel('rectangle', [1,2]);  % 结构元素
        marker = imerode(cannyEdges, se);
        enhancedEdges = imreconstruct(marker, cannyEdges);
        enhancedEdges = imdilate(enhancedEdges, strel('rectangle', [2,6]));

        %% **5.4 提取车道边缘点**
        [edgeY, edgeX] = find(enhancedEdges);  % 车道像素点 (u, v)

        %% **5.5 计算世界坐标**
        Z_world = 1000;  % 设定固定深度 1m（单位：mm）
        worldPoints = zeros(length(edgeX), 3);

        for i = 1:length(edgeX)
            uv1 = [edgeX(i); edgeY(i); 1];
            X_cam = inv(K) * uv1 * Z_world;  % 计算相机坐标
            X_world = R \ (X_cam - t);  % 计算世界坐标
            worldPoints(i, :) = X_world';
        end

        %% **5.6 显示结果**
        subplot(1, 2, 1);
        imshow(frame);
        hold on;
        plot(edgeX, edgeY, 'g.', 'MarkerSize', 5);
        title('检测到的车道边缘');

        subplot(1, 2, 2);
        plot3(worldPoints(:, 1), worldPoints(:, 2), worldPoints(:, 3), 'r.');
        grid on;
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        title('转换后的世界坐标');
        view(3);

        pause(0.1);
    end
end

