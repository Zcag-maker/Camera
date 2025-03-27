
clear;
clc;

% 视频路径
videoPath = 'C:\Users\DELL\Desktop\底特律大学\第二学期\experiment\video\video2.avi'; % 替换为实际视频路径

% 检查视频文件是否存在
if ~isfile(videoPath)
    disp('视频文件未找到，请检查路径。')
 
else
    % 创建视频读取对象
    videoObj = VideoReader(videoPath);

    % 记录视频的尺寸
    frameWidth = videoObj.Width;
    frameHeight = videoObj.Height;

    % 创建固定的图形窗口
    hFig = figure('Name', '处理视频', 'NumberTitle', 'off', ...
                  'MenuBar', 'none', 'ToolBar', 'none'); % 禁用菜单和工具栏

    % 帧计数器
    frameCount = 0;

    % 遍历视频中的每一帧
    while hasFrame(videoObj)
        % 读取当前帧
        frame = readFrame(videoObj);
        frameCount = frameCount + 1;

        % 每5帧处理一次
        if mod(frameCount, 5) == 0
            % 1. 转换为灰度图像
            grayImg = rgb2gray(frame);

            % 2. 去噪处理（双边滤波）
            blurImg = imbilatfilt(grayImg);

            % 3. Canny 边缘检测
            cannyEdges = edge(blurImg, 'Canny', [0.2, 0.4]);

            % 4. 形态学腐蚀生成种子图像
            se = strel('rectangle', [1,2]); % 结构元素
            marker = imerode(cannyEdges, se);

            % 5. 使用种子图像和掩膜图像进行重构
            reconstructedEdges = imreconstruct(marker, cannyEdges);

            % 6. 可选：对重构结果进行膨胀增强
            enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2,6]));

            % 7. 删除边缘线中间的部分
            stats = regionprops(enhancedEdges, 'BoundingBox', 'Area', 'MajorAxisLength', 'MinorAxisLength');
            filteredEdges = enhancedEdges;
            centerX_min = frameWidth * 0.15;
            centerX_max = frameWidth * 0.7;
            for i = 1:length(stats)
                bbox = stats(i).BoundingBox;
                xPos = bbox(1);
                width = bbox(3);

                % 仅删除位于边缘线中间的部分
                if (xPos > centerX_min && xPos + width < centerX_max)
                    filteredEdges(round(bbox(2)):round(bbox(2)+bbox(4)), round(xPos):round(xPos+width)) = 0;
                end
            end

            % 8. 显示处理结果
            subplot(2, 2, 2);
            imshow(blurImg);
            %title('去噪图像');

            subplot(2, 2, 3);
            imshow(reconstructedEdges);
            %title('重构边缘检测结果');

            subplot(2, 2, 4);
            imshow(filteredEdges);
            %title('删除边缘线中间部分的结果');

            % 控制显示速度，与视频帧率一致
            pause(1 / videoObj.FrameRate);
        end
    end

    disp('视频处理完成，动态显示所有结果。');
end