clear;
clc;

% 视频路径
videoPath = 'C:\Users\DELL\Desktop\底特律大学\第二学期\experiment\video\video2.avi'; % 替换为实际路径

% 检查视频文件是否存在
if ~isfile(videoPath)
    disp('视频文件未找到，请检查路径。');
else
    % 创建视频读取对象
    videoObj = VideoReader(videoPath);
    frameWidth = videoObj.Width;
    frameHeight = videoObj.Height;

    % 创建固定的图形窗口
    hFig = figure('Name', '去除车道内多余内容 & 颜色识别', 'NumberTitle', 'off', ...
                  'MenuBar', 'none', 'ToolBar', 'none');

    % 帧计数器
    frameCount = 0;

    while hasFrame(videoObj)
        % 读取当前帧
        frame = readFrame(videoObj);
        frameCount = frameCount + 1;

        if mod(frameCount, 5) == 0
            % 1. 转换为灰度图像
            grayImg = rgb2gray(frame);

            % 2. 去噪处理（双边滤波）
            blurImg = imbilatfilt(grayImg);

            % 3. Canny 边缘检测
            cannyEdges = edge(blurImg, 'Canny', [0.2, 0.4]);

            % 4. 形态学处理，增强边缘
            se = strel('rectangle', [1,2]); % 结构元素
            marker = imerode(cannyEdges, se);
            reconstructedEdges = imreconstruct(marker, cannyEdges);
            enhancedEdges = imdilate(reconstructedEdges, strel('rectangle', [2,6]));

            % 5. 计算连通区域属性
            stats = regionprops(enhancedEdges, 'BoundingBox', 'MajorAxisLength', 'Area', 'PixelIdxList', 'Eccentricity', 'Orientation', 'Solidity');

            % 6. 设定阈值，调整过滤标准
            minAreaThreshold = 800;  % 增加区域面积阈值，去除小区域
            minLengthThreshold = 150; % 增加边缘长度阈值，避免短线被标记
            minEccentricity = 0.85;  % 增加圆度阈值，确保线条不太过弯曲
            maxOrientationThreshold = 10;  % 增加边缘线的最大倾斜度，避免曲线过大的误标
            minSolidity = 0.6;  % 增加紧密度阈值，确保线条更接近直线而不是不规则的区域

            % 7. 过滤出主要道路边缘（绿色）和小区域（红色）
            majorEdgeMask = false(size(enhancedEdges));
            smallEdgeMask = false(size(enhancedEdges));

            for i = 1:length(stats)
                area = stats(i).Area;
                majorLength = stats(i).MajorAxisLength;
                eccentricity = stats(i).Eccentricity;
                orientation = stats(i).Orientation;
                solidity = stats(i).Solidity;

                % 仅填充边缘像素点，而不是整个 `BoundingBox` 区域
                if area > minAreaThreshold && majorLength > minLengthThreshold && eccentricity > minEccentricity && abs(orientation) < maxOrientationThreshold && solidity > minSolidity
                    majorEdgeMask(stats(i).PixelIdxList) = true;
                else
                    smallEdgeMask(stats(i).PixelIdxList) = true;
                end
            end

            % 8. 计算车道区域
            laneMask = ~majorEdgeMask; % 取反，确保车道内内容被去除

            % 9. 确保 `majorEdgeMask` 和 `smallEdgeMask` 为 `double`
            majorEdgeMask = double(majorEdgeMask);
            smallEdgeMask = double(smallEdgeMask);

            % 10. **使用黑白边缘检测图像作为底图**
            edgeBase = repmat(uint8(enhancedEdges) * 255, [1, 1, 3]); % 三通道黑白图
            edgeBase = im2double(edgeBase); % 确保数据类型一致

            % 11. 绿色标注主要边缘
            edgeBase(:,:,1) = edgeBase(:,:,1) .* (1 - majorEdgeMask);  % 关闭 R 通道
            edgeBase(:,:,2) = edgeBase(:,:,2) .* (1 - majorEdgeMask) + majorEdgeMask;  % 绿色增强
            edgeBase(:,:,3) = edgeBase(:,:,3) .* (1 - majorEdgeMask);  % 关闭 B 通道

            % 12. 红色标注小区域和短边缘
            edgeBase(:,:,1) = edgeBase(:,:,1) + smallEdgeMask;  % R 通道增强
            edgeBase(:,:,2) = edgeBase(:,:,2) .* (1 - smallEdgeMask);  % G 通道减少
            edgeBase(:,:,3) = edgeBase(:,:,3) .* (1 - smallEdgeMask);  % B 通道减少

            % 13. 确保像素值范围在 0-1 之间
            edgeBase = min(max(edgeBase, 0), 1);

            % 14. 转换回 `uint8` 以便显示
            edgeBase = uint8(edgeBase * 255);

            % 15. 显示结果
            subplot(1, 3, 1);
            imshow(enhancedEdges);
            title('原始边缘');

            subplot(1, 3, 2);
            imshow(laneMask);  % 车道内部区域
            title('去除车道内多余内容');

            subplot(1, 3, 3);
            imshow(edgeBase);
            title('去噪后边缘线（绿色） & 其他线（红色）');

            % 控制显示速度
            pause(1 / videoObj.FrameRate);
        end
    end

    disp('视频处理完成，动态显示所有结果。');
end





