%--------------------------------------------------------------------------
% projdbscan.m - 雷达点云与图像融合系统
% 功能：实现雷达点云的DBSCAN聚类、投影到图像平面，并进行可视化
% 版本：1.0
% 日期：2024-01-13
% output : *_frame_image.txt *_frame_real.txt
%--------------------------------------------------------------------------s
clc;
clear;
close all;
% day_data night_data
%% 1. 加载相机与雷达参数
% 配置参数
config = struct();
% 数据处理参数
config.scaleFactor = 0.01;      % 高度映射因子
config.epsilon = 1;           % DBSCAN聚类距离阈值
config.minPts = 2;              % DBSCAN最小点数
doCluster = true;
% 显示参数
config.clim = [0, 50];          % 颜色映射范围
config.scatterSize = 20;        % 散点大小
% 时间参数
config.pauseTime = 0.02;        % 正常播放帧间隔
config.pauseSleep = 0.05;       % 暂停时休眠时间
% ROI参数
config.roiScaleFactor = 0.15;   % ROI缩放因子
config.roiMethod = 'logarithmic';    % ROI计算方法
config.roiMinSize = 20;         % ROI最小尺寸
config.roiMaxSize = 200;        % ROI最大尺寸
% 输出目录
config.output_dir = "E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\D-S Fusion\data\day_data";

% 相机参数文件
config.cameraParamsFile = 'E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\Radar\radar_test\cameraParams640.mat';
% 雷达到相机的变换矩阵
config.T_radar2camera = [0, -1,  0,  0.35;  % R + T [x,y,z]偏移
                         0,  0, -1,    0;
                         1,  0,  0,    0;
                         0,  0,  0,    1];
% T = [0.35; -0.1; 0];
% x作用：调整雷达在相机坐标系中的左右位置
% 正值：雷达在相机右侧 → 投影点向左移动
% 负值：雷达在相机左侧 → 投影点向右移动
% y作用：调整雷达在相机坐标系中的上下位置
% 正值：雷达在相机下方 → 投影点向下移动
% 负值：雷达在相机上方 → 投影点向上移动
% z作用：调整雷达在相机坐标系中的前后距离
% 正值：雷达在相机前方 → 投影点向外扩散
% 负值：雷达在相机后方 → 投影点向中心收缩

% 选择文件夹（当前使用固定路径，可替换为交互式选择）
% folderPath = 'E:\hducc\Data_process\radar_data\data\20251219-203223';
folderPath = uigetdir('E:\hducc\Data_process\radar_data\data','选择图片文件夹');
if folderPath == 0
    disp('未选择文件夹，程序终止。');
    return;
end
config.radarDataPath = fullfile(folderPath, 'AKBK_data_2d.mat');
config.videoPath     = fullfile(folderPath, 'video10.mp4');
fileName = split(folderPath,'\');
name = string(fileName(6));

% 相机到像素的变换矩阵(内参)
try
    load(config.cameraParamsFile);
    K_camera = cameraParams640.K;
    % 确保畸变系数是[RadialDistortion, TangentialDistortion]的组合
    if isfield(cameraParams640, 'RadialDistortion') && isfield(cameraParams640, 'TangentialDistortion')
        distortion_coefficient = [cameraParams640.RadialDistortion, cameraParams640.TangentialDistortion];
    else
        distortion_coefficient = [0, 0, 0, 0]; % 默认无畸变
    end
    fprintf('✓ 成功加载相机参数文件: %s\n', config.cameraParamsFile);
catch ME
    error('无法加载相机参数文件: %s\n错误信息: %s', config.cameraParamsFile, ME.message);
end

% 雷达到相机的变换矩阵(外参)
T_radar2camera = config.T_radar2camera; 

%% 2. 读取雷达数据和视频
% 加载雷达数据
try
    load(config.radarDataPath);
    frames = fieldnames(dataStruct);
    numFramesRadar = numel(frames);
    fprintf('✓ 成功加载雷达数据文件: %s\n', config.radarDataPath);
    fprintf('  雷达总帧数: %d\n', numFramesRadar);
catch ME
    error('无法加载雷达数据文件: %s\n错误信息: %s', config.radarDataPath, ME.message);
end

% 打开视频文件
try
    vReader = VideoReader(config.videoPath);
    fprintf('✓ 成功打开视频文件: %s\n', config.videoPath);
    fprintf('  视频分辨率: %dx%d\n', vReader.Width, vReader.Height);
    fprintf('  视频帧率: %.2f FPS\n', vReader.FrameRate);
    videoFrames = struct();
catch ME
    error('无法打开视频文件: %s\n错误信息: %s', config.videoPath, ME.message);
end

% 预读取所有视频帧并建立时间戳
fprintf('开始预读取视频帧...\n');
frameCount = 0;
errorCount = 0;
try
    while hasFrame(vReader)
        frameCount = frameCount + 1;
        try
            videoFrames(frameCount).data = readFrame(vReader);
            videoFrames(frameCount).time = vReader.CurrentTime;
        catch ME
            errorCount = errorCount + 1;
            fprintf('  警告：读取第%d帧失败: %s\n', frameCount, ME.message);
        end
    end
    numFramesVideo = frameCount;
    fprintf('✓ 视频帧预读取完成\n');
    fprintf('  成功读取: %d 帧\n', numFramesVideo - errorCount);
    fprintf('  读取失败: %d 帧\n', errorCount);
catch ME
    error('视频帧预读取失败: %s', ME.message);
end

% 帧同步：取较小的帧数，确保同步
numFrames = min(numFramesRadar, numFramesVideo);
fprintf('\n帧同步信息:\n');
fprintf('  雷达帧数: %d\n', numFramesRadar);
fprintf('  视频帧数: %d\n', numFramesVideo);
fprintf('  实际处理帧数: %d\n', numFrames);

% 建立帧映射（如果需要时间同步）
try
    frameMapping = createFrameMapping(dataStruct, videoFrames, frames);
    fprintf('✓ 成功创建帧映射\n');
catch ME
    warning('创建帧映射失败，使用默认一一映射: %s\n', E.message);
    frameMapping = 1:numFrames;
end

%% 输出目录设置
output_dir = config.output_dir;
if ~exist(output_dir, 'dir')
    try
        mkdir(output_dir);
        fprintf('✓ 成功创建输出目录: %s\n', output_dir);
    catch ME
        error('无法创建输出目录: %s\n错误信息: %s', output_dir, ME.message);
    end
end

% 保存实际雷达数据
real_filename = fullfile(output_dir, sprintf('%s_frame_real.txt',name));
fid_real = fopen(real_filename, 'w');

% 保存图像坐标数据
image_filename = fullfile(output_dir, sprintf('%s_frame_image.txt',name));
fid_image = fopen(image_filename, 'w');

%% 4. 逐帧处理主循环
% 初始化播放控制参数
frameIdx = 1;  % 独立控制帧索引
currentFrame = 1;  % 添加当前帧跟踪
isPaused = false;  % 播放状态

% 将变量存入base workspace，供keyControl访问
assignin('base', 'isPaused', false);
assignin('base', 'frameIdx', frameIdx);
assignin('base', 'currentFrame', currentFrame);
assignin('base', 'numFrames', numFrames);

% 创建主图窗
fig = figure('Name', 'Radar-Image Fusion', 'KeyPressFcn', @keyControl);

% 初始化显示 - 预配置所有图形对象
fprintf('初始化可视化显示...\n');

% 左侧子图：图像和雷达投影点
ax1 = subplot(1, 2, 1);
hVideo = imshow(zeros(vReader.Height, vReader.Width, 3, 'uint8'), 'Parent', ax1);
hold on;

% 创建散点图对象 - 预分配，减少创建次数
hScatter1 = scatter(NaN, NaN, config.scatterSize, 'filled', 'Parent', ax1);

% 设置轴属性
xlabel(ax1, 'X Position'); ylabel(ax1, 'Y Position');
axis(ax1, [0, vReader.Width, 0, vReader.Height]);
grid(ax1, 'on');

% 设置colormap和colorbar
colormap(ax1, jet);
cbar = colorbar(ax1);
ylabel(cbar, '距离 (m)');
clim(ax1, config.clim);

% 右侧子图：点云数据
ax2 = subplot(1, 2, 2);
hScatter2 = scatter(NaN, NaN, 10, 'filled', 'Parent', ax2);
xlabel(ax2, 'X Position'); % 横向距离
ylabel(ax2, 'Y Position'); % 纵向距离
title(ax2, 'Point Cloud Data');
axis(ax2, [ -25, 25, -5, 50]);
grid(ax2, 'on');

% 优化动画性能设置
drawnow('limitrate'); % 限制重绘频率
fprintf('✓ 可视化显示初始化完成\n');

while frameIdx <= numFrames
    % 从base workspace获取暂停状态和帧索引
    isPaused = evalin('base', 'isPaused');
    frameIdx = evalin('base', 'frameIdx');
    
    if ~isPaused
        try
            currentFrame = frameIdx;
            assignin('base', 'currentFrame', currentFrame);
            
            % === 同步读取视频帧 ===
            if frameIdx <= numFramesVideo
                frame = videoFrames(frameIdx).data;
            else
                frame = videoFrames(end).data;
            end
            % frame = undistortImage(frame, cameraParams640);
            % 将当前帧和雷达数据存入base workspace，供保存功能使用
            assignin('base', 'frame', frame);
            
            % === 同步读取雷达数据 ===
            if ~isempty(frameMapping) && frameIdx <= length(frameMapping)
                radarFrameIdx = frameMapping(frameIdx);
            else
                radarFrameIdx = frameIdx;
            end
            
            if radarFrameIdx <= numFramesRadar
                K_data = dataStruct.(frames{radarFrameIdx}).BK;
            else
                K_data = [];
            end
            % 提取雷达数据
            [radarPoints_xyz, radarPoints_ravs] = extractRadarData(K_data, config.scaleFactor);
            % 显示帧同步信息
            frameInfoStr = sprintf('视频帧: %d/%d | 雷达帧: %d/%d', ...
                                  frameIdx, numFrames, radarFrameIdx, numFramesRadar);
            
            if isempty(radarPoints_xyz) % 无雷达数据时
                clearDisplay(hScatter1, hVideo, frame);
                title(sprintf('%s | 无雷达数据', frameInfoStr));
                set(hScatter2, 'XData', [], 'YData', []);
                
                % 保存空数据
                saveEmptyData(fid_real, fid_image, frameIdx);
                drawnow;
                pause(config.pauseTime);
                
                % 更新帧索引（仅在不暂停时）
                if ~evalin('base', 'isPaused')
                    frameIdx = frameIdx + 1;
                    assignin('base', 'frameIdx', frameIdx);
                end
                continue;
            end
            % radarPoints_ravs(:,4)= radarPoints_ravs(:,4)./max(radarPoints_ravs(:,4));
            % 距离+速度聚类
            [clusterCenters_xyz, clusterCenters_ravs, clusterLabels, uniqueClusters] = ...
                clusterByDistance(radarPoints_xyz, radarPoints_ravs, ...
                                          config.epsilon, config.minPts, doCluster);
            if isempty(clusterCenters_xyz)
                clearDisplay(hScatter1, hVideo, frame);
                title(sprintf('%s | 无有效聚类', frameInfoStr));
                set(hScatter2, 'XData', [], 'YData', []);
                saveEmptyData(fid_real, fid_image, frameIdx);
                drawnow;
                pause(config.pauseTime);
                % 更新帧索引（仅在不暂停时）
                if ~evalin('base', 'isPaused')
                    frameIdx = frameIdx + 1;
                    assignin('base', 'frameIdx', frameIdx);
                end
                continue;
            end
            clusterDensities = zeros(length(uniqueClusters), 1);
            for i = 1:length(uniqueClusters)
                % 获取每个簇的点索引
                cluster_points = radarPoints_xyz(clusterLabels == uniqueClusters(i), :);
                % 计算该簇的点云密度
                cluster_volume = (config.epsilon^2) * length(cluster_points);  % 假设簇的体积是邻域半径的立方
                clusterDensities(i) = length(cluster_points) / cluster_volume;
            end
            % 投影到图像平面
            [u, v, validIdx] = projectRadarToImage(clusterCenters_xyz, K_camera, ...
                                                   distortion_coefficient, ...
                                                   T_radar2camera, size(frame));
            
            % 只保留有效投影的聚类中心，确保两个文件数据行数一致
            if any(validIdx)
                % 距离用于颜色编码（原始值，0-50）
                distances = min(clusterCenters_ravs(validIdx,1), 50);
                updateScatterPlot(hScatter1, u(validIdx), v(validIdx), distances);
                
                % 只保存有效投影的聚类中心
                radar_xyz_image = [u(validIdx)', v(validIdx)'];
                % 只保存对应行的雷达数据
                clusterCenters_xyzdv = [clusterCenters_xyz(validIdx,:), clusterCenters_ravs(validIdx,:), clusterDensities(validIdx)];
                clusterCenters_xyzdv(:,7)= clusterCenters_xyzdv(:,7)./max(clusterCenters_xyzdv(:,7));
                radar_xyzdv_real = clusterCenters_xyzdv;
                
                % 更新右侧散点图
                x_data = -clusterCenters_xyz(:,2)';
                y_data = clusterCenters_xyz(:,1)';
                c_data = clusterCenters_ravs(:,1)';
                updateScatterPlot(hScatter2, x_data, y_data, c_data);
            else
                radar_xyz_image = [];
                radar_xyzdv_real = [];
                clearDisplay(hScatter1, hVideo, frame);
                title(sprintf('%s | 无有效投影点', frameInfoStr));
                
                % 更新右侧散点图
                x_data = -clusterCenters_xyz(:,2)';
                y_data = clusterCenters_xyz(:,1)';
                c_data = clusterCenters_ravs(:,1)';
                updateScatterPlot(hScatter2, x_data, y_data, c_data);
            end
            
            % 将雷达数据存入base workspace，供保存功能使用
            assignin('base', 'radar_xyzdv_real', radar_xyzdv_real);
            assignin('base', 'radar_xyz_image', radar_xyz_image);

            % 保存当前帧数据
            saveFrameData(fid_real, fid_image, frameIdx, ...
                         radar_xyzdv_real, radar_xyz_image);
            
            % 更新图像和标题
            set(hVideo, 'CData', frame);
            title(sprintf('%s | 点: %d', frameInfoStr, size(clusterCenters_xyz, 1)));
            drawnow;
            pause(config.pauseTime);
            
            % 更新帧索引（仅在不暂停时）
            if ~evalin('base', 'isPaused')
                frameIdx = frameIdx + 1;
                assignin('base', 'frameIdx', frameIdx);
            end
            
        catch ME
            fprintf('处理帧 %d 时出错: %s\n', frameIdx, ME.message);
            if ~evalin('base', 'isPaused')
                frameIdx = frameIdx + 1;
                assignin('base', 'frameIdx', frameIdx);
            end
            continue;
        end
        
    else
        % 完全暂停状态
        title(sprintf('已暂停 | 当前帧: %d/%d', currentFrame, numFrames));
        drawnow;
        pause(0.1);  % 暂停状态下降低CPU占用
    end
end

fclose(fid_real);
fclose(fid_image);
disp('处理完成！');
disp(['处理总帧数: ' num2str(numFrames)]);
disp(['有效雷达帧数: ' num2str(countNonEmptyFrames(dataStruct))]);
disp(['视频总帧数: ' num2str(numFramesVideo)]);

%% 辅助函数 

% 创建雷达帧到视频帧的映射
function frameMapping = createFrameMapping(~, videoFrames, radarFrames)
    numRadarFrames = length(radarFrames);
    numVideoFrames = length(videoFrames);
    
    % 简单的一一映射（假设帧率相同）
    frameMapping = 1:min(numRadarFrames, numVideoFrames);
    
    % 如果有时间戳信息，可以进行更精确的同步
    % 示例：根据时间戳对齐
    % for i = 1:numRadarFrames
    %     radarTime = dataStruct.(radarFrames{i}).timestamp;
    %     % 找到最接近的视频帧...
    % end
end
% 统计有雷达数据的帧数
function count = countNonEmptyFrames(dataStruct)
    frames = fieldnames(dataStruct);
    count = 0;
    
    for i = 1:length(frames)
        K_data = dataStruct.(frames{i}).BK;
        if ~isempty(K_data)
            count = count + 1;
        end
    end
end

% 保存空数据
function saveEmptyData(fid_real, fid_image, frameIdx)
    % 保存空数据
    fprintf(fid_real, '# Frame %d - No Radar Data\n', frameIdx);
    fprintf(fid_image, '# Frame %d - No Image Data\n', frameIdx);
end

% 保存帧数据
function saveFrameData(fid_real, fid_image, frameIdx, radar_xyzdv_real, radar_xyz_image)
    if ~isempty(radar_xyzdv_real) && ~isempty(radar_xyz_image)
        fprintf(fid_real, '# Frame %d - Real Radar Data\n', frameIdx);
        fprintf(fid_real, '# Columns: X, Y, Z, Range, Azimuth, Velocity, Signal, Density\n');
        fprintf(fid_image, '# Frame %d - Image Coordinates\n', frameIdx);
        fprintf(fid_image, '# Columns: u, v\n');
        for i = 1:size(radar_xyzdv_real, 1)
            fprintf(fid_real, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', radar_xyzdv_real(i, :));
        end
        switch size(radar_xyz_image, 2)
            case 2
                format_str = '%.6f %.6f\n';
            case 6
                format_str = '%.6f %.6f %.6f %.6f %.6f %.6f\n';
                fprintf(fid_image, '# Columns: u, v, roi_x, roi_y, roi_width, roi_height\n');
            otherwise
                format_str = '%.6f %.6f\n';
        end
        for i = 1:size(radar_xyz_image, 1)
            fprintf(fid_image, format_str, radar_xyz_image(i, :));
        end
    elseif ~isempty(radar_xyzdv_real) && isempty(radar_xyz_image)
        saveEmptyData(fid_real, fid_image, frameIdx);
    end

end

function updateScatterPlot(hScatter, u, v, distances)
    if ~isempty(distances)
        % 确保distances是列向量
        if isrow(distances)
            distances = distances';
        end
        set(hScatter, 'XData', u, 'YData', v, 'CData', distances);
    else
        % 清空散点图：确保所有数据属性都一致
        set(hScatter, 'XData', [], 'YData', [], 'CData', []);
    end
end

% 清空显示 
function clearDisplay(hScatter, hVideo, frame)
    
    set(hScatter, 'XData', [], 'YData', [], 'CData', []);
    % 更新视频图像
    set(hVideo, 'CData', frame);
end

function [u, v, validIdx, camera_points] = projectRadarToImage(radarPoints, K_camera, ...
                                                               distortion_coefficient, ...
                                                               T_radar2camera, imageSize)

    if isempty(radarPoints)
        u = []; v = []; validIdx = []; camera_points = [];
        return;
    end
    
    % 参数提取（提高可读性）
    k1 = distortion_coefficient(1);
    k2 = distortion_coefficient(2);
    p1 = distortion_coefficient(3);
    p2 = distortion_coefficient(4);
    fx = K_camera(1,1);
    fy = K_camera(2,2);
    cx = K_camera(1,3);
    cy = K_camera(2,3);
    
    % 1. 转换到相机坐标系（使用矩阵乘法避免转置）
    num_points = size(radarPoints, 1);
    radar_homogeneous = [radarPoints, ones(num_points, 1)];  % M×4
    camera_points_homogeneous = (T_radar2camera * radar_homogeneous')';  % M×4
    camera_points = camera_points_homogeneous(:, 1:3);  % M×3
    
    % 2. 提取坐标并进行归一化（向量化操作）
    z = camera_points(:, 3);
    z_valid = z;
    z_valid(z <= 0) = inf;
    
    x_norm = camera_points(:, 1) ./ z_valid;
    y_norm = camera_points(:, 2) ./ z_valid;
    
    % 3. 畸变校正（向量化计算）
    r2 = x_norm.^2 + y_norm.^2;
    radial = 1 + k1 * r2 + k2 * r2.^2;
    
    % 径向畸变
    x_radial = x_norm .* radial;
    y_radial = y_norm .* radial;
    
    % 切向畸变（合并计算）
    xy = x_norm .* y_norm;
    x_tangent = 2 * p1 * xy + p2 * (r2 + 2 * x_norm.^2);
    y_tangent = p1 * (r2 + 2 * y_norm.^2) + 2 * p2 * xy;
    
    % 总畸变
    x_distorted = x_radial + x_tangent;
    y_distorted = y_radial + y_tangent;
    
    % 4. 像素坐标转换
    u = fx * x_distorted + cx +15;
    v = fy * y_distorted + cy;
    
    % 5. 筛选有效点
    valid_z = z > 0;
    if nargin >= 5 && ~isempty(imageSize)
        % 向量化的边界检查
        valid_uv = (u >= 1) & (u <= imageSize(2)) & ...
                   (v >= 1) & (v <= imageSize(1));
        validIdx = valid_z & valid_uv;
    else
        validIdx = valid_z;
    end
    % 确保输出为行向量（与原始函数保持一致）
    u = u(:)';
    v = v(:)';
end

function [radarPoints_xyz, radarPoints_rav] = extractRadarData(K_data, scaleFactor)
    radarPoints_xyz = [];
    radarPoints_rav = [];
    
    if isempty(K_data)
        return;
    end
    
    targetIDs = fieldnames(K_data);
    
    for j = 1:numel(targetIDs)
        target = K_data.(targetIDs{j});
        
        % 过滤静止目标
        if target.V == 0
            continue;
        end
        if target.R < 1
            continue;
        end
        % 球坐标转换到笛卡尔坐标
        S_target = target.P;
        R_target = target.R;
        A = deg2rad(target.A);
        E = deg2rad(target.E);
        
        x = R_target * cos(E) * cos(A);
        y = R_target * cos(E) * sin(A);
        z = scaleFactor * R_target ; % 高度映射
        
        radarPoints_xyz = [radarPoints_xyz; x, y, z];
        radarPoints_rav = [radarPoints_rav; target.R,A,target.V,S_target];
    end
end

function keyControl(~, event)
    persistent pauseState
    if isempty(pauseState)
        pauseState = false;
    end
    % 获取保存路径
    try
        savePath = evalin('base', 'E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\data\day_data');
    catch
        savePath = fullfile('E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion', '.\data\day_data');
        assignin('base', 'savePath', savePath);
    end
    % 确保保存文件夹存在
    if ~exist(savePath, 'dir')
        mkdir(savePath);
    end

    % 从base workspace获取当前状态
    try
        currentFrame = evalin('base', 'currentFrame');
        numFrames = evalin('base', 'numFrames');
    catch
        currentFrame = 1;
        numFrames = 1;
    end

    switch event.Key
        case 'space'
            pauseState = ~pauseState;
            assignin('base', 'isPaused', pauseState);
            
            if pauseState
                fprintf('已暂停 | 当前帧: %d/%d\n', currentFrame, numFrames);
                
                % 暂停时显示当前帧信息
                try
                    h = findobj('Name', 'Radar-Image Fusion');
                    if ~isempty(h)
                        ax = findobj(h, 'Type', 'axes');
                        if length(ax) >= 1
                            title(ax(1), sprintf('已暂停 | 当前帧: %d/%d', currentFrame, numFrames));
                        end
                    end
                catch
                end
            else
                fprintf('继续播放\n');
            end
            
        case 'rightarrow'  % 单帧前进
            if pauseState
                try
                    frameIdx = evalin('base', 'frameIdx');
                    numFrames = evalin('base', 'numFrames');
                    
                    if frameIdx < numFrames
                        frameIdx = frameIdx + 1;
                        assignin('base', 'frameIdx', frameIdx);
                        fprintf('前进到帧: %d/%d\n', frameIdx, numFrames);
                    else
                        fprintf('已到达最后一帧\n');
                    end
                catch ME
                    fprintf('单帧前进失败: %s\n', ME.message);
                end
            else
                fprintf('请先暂停再使用单帧前进\n');
            end
            
        case 'leftarrow'  % 单帧后退
            if pauseState
                try
                    frameIdx = evalin('base', 'frameIdx');
                    
                    if frameIdx > 1
                        frameIdx = frameIdx - 1;
                        assignin('base', 'frameIdx', frameIdx);
                        fprintf('后退到帧: %d/%d\n', frameIdx, numFrames);
                    else
                        fprintf('已到达第一帧\n');
                    end
                catch ME
                    fprintf('单帧后退失败: %s\n', ME.message);
                end
            else
                fprintf('请先暂停再使用单帧后退\n');
            end
            
        case 's'  % 保存功能
            if pauseState
                try
                    frameIdx = evalin('base', 'currentFrame');
                    
                    % 从base workspace获取数据
                    try
                        frame = evalin('base', 'frame');  % 当前帧图像
                        
                        % 保存当前帧图像为PNG
                        imgFilename = sprintf('%04d_frame.png', frameIdx);
                        imgPath = fullfile(savePath, imgFilename);
                        imwrite(frame, imgPath);
                        fprintf('已保存图像: %s\n', imgFilename);
                        
                    catch ME1
                        fprintf('保存图像失败: %s\n', ME1.message);
                    end
                    
                    % 尝试获取并保存点云数据
                    try
                        pointCloud = evalin('base', 'radar_xyzdv_real');  % 点云数据
                        if ~isempty(pointCloud)
                            pcFilename1 = sprintf('%04d_point_xyzrav_cloud.txt', frameIdx);
                            pcPath1 = fullfile(savePath, pcFilename1);
                            dlmwrite(pcPath1, pointCloud, 'delimiter', ' ', 'precision', '%.6f');
                            fprintf('已保存点云数据: %s\n', pcFilename1);
                        end
                        
                        ptCloud = evalin('base', 'radar_xyz_image');
                        if ~isempty(ptCloud)
                            pcFilename2 = sprintf('%04d_point_uvroi_image.txt', frameIdx);
                            pcPath2 = fullfile(savePath, pcFilename2);
                            dlmwrite(pcPath2, ptCloud, 'delimiter', ' ', 'precision', '%.6f');
                            fprintf('已保存图像坐标: %s\n', pcFilename2);
                        end
                        
                    catch ME2
                        fprintf('保存点云数据失败: %s\n', ME2.message);
                    end
                    
                    fprintf('保存完成！文件保存在: %s\n', savePath);
                    
                catch ME
                    fprintf('保存失败: %s\n', ME.message);
                end
            else
                fprintf('请在暂停状态下使用保存功能\n');
            end
            
        case 'r'  % 重置到第一帧（新增功能）
            if pauseState
                assignin('base', 'frameIdx', 1);
                fprintf('已重置到第一帧\n');
            end
            
        case 'escape'  % 退出程序（新增功能）
            choice = questdlg('确定要退出程序吗？', '退出确认', '是', '否', '否');
            if strcmp(choice, '是')
                disp('程序退出');
                close(gcf);
                return;
            end
    end
end


%-----------s
function [clusterCenters_xyz, clusterCenters_rav, clusterLabels, uniqueClusters] = clusterByDistance(radarPoints_xyz, radarPoints_rav, epsilon, minPts, doCluster)

    if isempty(radarPoints_xyz)
        clusterCenters_xyz = [];
        clusterCenters_rav = [];
        clusterLabels = [];
        uniqueClusters = [];
        return;
    end

    % 使用 DBSCAN 聚类
    [clusterCenters_xyz, clusterCenters_rav, clusterLabels] = radarDBSCAN(radarPoints_xyz, radarPoints_rav, epsilon, minPts, doCluster);
    
    % 获取有效聚类（仅作为额外输出）
    uniqueClusters = unique(clusterLabels(clusterLabels > 0));
end

function [clusterCenters_xyz, clusterCenters_rav, clusterLabels] = radarDBSCAN(radarPoints_xyz, radarPoints_rav, epsilon, minPts, doCluster)

    if isempty(radarPoints_xyz)
        clusterCenters_xyz = [];
        clusterCenters_rav = [];
        clusterLabels = [];
        return;
    end
    
    % 如果开关为false，不进行聚类，返回所有点作为独立聚类
    if ~doCluster
        clusterCenters_xyz = radarPoints_xyz;
        clusterCenters_rav = radarPoints_rav;
        clusterLabels = ones(size(radarPoints_xyz, 1), 1); % 不聚类时，给所有点一个标签1
        return;
    end
    
    % 1. 特征工程：位置（xy） + 距离 + 速度 + 角度信息 [positions, ranges, angles, v_norm];
    features = createRadarFeatures(radarPoints_xyz, radarPoints_rav);
    
    % 2. 运行DBSCAN
    clusterLabels = dbscan_radar(features, epsilon, minPts);
    
    % 3. 计算聚类中心
    uniqueClusters = unique(clusterLabels(clusterLabels > 0));
    clusterCenters_xyz = [];
    clusterCenters_rav = [];
    if isempty(uniqueClusters)
        clusterCenters_xyz = [];
        clusterCenters_rav = [];
    end
    for c = uniqueClusters'
        idx = (clusterLabels == c);
        
        % 计算加权中心（距离越远权重越小）
        weights = 1 ./ radarPoints_rav(idx, 1);
        weights = weights / sum(weights);
        
        center_x = sum(radarPoints_xyz(idx, 1) .* weights);
        center_y = sum(radarPoints_xyz(idx, 2) .* weights);
        center_z = sum(radarPoints_xyz(idx, 3) .* weights);
        center_r = sum(radarPoints_rav(idx, 1) .* weights);
        center_a = sum(radarPoints_rav(idx, 2) .* weights);
        center_v = sum(radarPoints_rav(idx, 3) .* weights);
        center_s = sum(radarPoints_rav(idx, 4) .* weights);

        clusterCenters_xyz = [clusterCenters_xyz; center_x, center_y, center_z];
        clusterCenters_rav = [clusterCenters_rav; center_r, center_a, center_v, center_s];
    end

    function features = createRadarFeatures(radarPoints_xyz, radarPoints_rav)
        % 位置特征
        positions = radarPoints_xyz(:, 1:2);
        % 距离特征
        ranges = radarPoints_rav(:,1);
        % 角度特征（方位角）
        angles = radarPoints_rav(:,2);
        % 速度特征
        v_norm = normalizeVelocity(radarPoints_rav(:,3));
        % 速度特征
        s_norm = radarPoints_rav(:,4);
        % 组合特征
        features = [positions, ranges, angles, v_norm,s_norm];
        % 归一化
        if(length(features(:,1)) > 1)
            features = (features - mean(features, 1)) ./ (std(features, 0, 1) + eps);
        end
    
        function v_norm = normalizeVelocity(velocities)
        % 速度归一化
            v_mean = mean(abs(velocities));
            v_norm = velocities / (v_mean + eps);
        end
    end
end

function labels = dbscan_radar(features, epsilon, minPts)
    n = size(features, 1);
    labels = zeros(n, 1);
    visited = false(n, 1);
    clusterId = 0;
    
    % 预计算距离矩阵（小数据集）
    if n <= 5000
        distances = pdist2(features(:,1:2), features(:,1:2), 'euclidean');
    else
        distances = [];
    end
    %欧几里得距离（不标准化）euclidean
    %标准化欧几里得距离 seuclidean 
    %使用余弦相似度 cosine
    %使用曼哈顿距离 cityblock 
    for i = 1:n
        if visited(i)
            continue;
        end
        

        visited(i) = true;
        
        % 查找邻域
        if ~isempty(distances)
            neighbors = find(distances(i, :) <= epsilon);
        else
            neighbors = findNeighborsEuclidean(features(:,1:2), i, epsilon);
        end
        
        if length(neighbors) < minPts
            labels(i) = -1;
        else
            clusterId = clusterId + 1;
            labels(i) = clusterId;
            
            % 扩展聚类
            k = 1;
            while k <= length(neighbors)
                pointIdx = neighbors(k);
                
                if ~visited(pointIdx)
                    visited(pointIdx) = true;
                    
                    % 查找当前点的邻域
                    if ~isempty(distances)
                        newNeighbors = find(distances(pointIdx, :) <= epsilon);

                    else
                        newNeighbors = findNeighborsEuclidean(features(:,1:2), pointIdx, epsilon);

                    end
                    if length(newNeighbors) >= minPts
                        neighbors = [neighbors, setdiff(newNeighbors, neighbors)];
                    end
                end
                
                if labels(pointIdx) <= 0
                    labels(pointIdx) = clusterId;
                end
                
                k = k + 1;
            end
        end
    end
end

