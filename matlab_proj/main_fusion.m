%--------------------------------------------------------------------------
% main_fusion.m - 雷达-图像融合主脚本
% 功能：整合projdbscan、detectWithYolov5、chaparis和ds_fusion
% 流程：
% 1. 读取雷达数据和视频
% 2. 对每帧执行projdbscan进行点云投影
% 3. 对每帧执行detectWithYolov5进行图像增强和目标检测
% 4. 对结果执行chaparis进行匹配
% 5. 最后执行ds_fusion进行融合
%--------------------------------------------------------------------------
clc;
clear;
close all;

%% 1. 配置参数
config = struct();

% 数据处理参数
config.scaleFactor = 0.01;      % 高度映射因子
config.epsilon = 1;           % DBSCAN聚类距离阈值
config.minPts = 2;              % DBSCAN最小点数
config.doCluster = true;

% 相机参数文件
config.cameraParamsFile = 'E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\Radar\radar_test\cameraParams640.mat';

% 雷达到相机的变换矩阵
config.T_radar2camera = [0, -1,  0,  0.35;  % R + T [x,y,z]偏移
                         0,  0, -1,    0;
                         1,  0,  0,    0;
                         0,  0,  0,    1];

% 选择文件夹
folderPath = uigetdir('E:\hducc\Data_process\radar_data\data','选择图片文件夹');
if folderPath == 0
    disp('未选择文件夹，程序终止。');
    return;
end

config.radarDataPath = fullfile(folderPath, 'AKBK_data_2d.mat');
config.videoPath     = fullfile(folderPath, 'video10.mp4');
fileName = split(folderPath,'\');
config.name = string(fileName(6));

% 输出目录
config.data_dir = 'complete_data';
config.out_data_dir = fullfile(config.data_dir, config.name);

% 确保输出目录存在
if ~exist(config.data_dir, 'dir')
    mkdir(config.data_dir);
end
if ~exist(config.out_data_dir, 'dir')
    mkdir(config.out_data_dir);
end

%% 2. 加载相机参数
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

%% 3. 读取雷达数据和视频
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
catch ME
    error('无法打开视频文件: %s\n错误信息: %s', config.videoPath, ME.message);
end

% 帧同步：取较小的帧数，确保同步
numFramesVideo = vReader.NumFrames;
numFrames = min(numFramesRadar, numFramesVideo);
fprintf('\n帧同步信息:\n');
fprintf('  雷达帧数: %d\n', numFramesRadar);
fprintf('  视频帧数: %d\n', numFramesVideo);
fprintf('  实际处理帧数: %d\n', numFrames);

%% 4. 初始化YOLOv5检测器
modelPath = 'models/yolov5s_cp3.mat';
classNames = helper.getCOCOClassNames1;
if gpuDeviceCount > 0
    executionEnvironment = 'gpu';
else
    executionEnvironment = 'cpu';
end
try
    fprintf('初始化YOLOv5检测器...\n');
    detector = initYolov5Detector(modelPath,classNames,executionEnvironment);
    fprintf('✓ YOLOv5检测器初始化完成\n');
catch ME
    error('无法初始化YOLOv5检测器: %s', ME.message);
end

%% 5. 预分配存储和文件句柄
frame_detections = struct('frame', {}, 'targets', {});
frame_radar_data = struct('frame', {}, 'radar_points', {});
frame_image_data = struct('frame', {}, 'image_points', {});

% 时间统计变量
frame_times = zeros(numFrames, 1);  % 每一帧的处理时间
frame_targets = zeros(numFrames, 1); % 每一帧的目标数
final_targets = zeros(numFrames, 1); % 每一帧最终检测到的目标数（融合后）

% 打开文件句柄
 detection_file = fullfile(config.out_data_dir, sprintf('%s_detection.txt', config.name));
fid_det = fopen(detection_file, 'w');

real_file = fullfile(config.out_data_dir, sprintf('%s_frame_real.txt', config.name));
fid_real = fopen(real_file, 'w');

image_file = fullfile(config.out_data_dir, sprintf('%s_frame_image.txt', config.name));
fid_image = fopen(image_file, 'w');

% 打开融合结果日志文件
fusion_log_file = fullfile(config.out_data_dir, sprintf('%s_fusion_result.txt', config.name));
fid_fusion = fopen(fusion_log_file, 'w');
fprintf(fid_fusion, '================ 融合结果日志 ================\n');
fprintf(fid_fusion, '开始时间: %s\n', datestr(now));
fprintf(fid_fusion, '列定义: [X, Y, W, H, 置信度, 类型]\n');
fprintf(fid_fusion, '类型代码: 1=融合(Fused), 2=纯雷达(RadarOnly), 3=纯视觉(VisionOnly)\n');
%% 6. 逐帧处理
fprintf('\n开始逐帧处理...\n');
for frameIdx = 1:numFrames
    try
        
        % 读取视频帧
        vReader.CurrentTime = (frameIdx-1) / vReader.FrameRate;
        frame = readFrame(vReader);
        imageSize = size(frame);
        
        % 读取雷达数据
        if frameIdx <= numFramesRadar
            K_data = dataStruct.(frames{frameIdx}).BK;
        else
            K_data = [];
        end
        % 记录帧开始时间
        frame_start_time = tic;
        fprintf('处理第 %d/%d 帧...\n', frameIdx, numFrames);
        
        % 1. 执行点云投影
        [radar_xyz_image, radar_xyzdv_real] = projdbscan(K_data, config, K_camera, distortion_coefficient, T_radar2camera, imageSize);
        
        % 2. 执行目标检测
        [bboxes, scores, labelIds] = detectWithYolov5(detector, frame);
        
        % 记录目标数
        num_targets = size(bboxes, 1);
        frame_targets(frameIdx) = num_targets;
        
        % 初始化targets为空结构体
        targets = struct('type', {}, 'conf', {}, 'bbox', {});
        
        % 保存检测结果
        if ~isempty(bboxes)
            for i = 1:size(bboxes, 1)
                targets(i).type = 'car'; % 简化处理，实际应该根据labelIds映射
                targets(i).conf = scores(i);
                targets(i).bbox = bboxes(i, :);
            end
            frame_detections(end+1).frame = frameIdx;
            frame_detections(end).targets = targets;
            
            % 实时保存检测结果
            fprintf(fid_det, '=== 第%d帧 ===\n', frameIdx);
            fprintf(fid_det, '目标数: %d\n', length(targets));
            for j = 1:length(targets)
                target = targets(j);
                fprintf(fid_det, '目标%d 类型: %s 置信度：%.4f\n', j, target.type, target.conf);
                fprintf(fid_det, '目标%d 位置: %.2f %.2f %.2f %.2f\n', j, target.bbox(1), target.bbox(2), target.bbox(3), target.bbox(4));
            end
        else
            % 保存空检测结果
            fprintf(fid_det, '=== 第%d帧 ===\n', frameIdx);
            fprintf(fid_det, '目标数: 0\n');
        end
        
        % 保存雷达数据
        if ~isempty(radar_xyzdv_real)
            frame_radar_data(end+1).frame = frameIdx;
            frame_radar_data(end).radar_points = radar_xyzdv_real;
            
            % 实时保存雷达真实数据
            fprintf(fid_real, '# Frame %d - Real Radar Data\n', frameIdx);
            fprintf(fid_real, '# Columns: X, Y, Z, Range, Azimuth, Velocity, Signal, Density\n');
            for j = 1:size(radar_xyzdv_real, 1)
                fprintf(fid_real, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', radar_xyzdv_real(j, :));
            end
        end
        
        % 保存图像坐标数据
        if ~isempty(radar_xyz_image)
            frame_image_data(end+1).frame = frameIdx;
            frame_image_data(end).image_points = radar_xyz_image;
            
            % 实时保存图像坐标数据
            fprintf(fid_image, '# Frame %d - Image Coordinates\n', frameIdx);
            fprintf(fid_image, '# Columns: u, v\n');
            for j = 1:size(radar_xyz_image, 1)
                fprintf(fid_image, '%.6f %.6f\n', radar_xyz_image(j, :));
            end
        end
        
        % 检查是否有目标或点云数据
        has_targets = ~isempty(bboxes);
        has_pointcloud = ~isempty(radar_xyzdv_real) && ~isempty(radar_xyz_image);
        
        % 如果既没有目标也没有点云，跳过后续处理
        if ~has_targets && ~has_pointcloud
            fprintf('  无目标和点云数据，跳过该帧\n');
            % 记录处理时间
            frame_times(frameIdx) = toc(frame_start_time);
            continue;
        end
        
        % 3. 执行chaparis匹配
        fprintf('  执行chaparis匹配...\n');
        fusion_input = [];
        try
            fusion_input = chaparis(config, frameIdx, frame, targets, radar_xyzdv_real, radar_xyz_image);
        catch ME
            fprintf('  chaparis执行出错: %s\n', ME.message);
        end
        
        % 4. 执行ds_fusion融合
        fprintf('  执行ds_fusion融合...\n');
        try
            ds_result = ds_fusion(config, frameIdx, fusion_input, targets, radar_xyz_image);
            
            % 记录最终检测到的目标数
            num_objs = size(ds_result, 1);
            final_targets(frameIdx) = num_objs;
            
            % 保存融合结果到日志
            if num_objs > 0
                fprintf(fid_fusion, 'Frame %d: 检测到 %d 个目标\n', frameIdx, num_objs);
                for k = 1:num_objs
                    type_str = '未知';
                    switch ds_result(k, 6)
                        case 1, type_str = 'Fused';
                        case 2, type_str = 'RadarOnly';
                        case 3, type_str = 'VisionOnly';
                    end
                    
                    fprintf(fid_fusion, '    ID:%-2d Type:%-10s Pos:[%6.1f, %6.1f] Size:[%4.1f, %4.1f] Conf:%.4f\n', ...
                            k, type_str, ds_result(k,1), ds_result(k,2), ...
                            ds_result(k,3), ds_result(k,4), ds_result(k,5));
                end
            else
                fprintf(fid_fusion, 'Frame %d: 无有效目标\n', frameIdx);
            end
            
            % 实时可视化ds融合结果（视频流形式）
            % 无论有无目标，每一帧都更新显示
            % 复制原始帧用于显示
            display_frame = frame;
            
            % 检查图形窗口是否存在，如果不存在则创建
            if ~exist('fusionFig', 'var') || ~ishandle(fusionFig)
                fusionFig = figure('Name', 'DS Fusion Result', 'Position', [200, 200, 800, 600]);
            end
            
            % 确保窗口可见
            figure(fusionFig);
            clf; % 清除当前窗口内容
            
            % 显示图像
            imshow(display_frame);
            hold on;
            
            % 绘制融合结果（如果有目标）
            if num_objs > 0
                for k = 1:num_objs
                    x = ds_result(k, 1);
                    y = ds_result(k, 2);
                    w = ds_result(k, 3);
                    h = ds_result(k, 4);
                    conf = ds_result(k, 5);
                    type = ds_result(k, 6);
                    
                    % 根据类型选择颜色
                    switch type
                        case 1, color = [0, 1, 0]; % 融合目标 - 绿色
                        case 2, color = [1, 0, 0]; % 纯雷达目标 - 红色
                        case 3, color = [0, 0, 1]; % 纯视觉目标 - 蓝色
                        otherwise, color = [0.5, 0.5, 0.5]; % 未知类型 - 灰色
                    end
                    
                    % 计算边界框坐标
                    x1 = x - w/2;
                    y1 = y - h/2;
                    x2 = x + w/2;
                    y2 = y + h/2;
                    
                    % 绘制边界框
                    rectangle('Position', [x1, y1, w, h], 'EdgeColor', color, 'LineWidth', 2);
                    
                    % 显示类型和置信度
                    type_str = '';
                    switch type
                        case 1, type_str = 'Fused';
                        case 2, type_str = 'Radar';
                        case 3, type_str = 'Vision';
                    end
                    text(x1, y1-10, sprintf('%s: %.2f', type_str, conf), 'Color', color, 'FontSize', 10, 'BackgroundColor', 'white');
                end
            else
                % 显示无目标信息
                text(10, 60, 'No valid targets detected', 'Color', 'red', 'FontSize', 12, 'BackgroundColor', 'white');
            end
            
            % 显示帧号和处理时间
            text(10, 20, sprintf('Frame: %d/%d', frameIdx, numFrames), 'Color', 'white', 'FontSize', 12, 'BackgroundColor', 'black');
            text(10, 40, sprintf('Processing Time: %.2f ms', frame_times(frameIdx)*1000), 'Color', 'white', 'FontSize', 12, 'BackgroundColor', 'black');
            
            % 设置标题
            title('DS Fusion Result');
            hold off;
            drawnow;
        catch ME
            fprintf('  ds_fusion执行出错: %s\n', ME.message);
        end
        
        % 记录帧结束时间
        frame_times(frameIdx) = toc(frame_start_time);
        fprintf('  处理时间: %.4f 秒\n', frame_times(frameIdx));
        
    catch ME
        fprintf('处理第 %d 帧时出错: %s\n', frameIdx, ME.message);
        % 记录错误帧的处理时间
        frame_times(frameIdx) = toc(frame_start_time);
        continue;
    end
end

% 关闭文件句柄
fclose(fid_det);
fclose(fid_real);
fclose(fid_image);
fclose(fid_fusion);

fprintf('✓ 中间结果保存完成\n');
fprintf('✓ 融合结果日志保存完成\n');

%% 7. 时间统计和可视化
fprintf('\n================ 时间统计 =================\n');

% 转换时间单位为毫秒
frame_times_ms = frame_times * 1000;  % 转换为毫秒
valid_frames = frame_times_ms > 0;

% 统计所有帧（包括第一帧）
avg_frame_time_ms = mean(frame_times_ms(valid_frames));
total_processing_time_ms = sum(frame_times_ms);
num_valid_frames = sum(valid_frames);

fprintf('总处理时间: %.2f ms\n', total_processing_time_ms);
fprintf('平均每帧处理时间: %.2f ms\n', avg_frame_time_ms);
if num_valid_frames > 0
    fprintf('处理帧率: %.2f FPS\n', num_valid_frames / (total_processing_time_ms / 1000));
end

% 按最终检测到的目标数统计处理时间（包括第一帧）
unique_final_target_counts = unique(final_targets);
time_by_final_target_count = zeros(length(unique_final_target_counts), 2);

for i = 1:length(unique_final_target_counts)
    final_target_count = unique_final_target_counts(i);
    mask = final_targets == final_target_count;
    if any(mask)
        avg_time_ms = mean(frame_times_ms(mask));
        time_by_final_target_count(i, :) = [final_target_count, avg_time_ms];
        fprintf('最终检测目标数 %d: 平均处理时间 %.2f ms\n', final_target_count, avg_time_ms);
    end
end

% 为子图3准备数据（排除第一帧）
if numFrames > 1
    final_targets_no_first = final_targets(2:end);
    frame_times_ms_no_first = frame_times_ms(2:end);
    % 按最终检测到的目标数统计处理时间（排除第一帧）
    unique_final_target_counts_no_first = unique(final_targets_no_first);
    time_by_final_target_count_no_first = zeros(length(unique_final_target_counts_no_first), 2);
    
    for i = 1:length(unique_final_target_counts_no_first)
        final_target_count = unique_final_target_counts_no_first(i);
        mask = final_targets_no_first == final_target_count;
        if any(mask)
            avg_time_ms = mean(frame_times_ms_no_first(mask));
            time_by_final_target_count_no_first(i, :) = [final_target_count, avg_time_ms];
        end
    end
else
    final_targets_no_first = [];
    frame_times_ms_no_first = [];
    time_by_final_target_count_no_first = [];
end

% 生成统计图
figure('Name', '处理时间统计', 'Position', [100, 100, 1000, 600]);

% 子图1: 每帧处理时间
subplot(2, 2, 1);
plot(1:numFrames, frame_times_ms, 'b-', 'LineWidth', 1.5);
hold on;
if numFrames > 1
    plot([1, numFrames], [avg_frame_time_ms, avg_frame_time_ms], 'r--', 'LineWidth', 1.5);
end
hold off;
xlabel('帧索引');
ylabel('处理时间 (ms)');
title('每帧处理时间');
if numFrames > 1
    legend('实际时间', '平均时间 (排除第一帧)');
else
    legend('实际时间');
end
grid on;

% 子图2: 最终检测目标数分布
subplot(2, 2, 2);
histogram(final_targets, 'BinMethod', 'integers');
xlabel('最终检测目标数');
ylabel('帧数');
title('最终检测目标数分布');
grid on;

% 子图3: 最终检测目标数与处理时间关系（排除第一帧）
subplot(2, 2, 3);
if numFrames > 1
    scatter(final_targets_no_first, frame_times_ms_no_first, 10, 'b.');
    hold on;
    if ~isempty(time_by_final_target_count_no_first)
        sorted_indices = sortrows(time_by_final_target_count_no_first, 1);
        plot(sorted_indices(:, 1), sorted_indices(:, 2), 'r-', 'LineWidth', 2);
    end
    hold off;
else
    % 如果只有一帧，显示提示信息
    text(0.5, 0.5, '数据不足', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    axis off;
end
xlabel('最终检测目标数');
ylabel('处理时间 (ms)');
title('最终检测目标数与处理时间关系（排除第一帧）');
grid on;

% 子图4: 处理时间分布
subplot(2, 2, 4);
histogram(frame_times_ms, 20);
xlabel('处理时间 (ms)');
ylabel('帧数');
title('处理时间分布');
grid on;

% 保存统计结果为TXT文件
stats_txt_file = fullfile(config.out_data_dir, sprintf('%s_processing_stats.txt', config.name));
fid_stats = fopen(stats_txt_file, 'w');
fprintf(fid_stats, '================ 处理时间统计 =================\n');
fprintf(fid_stats, '总处理时间: %.2f ms\n', total_processing_time_ms);
fprintf(fid_stats, '平均每帧处理时间: %.2f ms\n', avg_frame_time_ms);
if num_valid_frames > 0
    fprintf(fid_stats, '处理帧率: %.2f FPS\n', num_valid_frames / (total_processing_time_ms / 1000));
end
fprintf(fid_stats, '\n按最终检测目标数统计 (所有帧):\n');
for i = 1:length(unique_final_target_counts)
    final_target_count = unique_final_target_counts(i);
    mask = final_targets == final_target_count;
    if any(mask)
        avg_time_ms = mean(frame_times_ms(mask));
        count = sum(mask);
        fprintf(fid_stats, '最终检测目标数 %d: 平均处理时间 %.2f ms (共 %d 帧)\n', final_target_count, avg_time_ms, count);
    end
end

if numFrames > 1
    fprintf(fid_stats, '\n按最终检测目标数统计 (排除第一帧):\n');
    for i = 1:length(unique_final_target_counts_no_first)
        final_target_count = unique_final_target_counts_no_first(i);
        mask = final_targets_no_first == final_target_count;
        if any(mask)
            avg_time_ms = mean(frame_times_ms_no_first(mask));
            count = sum(mask);
            fprintf(fid_stats, '最终检测目标数 %d: 平均处理时间 %.2f ms (共 %d 帧)\n', final_target_count, avg_time_ms, count);
        end
    end
end
fprintf(fid_stats, '\n每帧详细数据:\n');
fprintf(fid_stats, '帧索引\t原始目标数\t最终检测目标数\t处理时间(ms)\n');
for i = 1:numFrames
    fprintf(fid_stats, '%d\t%d\t%d\t%.2f\n', i, frame_targets(i), final_targets(i), frame_times_ms(i));
end
fclose(fid_stats);

% 保存统计图
figure_path = fullfile(config.out_data_dir, 'processing_stats');
if ~exist(figure_path, 'dir')
    mkdir(figure_path);
end
figure_file = fullfile(figure_path, 'processing_time_stats.png');
saveas(gcf, figure_file);

fprintf('\n✓ 时间统计完成\n');
fprintf('✓ 统计结果保存为TXT格式\n');
fprintf('✓ 统计图保存完成\n');

fprintf('\n处理完成！\n');
