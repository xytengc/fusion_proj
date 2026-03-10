%--------------------------------------------------------------------------
% compare_all_methods.m - 多传感器融合对比实验评估系统
% 功能：对比 单视觉(YOLO) vs 单雷达(Radar) vs DS融合(Fusion) 的性能
%--------------------------------------------------------------------------
clear; close all; clc;
% day_data night_data
%% 1. 数据集与路径配置
config = struct();
config.dist_threshold = 50; % 允许的中心偏差距离 (像素)

% === 定义数据集列表 (添加您的所有视频片段) ===
datasets = [
    % 黑夜
     struct('name', 'cycle', 'type', 'night'),
     struct('name', 'stright', 'type', 'night'),
     struct('name', 'load_600', 'type', 'night'),
     struct('name', 'suv_pass', 'type', 'night'),
    % 正常光
     struct('name', 'day_loadlowlight', 'type', 'day'),
];

% 自动获取每个数据集的实际帧号
for k = 1:length(datasets)
    d = datasets(k);
    % 根据数据集类型选择对应的基本目录
    if strcmpi(d.type, 'day')
        base_dir = fullfile('data', 'day_data');
    else
        base_dir = fullfile('data', 'night_data');
    end
    img_dir = fullfile(base_dir, d.name, 'image');
    datasets(k).frames = get_actual_frames_from_images(img_dir, d.name);
end

%% 2. 初始化统计变量
% 分别记录白天和夜晚的三种算法表现
methods = {'YOLO', 'Radar', 'Fusion'};
stats_day = init_method_stats(methods);
stats_night = init_method_stats(methods);

%% 3. 批量评估循环
fprintf('================ 开始对比实验评估 ================\n');

for k = 1:length(datasets)
    d = datasets(k);
    fprintf('\n>>> 处理视频片段: [%s] (%s) <<<\n', d.name, d.type);
    
    % --- A. 预加载当前视频的所有原始数据 (加速处理) ---
    % 假设 txt 文件位于 data/night_data/[video_name]_detection.txt
    % 或者 data/night_data/[video_name]/[video_name]_detection.txt
    % 这里按您之前的 cycle_detection.txt 格式适配
    
    % 根据数据集类型选择对应的基本目录
    if strcmpi(d.type, 'day')
        base_dir = fullfile('data', 'day_data');
    else
        base_dir = fullfile('data', 'night_data');
    end
    
    % 1. 视觉检测文件路径 - 根据数据集类型选择不同的后缀
    if strcmpi(d.type, 'day')
        det_suffix = '_detection.txt';
    else
        det_suffix = '_detection_x.txt';
    end
    det_file = fullfile(base_dir, [d.name, det_suffix]); 
    if ~exist(det_file, 'file') % 尝试在子文件夹找
        det_file = fullfile(base_dir, d.name, [d.name, det_suffix]);
    end
    % 2. 雷达数据文件路径
    rad_file = fullfile(base_dir, [d.name, '_frame_image.txt']);
    if ~exist(rad_file, 'file')
        rad_file = fullfile(base_dir, d.name, [d.name, '_frame_image.txt']);
    end
    
    % 加载原始数据
    raw_yolo_data = parse_detection_file_fast(det_file);
    raw_radar_data = parse_radar_file_fast(rad_file);
    
    % --- B. 逐帧评估 ---
    for i = 1:length(d.frames)
        frame_id = d.frames(i);
        
        % 1. 获取真值 GT
        xml_path = fullfile(base_dir, d.name, 'image', ...
                           sprintf('%s_original_image_frame_%d.xml', d.name, frame_id));
        gt_boxes = parse_labelimg_xml(xml_path);
        num_gt = size(gt_boxes, 1);
        
        % 2. 获取各方法预测点 [x, y]
        
        % (a) YOLO Only (取检测框中心)
        pts_yolo = get_yolo_points(raw_yolo_data, frame_id);
        
        % (b) Radar Only (取投影点)
        pts_radar = get_radar_points(raw_radar_data, frame_id);
        
        % (c) DS Fusion (从mat加载)
        mat_path = fullfile(base_dir, d.name, 'ds_results', ...
                           sprintf('ds_result_frame_%d.mat', frame_id));
        pts_fusion = get_fusion_points(mat_path);
        
        % 3. 计算指标并累加
        % 无论白天还是晚上，都分别计算三种方法的 TP/FP/FN
        [tp_y, fp_y, fn_y] = match_frames(gt_boxes, pts_yolo, config.dist_threshold);
        [tp_r, fp_r, fn_r] = match_frames(gt_boxes, pts_radar, config.dist_threshold);
        [tp_f, fp_f, fn_f] = match_frames(gt_boxes, pts_fusion, config.dist_threshold);
        % 根据场景类型累加到总表
        if strcmpi(d.type, 'day')
            stats_day = update_stats(stats_day, num_gt, tp_y, fp_y, fn_y, tp_r, fp_r, fn_r, tp_f, fp_f, fn_f);
        else
            stats_night = update_stats(stats_night, num_gt, tp_y, fp_y, fn_y, tp_r, fp_r, fn_r, tp_f, fp_f, fn_f);
        end
        
        if mod(i, 100) == 0, fprintf('   进度: %d/%d\n', i, length(d.frames)); end
    end
end

%% 4. 输出最终对比报表
print_comparison_table(stats_day, '白天场景 (Day)');
print_comparison_table(stats_night, '夜晚场景 (Night)');


%% ==================== 核心辅助函数 ====================

% --- 提取数据函数 ---
function pts = get_yolo_points(all_data, fid)
    pts = [];
    idx = find([all_data.frame] == fid, 1);
    if ~isempty(idx)
        targets = all_data(idx).targets;
        for k=1:length(targets)
            b = targets(k).bbox; % x,y,w,h
            pts = [pts; b(1)+b(3)/2, b(2)+b(4)/2]; % 转为中心点
        end
    end
end

function pts = get_radar_points(all_data, fid)
    pts = [];
    idx = find([all_data.frame] == fid, 1);
    if ~isempty(idx)
        pts = all_data(idx).points; % 已经是 x,y
    end
end

function pts = get_fusion_points(mat_path)
    pts = [];
    if exist(mat_path, 'file')
        d = load(mat_path);
        if isfield(d, 'ds_result') && ~isempty(d.ds_result)
            pts = d.ds_result(:, 1:2); % 取 x,y
        end
    end
end

% --- 统计结构体管理 ---
function s = init_method_stats(methods)
    for m = methods
        s.(m{1}) = struct('TP',0, 'FP',0, 'FN',0, 'GT',0);
    end
end

function s = update_stats(s, n_gt, ty, fy, fny, tr, fr, fnr, tf, ff, fnf)
    % 更新 YOLO
    s.YOLO.GT = s.YOLO.GT + n_gt; s.YOLO.TP = s.YOLO.TP + ty;
    s.YOLO.FP = s.YOLO.FP + fy;   s.YOLO.FN = s.YOLO.FN + fny;
    % 更新 Radar
    s.Radar.GT = s.Radar.GT + n_gt; s.Radar.TP = s.Radar.TP + tr;
    s.Radar.FP = s.Radar.FP + fr;   s.Radar.FN = s.Radar.FN + fnr;
    % 更新 Fusion
    s.Fusion.GT = s.Fusion.GT + n_gt; s.Fusion.TP = s.Fusion.TP + tf;
    s.Fusion.FP = s.Fusion.FP + ff;   s.Fusion.FN = s.Fusion.FN + fnf;
end

% --- 文件解析函数 (加速版) ---
function data = parse_detection_file_fast(filepath)
    data = struct('frame', {}, 'targets', {});
    if ~exist(filepath, 'file'), return; end
    content = regexp(fileread(filepath), '\n', 'split');
    curr_f = 0; targets = []; conf = 0.5;
    for i=1:length(content)
        l = strtrim(content{i});
        if isempty(l), continue; end
        if contains(l, '=== 第')
            if curr_f>0, data(end+1).frame=curr_f; data(end).targets=targets; end
            t = regexp(l, '第(\d+)帧', 'tokens'); curr_f=str2double(t{1}{1}); targets=[];
        elseif contains(l, '置信度：')
            t = regexp(l, '置信度：([\d.]+)', 'tokens'); if ~isempty(t), conf=str2double(t{1}{1}); end
        elseif contains(l, '位置:')
            t = regexp(l, '[\d.]+', 'match');
            if length(t)>=4
                bbox = str2double(t(end-3:end));
                targets = [targets, struct('bbox', bbox, 'conf', conf)];
            end
        end
    end
    if curr_f>0, data(end+1).frame=curr_f; data(end).targets=targets; end
end

function data = parse_radar_file_fast(filepath)
    data = struct('frame', {}, 'points', {});
    if ~exist(filepath, 'file'), return; end
    content = regexp(fileread(filepath), '\n', 'split');
    curr_f = 0; pts = [];
    for i=1:length(content)
        l = strtrim(content{i});
        if isempty(l), continue; end
        if contains(l, '# Frame')
            if curr_f>0, data(end+1).frame=curr_f; data(end).points=pts; end
            t = regexp(l, '# Frame (\d+)', 'tokens'); curr_f=str2double(t{1}{1}); pts=[];
        elseif ~startsWith(l, '#')
            v = sscanf(l, '%f'); if length(v)>=2, pts=[pts; v(1:2)']; end
        end
    end
    if curr_f>0, data(end+1).frame=curr_f; data(end).points=pts; end
end

% --- 核心匹配逻辑 (Point-in-Box) ---
function [tp, fp, fn] = match_frames(gt_boxes, pred_points, dist_thresh)
    num_gt = size(gt_boxes, 1);
    num_pred = size(pred_points, 1);
    gt_matched = false(num_gt, 1);
    
    for i = 1:num_pred
        px = pred_points(i, 1); py = pred_points(i, 2);
        best_idx = -1; min_dist = inf;
        for j = 1:num_gt
            if gt_matched(j), continue; end
            gx = gt_boxes(j,1); gy = gt_boxes(j,2); gw = gt_boxes(j,3); gh = gt_boxes(j,4);
            inside = (px>=gx && px<=gx+gw && py>=gy && py<=gy+gh);
            d = sqrt((px-(gx+gw/2))^2 + (py-(gy+gh/2))^2);
            if inside || d < dist_thresh
                if d < min_dist, min_dist = d; best_idx = j; end
            end
        end
        if best_idx ~= -1, gt_matched(best_idx) = true; end
    end
    tp = sum(gt_matched);
    fn = num_gt - tp;
    fp = num_pred - tp;
end

% --- XML解析 ---
function gt_boxes = parse_labelimg_xml(xml_file)
    gt_boxes = [];
    if ~exist(xml_file, 'file'), return; end
    try
        txt = fileread(xml_file);
        x1 = regexp(txt, '<xmin>(\d+)</xmin>', 'tokens'); y1 = regexp(txt, '<ymin>(\d+)</ymin>', 'tokens');
        x2 = regexp(txt, '<xmax>(\d+)</xmax>', 'tokens'); y2 = regexp(txt, '<ymax>(\d+)</ymax>', 'tokens');
        if ~isempty(x1)
            for k=1:length(x1)
                b = [str2double(x1{k}{1}), str2double(y1{k}{1}), str2double(x2{k}{1}), str2double(y2{k}{1})];
                gt_boxes = [gt_boxes; b(1), b(2), b(3)-b(1), b(4)-b(2)];
            end
        end
    catch
    end
end

% --- 打印函数 ---
function print_comparison_table(stats, title_str)
    fprintf('\n\n');
    fprintf('=================================================================================\n');
    fprintf('                  %s 算法性能对比\n', title_str);
    fprintf('=================================================================================\n');
    fprintf('| %-10s | %-6s | %-6s | %-6s | %-6s | %-9s | %-9s | %-8s |\n', ...
            'Method', 'GT', 'TP', 'FP', 'FN', 'Precision', 'Recall', 'F1-Score');
    fprintf('|------------|--------|--------|--------|--------|-----------|-----------|----------|\n');
    
    methods = {'YOLO', 'Radar', 'Fusion'};
    for i = 1:length(methods)
        m = methods{i};
        s = stats.(m);
        prec = s.TP / (s.TP + s.FP + 1e-6) * 100;
        rec  = s.TP / (s.TP + s.FN + 1e-6) * 100;
        f1   = 2 * prec * rec / (prec + rec + 1e-6) / 100;
        
        fprintf('| %-10s | %-6d | %-6d | %-6d | %-6d | %-8.2f%% | %-8.2f%% | %-8.4f |\n', ...
                m, s.GT, s.TP, s.FP, s.FN, prec, rec, f1);
    end
    fprintf('=================================================================================\n');
end

% --- 从图片文件名中提取实际帧号 --- 
function frames = get_actual_frames_from_images(img_dir, prefix)
    frames = [];
    if ~exist(img_dir, 'dir'), return; end
    
    % 获取所有图片文件
    img_files = dir(fullfile(img_dir, sprintf('%s_original_image_frame_*.jpg', prefix)));
    
    for i = 1:length(img_files)
        filename = img_files(i).name;
        % 提取帧号，使用字符串连接避免sprintf转义字符问题
        pattern = [prefix, '_original_image_frame_(\d+)'];
        match = regexp(filename, pattern, 'tokens');
        if ~isempty(match)
            frame_num = str2double(match{1}{1});
            frames = [frames; frame_num];
        end
    end
    
    % 排序并去重
    frames = sort(unique(frames));
end