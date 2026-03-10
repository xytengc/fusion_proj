%--------------------------------------------------------------------------
% chaparis.m - 基于匈牙利算法的雷达点云与图像匹配系统
% 功能：实现雷达–视觉目标最优匹配
% 版本：2.0
% 日期：2024-01-13
% 优化：代码模块化、性能优化、错误处理增强
%--------------------------------------------------------------------------
function fusion_input = chaparis(config, frameIdx, frame, targets, radar_xyzdv_real, radar_xyz_image)
    % chaparis - 执行雷达-视觉目标匹配
    %   fusion_input = chaparis(config, frameIdx, frame, targets, radar_xyzdv_real, radar_xyz_image)
    % 
    % 输入参数:
    %   config - 配置参数结构体
    %   frameIdx - 当前帧索引
    %   frame - 当前帧图像
    %   targets - 目标检测结果
    %   radar_xyzdv_real - 雷达真实数据
    %   radar_xyz_image - 雷达图像坐标
    % 
    % 输出参数:
    %   fusion_input - 匹配结果结构体
    
    do_vis  = false;% 作为中间函数，关闭可视化
    
    % 确保配置参数存在
    if ~isfield(config, 'condition')
        config.condition = 'day_data';
    end
    if ~isfield(config, 'name')
        config.name = 'default';
    end
    
    % 确保路径正确
    if ~isfield(config, 'data_dir')
        script_dir = pwd;
        config.data_dir = fullfile(script_dir, 'data', config.condition); 
    end
    if ~isfield(config, 'out_data_dir')
        script_dir = pwd;
        config.out_data_dir =fullfile(script_dir, 'data', config.condition,config.name);
    end
    
    % 确保输出目录存在
    if ~exist(config.out_data_dir, 'dir')
        mkdir(config.out_data_dir);
    end
    
    % 处理参数
    if ~isfield(config, 'imgHeight')
        config.imgHeight  = 480;
    end
    if ~isfield(config, 'imgWidth')
        config.imgWidth   = 640;
    end
    if ~isfield(config, 'confThresh')
        config.confThresh = 0.7;
    end
    if ~isfield(config, 'validTypes')
        config.validTypes = {'bus', 'camper', 'car', 'cyclist', 'motorcycle', 'person', 'tractor', 'truck', 'van'};
    end
    % 匹配参数
    if ~isfield(config, 'alpha')
        config.alpha = 1.0;    % 图像距离权重
    end
    if ~isfield(config, 'beta')
        config.beta = 0.5;     % 雷达距离权重
    end
    if ~isfield(config, 'gamma')
        config.gamma = 0.3;    % 置信度权重
    end
    if ~isfield(config, 'dmax')
        config.dmax = 50;      % 最大雷达量程
    end
    % 检测框扩展参数
    if ~isfield(config, 'bbox_expand_ratio_horiz')
        config.bbox_expand_ratio_horiz = 0.3;    % 横向扩展比例
    end
    if ~isfield(config, 'bbox_expand_ratio_vert')
        config.bbox_expand_ratio_vert = 0.2;     % 纵向扩展比例
    end
    if ~isfield(config, 'min_expand_pixels')
        config.min_expand_pixels = 25;           % 最小扩展像素
    end
    % 二次筛选阈值
    if ~isfield(config, 'dist_threshold')
        config.dist_threshold = 5.0;       % 距离阈值（米）
    end
    if ~isfield(config, 'vel_threshold')
        config.vel_threshold = 2.0;        % 速度阈值（米/秒）
    end
    % 性能参数
    if ~isfield(config, 'enable_visualization')
        config.enable_visualization = false;% 作为中间函数，关闭可视化
    end
    % 成本阈值
    if ~isfield(config, 'costUnmatched')
        config.costUnmatched = 1e6;        % 未匹配代价
    end
    
    % 初始化fusion_input
    fusion_input = [];
    
    try
        % 构建当前帧的数据结构
        frame_detections = struct('frame', frameIdx, 'targets', targets);
        
        if ~isempty(radar_xyzdv_real)
            frame_radar_data = struct('frame', frameIdx, 'radar_points', radar_xyzdv_real);
        else
            frame_radar_data = struct('frame', frameIdx, 'radar_points', []);
        end
        
        if ~isempty(radar_xyz_image)
            frame_image_data = struct('frame', frameIdx, 'image_points', radar_xyz_image);
        else
            frame_image_data = struct('frame', frameIdx, 'image_points', []);
        end
        
        % 处理当前帧并获取fusion_input
        fusion_input = process_single_frame(do_vis, frameIdx, frame, frame_detections, frame_radar_data, frame_image_data, config);
        
    catch ME
        % 异常处理
        fprintf('处理第 %d 帧时出错: %s\n', frameIdx, ME.message);
    end
end


%% ==================== 单帧处理函数 ====================
function fusion_input = process_single_frame(do_vis, frame_idx, img, frame_detections, frame_radar_data, frame_image_data, config)
    
    % 初始化fusion_input
    fusion_input = [];
    
    try
        %% 获取检测数据
        [cam, validTargetIdx] = get_detection_data(frame_detections, frame_idx, config);
        
        %% 获取雷达数据
        radar = get_radar_data(frame_radar_data, frame_image_data, frame_idx, config);
        
        %% 数据融合处理
        if validTargetIdx > 0 && ~isempty(radar)
            % 双源数据匹配
            [fusion_input, expanded_bboxes] = process_dual_source(...
                cam, radar, frame_idx, config);
            
        elseif validTargetIdx > 0
            % 仅视觉数据
            fusion_input = struct();
            fusion_input.matches = [];
            fusion_input.uR = [];
            fusion_input.uC = 1:validTargetIdx;
            expanded_bboxes = [];
            
        elseif ~isempty(radar)
            % 仅雷达数据
            fusion_input = struct();
            fusion_input.matches = [];
            fusion_input.uR = 1:size(radar, 1);
            fusion_input.uC = [];
            expanded_bboxes = [];
            
        else
            return;
        end
        
        %% 保存结果
        if ~isempty(fusion_input)
            save_fusion_data(fusion_input, frame_idx, config, img, cam, radar);
        end
        
        %% 可视化
        if config.enable_visualization && ~isempty(img) && ~isempty(fusion_input)
            visualize_results(do_vis, img, cam, radar, fusion_input, ...
                             expanded_bboxes, frame_idx, config);
        end
        
    catch ME
        fprintf('处理第 %d 帧时出错: %s\n', frame_idx, ME.message);
    end
end

%% ==================== 辅助函数 ====================

function [cam, validTargetIdx] = get_detection_data(frame_detections, frame_idx, config)
    cam = struct('bbox', {}, 'conf', {}, 'type', {});
    validTargetIdx = 0;
    
    % 检查输入是否有效
    if ~isstruct(frame_detections)
        return;
    end
    
    % 处理单个结构体的情况
    if isscalar(frame_detections)
        if frame_detections.frame == frame_idx
            current_targets = frame_detections.targets;
            
            if iscell(current_targets)
                % 单个目标的情况
                target = current_targets;
                if isfield(target, 'bbox') && isfield(target, 'type') && isfield(target, 'conf')
                    if target.conf >= config.confThresh
                        validTargetIdx = validTargetIdx + 1;
                        cam(validTargetIdx).conf = target.conf;
                        cam(validTargetIdx).bbox = target.bbox;
                        cam(validTargetIdx).type = target.type;
                    end
                end
            elseif iscell(current_targets) || isstruct(current_targets)
                % 多个目标的情况
                for targetIdx = 1:length(current_targets)
                    target = current_targets(targetIdx);
                    if isfield(target, 'bbox') && isfield(target, 'type') && isfield(target, 'conf')
                        if target.conf >= config.confThresh
                            validTargetIdx = validTargetIdx + 1;
                            cam(validTargetIdx).conf = target.conf;
                            cam(validTargetIdx).bbox = target.bbox;
                            cam(validTargetIdx).type = target.type;
                        end
                    end
                end
            end
        end
        return;
    end
    
    % 处理结构体数组的情况
    frame_det_idx = find([frame_detections.frame] == frame_idx);
    if ~isempty(frame_det_idx)
        % 确保索引在有效范围内
        if frame_det_idx <= length(frame_detections)
            current_targets = frame_detections(frame_det_idx).targets;
            
            if iscell(current_targets)
                % 单个目标的情况
                target = current_targets;
                if isfield(target, 'bbox') && isfield(target, 'type') && isfield(target, 'conf')
                    if target.conf >= config.confThresh
                        validTargetIdx = validTargetIdx + 1;
                        cam(validTargetIdx).conf = target.conf;
                        cam(validTargetIdx).bbox = target.bbox;
                        cam(validTargetIdx).type = target.type;
                    end
                end
            elseif iscell(current_targets) || isstruct(current_targets)
                % 多个目标的情况
                for targetIdx = 1:length(current_targets)
                    target = current_targets(targetIdx);
                    if isfield(target, 'bbox') && isfield(target, 'type') && isfield(target, 'conf')
                        if target.conf >= config.confThresh
                            validTargetIdx = validTargetIdx + 1;
                            cam(validTargetIdx).conf = target.conf;
                            cam(validTargetIdx).bbox = target.bbox;
                            cam(validTargetIdx).type = target.type;
                        end
                    end
                end
            end
        end
    end
end

function radar = get_radar_data(frame_radar_data, frame_image_data, frame_idx, config)
    radar = [];
    
    % 检查输入是否有效
    if ~isstruct(frame_radar_data) || ~isstruct(frame_image_data)
        return;
    end
    
    % 处理单个结构体的情况
    if isscalar(frame_radar_data) && isscalar(frame_image_data)
        if frame_radar_data.frame == frame_idx && frame_image_data.frame == frame_idx
            radar_points = frame_radar_data.radar_points;
            image_points = frame_image_data.image_points;
            
            if ~isempty(radar_points) && ~isempty(image_points) && size(radar_points, 1) == size(image_points, 1)
                radar = zeros(size(radar_points, 1), 7);
                radar(:,1) = image_points(:,1);      % u
                radar(:,2) = image_points(:,2);      % v
                radar(:,3) = radar_points(:,4);      % 距离
                radar(:,4) = radar_points(:,5);      % 角度
                radar(:,5) = radar_points(:,6);      % 速度
                radar(:,6) = radar_points(:,7);      % Signal
                radar(:,7) = radar_points(:,8);      % Density
            end
        end
        return;
    end
    
    % 处理结构体数组的情况
    frame_radar_idx = find([frame_radar_data.frame] == frame_idx);
    frame_image_idx = find([frame_image_data.frame] == frame_idx);
    
    if ~isempty(frame_radar_idx) && ~isempty(frame_image_idx)
        % 确保索引在有效范围内
        if frame_radar_idx <= length(frame_radar_data) && frame_image_idx <= length(frame_image_data)
            radar_points = frame_radar_data(frame_radar_idx).radar_points;
            image_points = frame_image_data(frame_image_idx).image_points;
            
            if ~isempty(radar_points) && ~isempty(image_points) && size(radar_points, 1) == size(image_points, 1)
                radar = zeros(size(radar_points, 1), 7);
                radar(:,1) = image_points(:,1);      % u
                radar(:,2) = image_points(:,2);      % v
                radar(:,3) = radar_points(:,4);      % 距离
                radar(:,4) = radar_points(:,5);      % 角度
                radar(:,5) = radar_points(:,6);      % 速度
                radar(:,6) = radar_points(:,7);      % Signal
                radar(:,7) = radar_points(:,8);      % Density
            end
        end
    end
end

%% ==================== 双源数据处理 ====================
function [fusion_input, expanded_bboxes] = process_dual_source(...
    cam, radar, frame_idx, config)
    
    Nr = size(radar, 1);
    Nc = length(cam);
    
    %% 1. 计算扩展检测框
    expanded_bboxes = compute_expanded_bboxes(cam, config);
    
    %% 2. 预计算检测框参数
    [bbox_centers, bbox_diags, bbox_size_factors, bbox_confs] = ...
        precompute_bbox_params(cam, config);
    
    %% 3. 计算成本矩阵
    Cost = compute_cost_matrix(radar, cam, expanded_bboxes, ...
                              bbox_centers, bbox_diags, bbox_size_factors, ...
                              bbox_confs, config);
    
    %% 4. 匈牙利算法匹配
    [M, uR, uC] = matchpairs(Cost, config.costUnmatched);
    
    %% 5. 构建匹配结果
    matches = build_matches(M, Cost, radar, cam);
    
    %% 6. 二次筛选（微调和遮挡检测）
    [refined_matches, occlusion_targets] = ...
        secondary_filtering(matches, uR, radar, cam, config);
    
    %% 8. 构建融合输入
    fusion_input = struct();
    fusion_input.matches = matches;
    fusion_input.uR = uR;
    fusion_input.uC = uC;
    
    if ~isempty(refined_matches)
        fusion_input.refined_matches = refined_matches;
    end
    
    if ~isempty(occlusion_targets)
        fusion_input.occlusion_targets = occlusion_targets;
    end
end

function expanded_bboxes = compute_expanded_bboxes(cam, config)
    Nc = length(cam);
    expanded_bboxes = zeros(Nc, 4);
    
    for j = 1:Nc
        x = cam(j).bbox(1);
        y = cam(j).bbox(2);
        w = cam(j).bbox(3);
        h = cam(j).bbox(4);
        
        expand_w = max(w * config.bbox_expand_ratio_horiz, config.min_expand_pixels);
        expand_h = max(h * config.bbox_expand_ratio_vert, config.min_expand_pixels);
        
        exp_x = max(1, x - expand_w/2);
        exp_y = max(1, y - expand_h/2);
        exp_w = min(config.imgWidth - exp_x, w + expand_w);
        exp_h = min(config.imgHeight - exp_y, h + expand_h);
        
        expanded_bboxes(j,:) = [exp_x, exp_y, exp_w, exp_h];
    end
end

function [bbox_centers, bbox_diags, bbox_size_factors, bbox_confs] = precompute_bbox_params(cam, config)
    
    Nc = length(cam);
    bbox_centers = zeros(Nc, 2);
    bbox_diags = zeros(Nc, 1);
    bbox_size_factors = zeros(Nc, 1);
    bbox_confs = zeros(Nc, 1);
    
    for j = 1:Nc
        x = cam(j).bbox(1);
        y = cam(j).bbox(2);
        w = cam(j).bbox(3);
        h = cam(j).bbox(4);
        
        bbox_centers(j,:) = [x + w/2, y + h/2];
        bbox_diags(j) = sqrt(w^2 + h^2);
        bbox_size_factors(j) = bbox_diags(j) / sqrt(config.imgWidth^2 + config.imgHeight^2);
        bbox_confs(j) = cam(j).conf;
    end
end

function Cost = compute_cost_matrix(radar, cam, expanded_bboxes, ...
                                   bbox_centers, bbox_diags, bbox_size_factors, ...
                                   bbox_confs, config)
    
    Nr = size(radar, 1);
    Nc = length(cam);
    Cost = Inf(Nr, Nc);
    
    % 使用向量化计算提高性能
    for j = 1:Nc
        % 获取当前检测框参数
        exp_bbox = expanded_bboxes(j,:);
        cx = bbox_centers(j,1);
        cy = bbox_centers(j,2);
        bbox_diag = bbox_diags(j);
        bbox_conf = bbox_confs(j);
        size_factor = bbox_size_factors(j);
        
        % 原始检测框
        x = cam(j).bbox(1);
        y = cam(j).bbox(2);
        w = cam(j).bbox(3);
        h = cam(j).bbox(4);
        
        for i = 1:Nr
            u = radar(i,1);
            v = radar(i,2);
            radar_dist = radar(i,3);
            
            % 判断是否在扩展区域内
            if (u >= exp_bbox(1) && u <= exp_bbox(1)+exp_bbox(3) && ...
                v >= exp_bbox(2) && v <= exp_bbox(2)+exp_bbox(4))
                
                % 1. 归一化图像距离
                dist_img = sqrt((u-cx)^2 + (v-cy)^2) / bbox_diag;
                
                % 2. 判断是否在原始框内
                in_bbox = (u >= x && u <= x+w && v >= y && v <= y+h);
                in_bbox_bonus = in_bbox * 0.5 + (~in_bbox) * 1.0;
                
                % 3. 归一化雷达距离
                dist_radar = min(radar_dist / config.dmax, 1.0);
                
                % 4. 计算成本
                cost = config.alpha * dist_img * in_bbox_bonus + ...
                       config.beta * dist_radar - ...
                       config.gamma * bbox_conf + 0.1 * size_factor;
                
                Cost(i,j) = max(cost, 0);
            end
        end
    end
end

function matches = build_matches(M, Cost, radar, cam)
    matches = struct('radar_id', {}, 'cam_id', {}, ...
                    'cost', {}, 'score', {}, 'match_type', {});
    
    for k = 1:size(M, 1)
        i = M(k,1);  % 雷达点索引
        j = M(k,2);  % 检测框索引
        
        % 计算匹配分数
        raw_cost = Cost(i,j);
        match_score = max(0, 1.0 - raw_cost);
        
        % 判断匹配类型
        u = radar(i,1);
        v = radar(i,2);
        bbox = cam(j).bbox;
        
        if (u >= bbox(1) && u <= bbox(1)+bbox(3) && ...
            v >= bbox(2) && v <= bbox(2)+bbox(4))
            match_type = 'inside';
        else
            match_type = 'expanded';
        end
        
        matches(k).radar_id = i;
        matches(k).cam_id = j;
        matches(k).cost = raw_cost;
        matches(k).score = match_score;
        matches(k).match_type = match_type;
    end
end

function [refined_matches, occlusion_targets] = secondary_filtering(matches, uR, radar, cam, config)
    
    refined_matches = [];
    occlusion_targets = [];
    
    % 收集落在检测框内的未匹配点
    unmatched_in_bbox = [];
    
    for i = 1:length(uR)
        radar_id = uR(i);
        u = radar(radar_id, 1);
        v = radar(radar_id, 2);
        
        for j = 1:length(cam)
            bbox = cam(j).bbox;
            if (u >= bbox(1) && u <= bbox(1)+bbox(3) && ...
                v >= bbox(2) && v <= bbox(2)+bbox(4))
                
                unmatched_in_bbox(end+1) = struct(...
                    'radar_id', radar_id, ...
                    'cam_id', j, ...
                    'distance', radar(radar_id, 3), ...
                    'velocity', radar(radar_id, 5));
                break;
            end
        end
    end
    
    if isempty(unmatched_in_bbox)
        return;
    end
    
    % 按检测框分组处理
    for j = 1:length(cam)
        % 找到该检测框的最优匹配
        best_match_idx = find([matches.cam_id] == j);
        
        if ~isempty(best_match_idx)
            best_match = matches(best_match_idx);
            best_distance = radar(best_match.radar_id, 3);
            best_velocity = radar(best_match.radar_id, 5);
            
            % 找到该检测框的未匹配点
            unmatched_for_bbox = unmatched_in_bbox([unmatched_in_bbox.cam_id] == j);
            
            for k = 1:length(unmatched_for_bbox)
                point = unmatched_for_bbox(k);
                
                % 计算差异
                dist_diff = abs(point.distance - best_distance);
                vel_diff = abs(point.velocity - best_velocity);
                
                if dist_diff < config.dist_threshold && vel_diff < config.vel_threshold
                    % 微调匹配
                    refined_match = best_match;
                    refined_match.refined = true;
                    refined_match.original_distance = best_distance;
                    refined_match.original_velocity = best_velocity;
                    refined_match.refined_distance = (best_distance + point.distance) / 2;
                    refined_match.refined_velocity = (best_velocity + point.velocity) / 2;
                    refined_match.additional_radar_id = point.radar_id;
                    
                    refined_matches = [refined_matches; refined_match];
                else
                    % 遮挡目标
                    occlusion_targets = [occlusion_targets; struct(...
                        'cam_id', j, ...
                        'radar_id', point.radar_id, ...
                        'distance', point.distance, ...
                        'velocity', point.velocity, ...
                        'distance_diff', dist_diff, ...
                        'velocity_diff', vel_diff)];
                end
            end
        end
    end
end

%% ==================== 可视化函数 ====================
function visualize_results(do_vis,img, cam, radar, fusion_input, ...
                         expanded_bboxes, frame_idx, config)
    
    if ~do_vis
        return;
    end
    try
        hFig = figure('Name', sprintf('匹配结果 - 第%d帧', frame_idx), ...
                     'NumberTitle', 'off', 'Visible', 'off');
        img = IM_TAGC_Enhancement(img);
        imshow(img);
        hold on;
        
        % 绘制检测框
        if ~isempty(cam)
            for i = 1:length(cam)
                bbox = cam(i).bbox;
                rectangle('Position', bbox, 'EdgeColor', 'red', 'LineWidth', 2);
                text(bbox(1), bbox(2)-5, sprintf('检测框 %d', i), ...
                     'Color', 'black', 'BackgroundColor', 'white', ...
                     'FontSize', 8, 'FontWeight', 'bold');
            end
        end
        
        % 绘制扩展框
        if ~isempty(expanded_bboxes)
            for j = 1:size(expanded_bboxes, 1)
                rectangle('Position', expanded_bboxes(j,:), ...
                         'EdgeColor', 'green', 'LineWidth', 1, 'LineStyle', '--');
            end
        end
        
        % 绘制雷达点
        if ~isempty(radar)
            % 分类雷达点
            if isfield(fusion_input, 'refined_matches') && ~isempty(fusion_input.refined_matches)
                refined_ids = [fusion_input.refined_matches.radar_id];
            else
                refined_ids = [];
            end
            
            all_ids = 1:size(radar, 1);
            normal_ids = setdiff(all_ids, refined_ids);
            
            % 绘制普通点
            if ~isempty(normal_ids)
                scatter(radar(normal_ids,1), radar(normal_ids,2), ...
                        10, 'b', 'filled', 'o');
            end
            
            % 绘制微调点
            if ~isempty(refined_ids)
                scatter(radar(refined_ids,1), radar(refined_ids,2), ...
                        12, [1 0.5 0], 'filled', 's');
            end
            
            % 标注雷达点
            for i = 1:size(radar, 1)
                if ismember(i, refined_ids)
                    text_color = 'yellow';
                else
                    text_color = [1 0.5 0];
                end
                text(radar(i,1)-5, radar(i,2)-5, sprintf('R%d', i), ...
                     'Color', text_color, 'FontSize', 8, 'FontWeight', 'bold');
            end
        end
        
        % 绘制匹配连接线
        if ~isempty(fusion_input.matches)
            for k = 1:length(fusion_input.matches)
                match = fusion_input.matches(k);
                
                if match.radar_id <= size(radar, 1) && match.cam_id <= length(cam)
                    rx = radar(match.radar_id, 1);
                    ry = radar(match.radar_id, 2);
                    
                    bbox = cam(match.cam_id).bbox;
                    cx = bbox(1) + bbox(3)/2;
                    cy = bbox(2) + bbox(4)/2;
                    
                    if strcmp(match.match_type, 'inside')
                        line_color = 'g';
                        line_style = '-';
                    else
                        line_color = 'm';
                        line_style = '--';
                    end
                    
                    plot([rx, cx], [ry, cy], 'Color', line_color, ...
                         'LineWidth', 1.5, 'LineStyle', line_style);
                end
            end
            
            title(sprintf('匹配结果可视化 - 第%d帧 (%d个匹配)', ...
                         frame_idx, length(fusion_input.matches)), ...
                 'FontSize', 12, 'FontWeight', 'bold');
        end
        
        hold off;
        
        % 提取config.out_data_dir的最后子目录名作为前缀
        path_parts = strsplit(config.out_data_dir, filesep);
        prefix = path_parts{end};
        
        % 保存图像
        mat_dir = fullfile(config.out_data_dir, 'match_jpgfile');
        if ~exist(mat_dir, 'dir')
            mkdir(mat_dir);
        end
        save_path = fullfile(mat_dir, ...
                           sprintf('%s_matching_result_frame_%d.jpg', prefix, frame_idx));
        saveas(hFig, save_path);
        close(hFig);
        
    catch ME
        fprintf('可视化第 %d 帧时出错: %s\n', frame_idx, ME.message);
    end
end

%% ==================== 数据保存函数 ====================
function save_fusion_data(fusion_input, frame_idx, config, img, cam, radar)
    % 添加雷达质量因子 qR
    if ~isempty(radar)
        qR = zeros(size(radar, 1), 1);
        for i = 1:size(radar, 1)
            signal = radar(i, 6);  % Signal
            density = radar(i, 7); % Density
            qR(i) = signal * density;
            qR(i) = max(0, min(1, qR(i)));
        end
        fusion_input.qR = qR;
    else
        fusion_input.qR = [];
    end
    
    % 添加视觉目标置信度 qV
    if ~isempty(cam)
        qV = zeros(length(cam), 1);
        for j = 1:length(cam)
            qV(j) = cam(j).conf;
        end
        fusion_input.qV = qV;
    else
        fusion_input.qV = [];
    end
    
    % 提取config.out_data_dir的最后子目录名作为前缀
    path_parts = strsplit(config.out_data_dir, filesep);
    prefix = path_parts{end};
    
    % 保存融合数据
    mat_dir = fullfile(config.out_data_dir, 'match_matfile');
    if ~exist(mat_dir, 'dir')
        mkdir(mat_dir);
    end
    save_path = fullfile(mat_dir, ...
                        sprintf('%s_fusion_input_data_frame_%d.mat', prefix, frame_idx));
    save(save_path, 'fusion_input');
    
    % 保存原始图像
    if ~isempty(img)
        image_dir = fullfile(config.out_data_dir, 'image');
        if ~exist(image_dir, 'dir')
            mkdir(image_dir);
        end
        image_path = fullfile(image_dir, sprintf('%s_original_image_frame_%d.jpg', prefix, frame_idx));
        imwrite(img, image_path);
    end
end

%% ==================== 日志函数 ====================
function log_write(log_fid, msg, varargin)
    if log_fid ~= -1
        fprintf(log_fid, msg, varargin{:});
    end
end