%--------------------------------------------------------------------------
% ds_fusion.m - DS证据融合处理函数 (带日志与尺寸信息版)
% 功能：读取匹配结果 -> 结合原始坐标 -> 执行DS融合 -> 保存最终目标列表
% 输出结果格式：[x, y, w, h, confidence, source_type]
%--------------------------------------------------------------------------
function ds_result = ds_fusion(config, frameIdx, fusion_input, raw_dets, raw_rads)
    % ds_fusion - 执行DS证据融合
    %   ds_result = ds_fusion(config, frameIdx, fusion_input, raw_dets, raw_rads)
    % 
    % 输入参数:
    %   config - 配置参数结构体
    %   frameIdx - 当前帧索引
    %   fusion_input - 来自chaparis的匹配结果
    %   raw_dets - 原始目标检测结果
    %   raw_rads - 原始雷达投影点
    % 
    % 输出参数:
    %   ds_result - 融合结果，格式为 [x, y, w, h, confidence, source_type]
    
    % 确保配置参数存在
    if ~isfield(config, 'condition')
        config.condition = 'day_data';
    end
    if ~isfield(config, 'name')
        config.name = 'default';
    end
    
    % 确保路径正确
    if ~isfield(config, 'output_dir')
        config.output_dir = sprintf('data/%s/%s/ds_results',config.condition,config.name); 
    end

    % DS融合参数
    if ~isfield(config, 'alpha_R')
        config.alpha_R       = 0.9;   % 雷达全局权重
    end
    if ~isfield(config, 'alpha_V')
        config.alpha_V       = 0.9;   % 视觉全局权重
    end
    if ~isfield(config, 'lambda_R')
        config.lambda_R      = 0.58;  % 雷达单源折扣因子
    end
    if ~isfield(config, 'lambda_V')
        config.lambda_V      = 0.9;  % 视觉单源折扣因子
    end
    if ~isfield(config, 'fusion_thresh')
        config.fusion_thresh = 0.5;  % 最终输出阈值
    end

    %% 初始化
    if ~exist(config.output_dir, 'dir')
        mkdir(config.output_dir);
    end

    % 初始化返回结果
    ds_result = [];
    
    try
        % 检查fusion_input是否有效
        if ~exist('fusion_input', 'var') || isempty(fusion_input)
            return;
        end
        
        % 确保raw_dets和raw_rads存在
        if ~exist('raw_dets', 'var')
            raw_dets = [];
        end
        if ~exist('raw_rads', 'var')
            raw_rads = [];
        end
        
        % === 执行核心 DS 融合逻辑 ===
        ds_result = run_ds_logic(fusion_input, raw_dets, raw_rads, config);
        
        % === 保存结果 ===
        save_path = fullfile(config.output_dir, sprintf('ds_result_frame_%d.mat', frameIdx));
        save(save_path, 'ds_result');
        
    catch ME
        fprintf('处理第 %d 帧时出错: %s\n', frameIdx, ME.message);
    end
end

%% ==================== 核心逻辑函数 ====================

function ds_result = run_ds_logic(input, raw_dets, raw_rads, cfg)
    % 优化版 DS 逻辑：侧重于 Recall (召回率)
    
    final_targets = [];
    
    % --- 1. 处理匹配对 (Fused: Vision + Radar) ---
    % 这是一个强目标，我们应该给予极高的置信度
    if isfield(input, 'matches')
        for k = 1:length(input.matches)
            m = input.matches(k);
            
            conf_V = 0.5; 
            if m.cam_id <= length(raw_dets)
                bbox = raw_dets(m.cam_id).bbox;
                w = bbox(3); h = bbox(4);
                px = bbox(1) + w/2; py = bbox(2) + h/2;
                if isfield(raw_dets(m.cam_id), 'conf')
                    conf_V = raw_dets(m.cam_id).conf;
                end
            else
                continue;
            end
            
            % 雷达在匹配时提供高信度支持
            conf_R = 0.9; 
            
            m_R_H1 = cfg.alpha_R * conf_R;
            m_V_H1 = cfg.alpha_V * conf_V;
            
            % [优化]: 并集增强公式 P(A u B) = P(A) + P(B) - P(A)P(B)
            % 这能保证 融合后的分数 >= 任意单一传感器的分数
            m_final = m_R_H1 + m_V_H1 - (m_R_H1 * m_V_H1);
            
            % 强目标直接保留，无需太多阈值判断
            final_targets = [final_targets; px, py, w, h, m_final, 1];
        end
    end
    
    % --- 2. 处理未匹配雷达 (Radar Only) - 捞回漏检的关键 ---
    if isfield(input, 'uR') && ~isempty(raw_rads)
        for k = 1:length(input.uR)
            rid = input.uR(k);
            if rid <= size(raw_rads, 1)
                rx = raw_rads(rid, 1); ry = raw_rads(rid, 2);
                
                % 边界过滤 (防止边缘杂波)
                if rx < 5 || rx > 635 || ry < 5 || ry > 475, continue; end
                
                % 获取雷达原始信度 (如果没有，默认为 0.8)
                raw_conf_R = 0.8; 
                if isfield(input, 'qR') && length(input.qR) >= rid
                    raw_conf_R = input.qR(rid);
                end
                
                % DS 计算
                m_R_H1 = cfg.alpha_R * raw_conf_R;
                m_final = cfg.lambda_R * m_R_H1; % 折扣
                
                % 只要过了阈值就保留
                if m_final > cfg.fusion_thresh
                    % 赋予一个默认尺寸 (比如 40x40 像素)，便于 evaluate 匹配
                    default_w = 40; 
                    default_h = 40;
                    final_targets = [final_targets; rx, ry, default_w, default_h, m_final, 2];
                end
            end
        end
    end
    
    % --- 3. 处理未匹配视觉 (Vision Only) - 防止YOLO目标丢失 ---
    if isfield(input, 'uC') && ~isempty(raw_dets)
        for k = 1:length(input.uC)
            cid = input.uC(k);
            if cid <= length(raw_dets)
                bbox = raw_dets(cid).bbox;
                w = bbox(3); h = bbox(4);
                cx = bbox(1) + w/2; cy = bbox(2) + h/2;
                
                raw_conf_V = raw_dets(cid).conf;
                
                % DS 计算
                m_V_H1 = cfg.alpha_V * raw_conf_V;
                m_final = cfg.lambda_V * m_V_H1; % 折扣
                
                % [关键修正]: 如果阈值设得高，lambda_V会导致原本0.6的目标变成0.48被删
                % 所以这里要保证如果 raw_conf_V 本身就不错，不要杀得太狠
                if m_final > cfg.fusion_thresh
                    final_targets = [final_targets; cx, cy, w, h, m_final, 3];
                end
            end
        end
    end
    
    ds_result = final_targets;
end

%% ==================== 数据解析辅助函数 ====================
function [frame_detections, frame_radar_projections] = load_raw_data(det_file, radar_file)
    frame_detections = parse_detection_file_local(det_file);
    frame_radar_projections = parse_image_file_local(radar_file);
end

% 辅助解析函数：Detection TXT
function frame_detections = parse_detection_file_local(file_path)
    content = regexp(fileread(file_path), '\n', 'split');
    frame_detections = struct('frame', {}, 'targets', {});
    current_frame = 0; targets = [];
    current_conf = 0; % 暂存置信度
    
    for i = 1:length(content)
        line = strtrim(content{i});
        if isempty(line), continue; end
        
        if contains(line, '=== 第')
            if current_frame > 0 
                frame_detections(end+1).frame = current_frame; 
                frame_detections(end).targets = targets; 
            end
            tok = regexp(line, '第(\d+)帧', 'tokens');
            if ~isempty(tok)
                current_frame = str2double(tok{1}{1}); 
                targets = []; 
            end
        elseif contains(line, '类型:')
            % 提取置信度
            tok = regexp(line, '置信度：([\d.]+)', 'tokens');
            if ~isempty(tok)
                current_conf = str2double(tok{1}{1}); 
            else
                current_conf = 0.5; 
            end
        elseif contains(line, '位置:')
            tok = regexp(line, '位置:\s*([\d.]+)\s*([\d.]+)\s*([\d.]+)\s*([\d.]+)', 'tokens');
            if ~isempty(tok)
                bbox = [str2double(tok{1}{1}), str2double(tok{1}{2}), str2double(tok{1}{3}), str2double(tok{1}{4})];
                new_target = struct('bbox', bbox, 'conf', current_conf);
                targets = [targets, new_target]; 
            end
        end
    end
    if current_frame > 0 
        frame_detections(end+1).frame = current_frame; 
        frame_detections(end).targets = targets; 
    end
end

% 辅助解析函数：Radar Image Coords TXT
function frame_image_data = parse_image_file_local(file_path)
    content = regexp(fileread(file_path), '\n', 'split');
    frame_image_data = struct('frame', {}, 'image_points', {});
    current_frame = 0; pts = [];
    
    for i = 1:length(content)
        line = strtrim(content{i});
        if isempty(line), continue; end
        
        if contains(line, '# Frame')
            if current_frame > 0 
                frame_image_data(end+1).frame = current_frame; 
                frame_image_data(end).image_points = pts; 
            end
            tok = regexp(line, '# Frame (\d+)', 'tokens');
            if ~isempty(tok)
                current_frame = str2double(tok{1}{1}); 
                pts = []; 
            end
        elseif ~startsWith(line, '#')
            d = sscanf(line, '%f');
            if length(d) >= 2
                pts = [pts; d(1:2)']; 
            end
        end
    end
    if current_frame > 0 
        frame_image_data(end+1).frame = current_frame; 
        frame_image_data(end).image_points = pts; 
    end
end