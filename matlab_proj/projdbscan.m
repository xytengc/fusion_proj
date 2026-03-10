%--------------------------------------------------------------------------
% projdbscan.m - 雷达点云投影函数
% 功能：实现雷达点云的DBSCAN聚类和投影到图像平面
% 版本：1.0
% 日期：2024-01-13
% 输入：
%   K_data - 雷达数据
%   config - 配置参数结构体
%   K_camera - 相机内参矩阵
%   distortion_coefficient - 畸变系数
%   T_radar2camera - 雷达到相机的变换矩阵
%   imageSize - 图像尺寸
% 输出：
%   radar_xyz_image - 图像坐标
%   radar_xyzdv_real - 雷达真实数据
%--------------------------------------------------------------------------
function [radar_xyz_image, radar_xyzdv_real] = projdbscan(K_data, config, K_camera, distortion_coefficient, T_radar2camera, imageSize)
    
    % 提取雷达数据
    [radarPoints_xyz, radarPoints_ravs] = extractRadarData(K_data, config.scaleFactor);
    
    if isempty(radarPoints_xyz)
        radar_xyz_image = [];
        radar_xyzdv_real = [];
        return;
    end
    
    % 距离+速度聚类
    [clusterCenters_xyz, clusterCenters_ravs, clusterLabels, uniqueClusters] = ...
        clusterByDistance(radarPoints_xyz, radarPoints_ravs, ...
                                      config.epsilon, config.minPts, config.doCluster);
    
    if isempty(clusterCenters_xyz)
        radar_xyz_image = [];
        radar_xyzdv_real = [];
        return;
    end
    
    % 计算聚类密度
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
                                           T_radar2camera, imageSize);
    
    % 只保留有效投影的聚类中心
    if any(validIdx)
        % 只保存有效投影的聚类中心
        radar_xyz_image = [u(validIdx)', v(validIdx)'];
        % 只保存对应行的雷达数据
        clusterCenters_xyzdv = [clusterCenters_xyz(validIdx,:), clusterCenters_ravs(validIdx,:), clusterDensities(validIdx)];
        if size(clusterCenters_xyzdv, 1) > 0
            clusterCenters_xyzdv(:,7) = clusterCenters_xyzdv(:,7)./max(clusterCenters_xyzdv(:,7));
        end
        radar_xyzdv_real = clusterCenters_xyzdv;
    else
        radar_xyz_image = [];
        radar_xyzdv_real = [];
    end
end

% 提取雷达数据
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

% 聚类函数
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

% DBSCAN聚类实现
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

% DBSCAN算法实现
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

% 查找欧几里得距离邻域
function neighbors = findNeighborsEuclidean(features, pointIdx, epsilon)
    neighbors = [];
    point = features(pointIdx, :);
    for i = 1:size(features, 1)
        if i == pointIdx
            continue;
        end
        distance = sqrt(sum((features(i, :) - point).^2));
        if distance <= epsilon
            neighbors = [neighbors, i];
        end
    end
end

% 投影函数
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

