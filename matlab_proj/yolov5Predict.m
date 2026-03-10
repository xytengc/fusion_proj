function [bboxes, scores, labelIds] = yolov5Predict(image,net, numClasses,executionEnvironment)
    % YOLOv5标准输入大小为640x640
    targetSize = [640, 640];
    % 调整图像大小和填充
    [img, scale, ~] = helper.letterbox(image, targetSize);
    % 转换为单精度并归一化
    img = single(img) / 255.0;
    % 添加批次维度
    img = reshape(img, [1, size(img, 1), size(img, 2), size(img, 3)]);
    % 转换为dlarray
    dlInput = dlarray(img, 'BSSC');
    % 如果GPU可用，使用GPU
    if (strcmp(executionEnvironment, 'auto') && gpuDeviceCount > 0) || strcmp(executionEnvironment, 'gpu')
        dlInput = gpuArray(dlInput);
    end
    % 执行推理
    outputs = predict(net, dlInput);
    
    % 处理YOLOv5输出
    % 对于YOLOv5，outputs是一个1xNx16的数组，其中N是检测数量
    predictions = extractdata(outputs);
    
    % 处理展平后的YOLOv5输出格式
    if numel(size(predictions)) == 3 && size(predictions, 1) == 1
        predictions = squeeze(predictions);  % 变为 [N, 16]
    end
    
    % 解析检测结果
    if size(predictions, 2) == numClasses+5
        % 16 = 4(bbox) + 1(confidence) + 11(classes)
        % YOLOv5 ONNX模型输出已经经过sigmoid激活，不需要再应用
        bboxes_raw = predictions(:, 1:4);  % 原始边界框坐标
        obj_scores = predictions(:, 5);     % 对象置信度
        class_probs = predictions(:, 6:end);  % 类别概率
        
        % 转换原始坐标 (相对到640x640) 到归一化坐标
        bboxes_norm = bboxes_raw / 640.0;  % 归一化到 [0, 1] 范围
    else
        error('不支持的输出格式: %s', mat2str(size(predictions)));
    end
    
    % 置信度阈值
    confThreshold = 0.25;
    validIdx = obj_scores > confThreshold;
    
    if sum(validIdx) == 0
        bboxes = [];
        scores = [];
        labelIds = [];
        return;
    end
    
    % 过滤检测结果
    bboxes_norm = bboxes_norm(validIdx, :);
    obj_scores = obj_scores(validIdx);
    class_probs = class_probs(validIdx, :);
    
    % 计算类别分数
    % class_probs 类别概率：表示如果有物体，它属于各个类别的概率
    % obj_scores 物体置信度：表示该框内有物体的概率 与类别无关
    % class_scores = class_probs .* obj_scores;
    class_scores = class_probs;
    [scores, labelIds] = max(class_scores, [], 2);
    
    % 转换边界框格式
    % 将归一化坐标转换为原始图像坐标
    bboxes = zeros(size(bboxes_norm));
    
    % 转换为640x640图像上的像素坐标
    bboxes(:, 1) = bboxes_norm(:, 1) * 640.0;  % x_center
    bboxes(:, 2) = bboxes_norm(:, 2) * 640.0;  % y_center
    bboxes(:, 3) = bboxes_norm(:, 3) * 640.0;  % width
    bboxes(:, 4) = bboxes_norm(:, 4) * 640.0;  % height
    
    % 转换为左上角坐标
    bboxes(:, 1) = bboxes(:, 1) - bboxes(:, 3) / 2;  % x1
    bboxes(:, 2) = bboxes(:, 2) - bboxes(:, 4) / 2;  % y1
    
    % 缩放到原始图像尺寸
    % 将640x640图像上的坐标转换为原始图像坐标
    bboxes = bboxes / scale;
    
    % 确保边界框在图像范围内
    bboxes(:, 1) = max(1, bboxes(:, 1));
    bboxes(:, 2) = max(1, bboxes(:, 2));
    bboxes(:, 3) = max(1, bboxes(:, 3));
    bboxes(:, 4) = max(1, bboxes(:, 4));
    
    % 过滤过小的框
    minSize = 20;
    validSize = bboxes(:, 3) > minSize & bboxes(:, 4) > minSize;
    bboxes = bboxes(validSize, :);
    scores = scores(validSize);
    labelIds = labelIds(validSize);
    
    % 确保在CPU上
    if isa(bboxes, 'gpuArray')
        bboxes = gather(bboxes);
    end
    if isa(scores, 'gpuArray')
        scores = gather(scores);
    end
    if isa(labelIds, 'gpuArray')
        labelIds = gather(labelIds);
    end
end