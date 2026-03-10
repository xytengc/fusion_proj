function [bboxes, scores, labelIds] = detectWithYolov5(detector, image, varargin)
    % detectWithYolov5 - 使用YOLOv5检测器进行目标检测
    %   [bboxes, scores, labels] = detectWithYolov5(detector, image, 'Option', value, ...)
    % 
    % 输入参数:
    %   detector        - 由initYolov5Detector返回的检测器结构体
    %   image           - 输入图像
    %   'ConfThreshold' - 置信度阈值，默认为detector.confThreshold
    %   'NMSThreshold'  - NMS阈值，默认为detector.nmsThreshold
    % 
    % 输出参数:
    %   bboxes    - 边界框，格式为 [x, y, width, height]
    %   scores    - 置信度分数
    %   labelIds  - 类别标签索引
    
    % 解析输入参数
    p = inputParser;
    addRequired(p, 'detector');
    addRequired(p, 'image');
    addParameter(p, 'ConfThreshold', detector.confThreshold, @isnumeric);
    addParameter(p, 'NMSThreshold', detector.nmsThreshold, @isnumeric);
    parse(p, detector, image, varargin{:});
    
    confThreshold = p.Results.ConfThreshold;
    nmsThreshold = p.Results.NMSThreshold;
    
    % 执行IM_TAGC_Enhancement图像增强
    enhancedImage = IM_TAGC_Enhancement(image);
    
    % 执行检测
    [bboxes, scores, labelIds] = yolov5Predict(enhancedImage, detector.net, detector.numClasses, detector.executionEnvironment);
    
    % 过滤低分数结果
    if ~isempty(scores)
        validIdx = scores > confThreshold;
        bboxes = bboxes(validIdx, :);
        scores = scores(validIdx);
        labelIds = labelIds(validIdx);
        % 执行非极大值抑制
        if ~isempty(scores)
            [bboxes, scores, labelIds] = selectStrongestBboxMulticlass(bboxes, scores, labelIds, 'OverlapThreshold', nmsThreshold);
        end
    end
end