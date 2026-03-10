function detector = initYolov5Detector(modelPath, classNames, executionEnvironment)
    % initYolov5Detector - 初始化YOLOv5检测器
    %   detector = initYolov5Detector(modelPath, classNames, executionEnvironment)
    % 
    % 输入参数:
    %   modelPath - 模型文件路径，默认为'models/yolov5s.mat'
    %   classNames - 类别名称，默认为COCO数据集类别
    %   executionEnvironment - 执行环境，可选'auto'、'gpu'或'cpu'，默认为'auto'
    % 
    % 输出参数:
    %   detector - 检测器结构体，包含以下字段:
    %       .net - 加载的网络模型
    %       .classNames - COCO类别名称
    %       .numClasses - 类别数量
    %       .executionEnvironment - 执行环境
    %       .targetSize - 目标输入尺寸
    %       .confThreshold - 置信度阈值
    %       .nmsThreshold - NMS阈值
    
    % 默认参数
    if nargin < 1 || isempty(modelPath)
        modelPath = 'models/yolov5s.mat';
    end
    
    if nargin < 2 || isempty(classNames)
        % 默认COCO类别名称
        classNames = {'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'};
    end
    
    if nargin < 3 || isempty(executionEnvironment)
        executionEnvironment = 'auto';
    end
    
    % 确定执行环境
    if strcmp(executionEnvironment, 'auto')
        if gpuDeviceCount > 0
            executionEnvironment = 'gpu';
        else
            executionEnvironment = 'cpu';
        end
    end
    
    % 加载网络
    data = load(modelPath);
    net = data.net;
    
    % 获取类别信息

    numClasses = size(classNames, 2);
    
    % 构建检测器结构体
    detector.net = net;
    detector.classNames = classNames;
    detector.numClasses = numClasses;
    detector.executionEnvironment = executionEnvironment;
    detector.targetSize    = [640, 640];  % YOLOv5标准输入大小
    detector.confThreshold = 0.25;     % 置信度阈值
    detector.nmsThreshold  = 0.45;      % NMS阈值
    
    % 显示初始化信息
    fprintf('YOLOv5检测器初始化完成:\n');
    fprintf('  执行环境: %s\n', executionEnvironment);
    fprintf('  类别数量: %d\n', numClasses);
    fprintf('  目标输入尺寸: %dx%d\n', detector.targetSize(1), detector.targetSize(2));
    fprintf('  置信度阈值: %.2f\n', detector.confThreshold);
    fprintf('  NMS阈值: %.2f\n', detector.nmsThreshold);
    fprintf('\n');
end