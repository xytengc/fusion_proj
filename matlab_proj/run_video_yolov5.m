clear
clc;
close all;
%% 初始化YOLOv5检测器
modelPath = 'models/yolov5s_cp3.mat';
classNames = helper.getCOCOClassNames1;
if gpuDeviceCount > 0
    executionEnvironment = 'gpu';
else
    executionEnvironment = 'cpu';
end
detector = initYolov5Detector(modelPath,classNames,executionEnvironment);
%% 选择视频文件
% uigetfile 选择文件  uigetdir 选择目录
folderPath = uigetdir('E:\hducc\Data_process\radar_data\data\', '选择视频文件');
if folderPath == 0
    disp('未选择视频文件，程序终止。');
    return;
end
%%
cameraParamsFile = 'E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\Radar\radar_test\cameraParams640.mat';
load(cameraParamsFile);
K_camera = cameraParams640.K;
fileName = split(folderPath,'\');
videoPath = fullfile(folderPath, 'video10.mp4');
videoReader = VideoReader(videoPath);
figure(1);
hVideo = imshow(zeros(videoReader.Height, videoReader.Width, 3, 'uint8'));
hold on;
%% 设置输出目录和文件
output_dir = "E:\hducc\FPGA\Image_process\image_math\Radar_Image_Fusion\D-S Fusion\data\day_data";
% 确保输出目录存在
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
% 生成输出文件名
output_txt = fullfile(output_dir, sprintf('%s_detection.txt', string(fileName(6))));
fid = fopen(output_txt, 'w');
%% 处理视频每一帧
frameCount = 0;
totalFrames = videoReader.NumFrames;
fprintf('开始处理视频，共%d帧\n', totalFrames);
while hasFrame(videoReader)
    frame = readFrame(videoReader);
    frameCount = frameCount + 1;
    % 执行目标检测
    t0 = tic;
    % frame = undistortImage(frame, cameraParams640,'OutputView', 'same');
    % frame = IM_TAGC_Enhancement(frame);
    [bboxes, scores, labelIds] = detectWithYolov5(detector, frame,'ConfThreshold', 0.5,'NMSThreshold', 0.45);
    tEnd1 = toc(t0);
    % 映射标签
    if ~isempty(labelIds)
        labels = detector.classNames(labelIds);
    else
        labels = {};
    end
    % 映射标签
    annotations = string(labels') + ": " + string(scores);
    % 标注检测结果 
    fps = 1 / tEnd1;
    frame = insertText(frame, [10, 30], sprintf('FPS: %.2f', fps), ...
                         'FontSize', 12, 'BoxColor', 'green');
    if ~isempty(bboxes)
    frame = insertObjectAnnotation(frame, 'rectangle', bboxes, annotations, ...
                                   'Color', 'yellow', 'LineWidth', 2);
    end
    set(hVideo, 'CData', frame);
    title(sprintf('已处理%d/%d帧\n', frameCount, totalFrames));
    drawnow;
    % 保存检测结果到txt文件
    fprintf(fid, '=== 第%d帧 ===\n', frameCount);
    fprintf(fid, '检测时间: %0.4f s\n', tEnd1);
    if ~isempty(bboxes)
        fprintf(fid, '目标数: %d\n', size(bboxes,1));
        for i = 1:length(bboxes(:,1))
            fprintf(fid, '目标%d 类型: %s  置信度：%0.4f\n', i, string(labels(i)), scores(i));
            fprintf(fid, '目标%d 位置: %0.4f %0.4f %0.4f %0.4f\n', i, bboxes(i,:));
        end
    else
        fprintf(fid, '未检测到目标\n');
    end
    fprintf(fid, '\n');
    % 显示处理进度
    if mod(frameCount, 100) == 0
        fprintf('已处理%d/%d帧 (%.1f%%)\n', frameCount, totalFrames, frameCount/totalFrames*100);
    end
end
%% 关闭文件和清理
fclose(fid);
fprintf('视频处理完成！\n');
fprintf('检测结果已保存到: %s\n', output_txt);
% 释放视频读取器
clear videoReader;