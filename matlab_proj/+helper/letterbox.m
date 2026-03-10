function [img, scale, offset] = letterbox(img, targetSize, varargin)
% LETTERBOX Resize and pad image while maintaining aspect ratio
%
% Inputs:
%   img        - Input image (uint8, single, or double)
%   targetSize - Target size [height, width]
%   varargin   - Optional parameters:
%                'padValue'  - Padding value (default: 114 for uint8, 0.447 for normalized)
%                'stride'    - Stride for auto-padding (default: 32 for YOLOv5)
%                'auto'      - Enable auto-padding (default: true)
%
% Outputs:
%   img        - Processed image
%   scale      - Scaling factor applied
%   offset     - Padding offset [x_offset, y_offset]

    % 参数解析
    p = inputParser;
    addRequired(p, 'img', @(x) ismatrix(x) || ndims(x) == 3);
    addRequired(p, 'targetSize', @(x) isnumeric(x) && numel(x) == 2);
    addParameter(p, 'padValue', [], @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'stride', 32, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'auto', true, @islogical);
    parse(p, img, targetSize, varargin{:});
    
    % 获取输入图像信息
    [h, w, c] = size(img);
    targetH = targetSize(1);
    targetW = targetSize(2);
    
    % 确定填充值
    if isempty(p.Results.padValue)
        % 根据图像数据类型自动选择填充值
        if isinteger(img) || isfloat(img) && max(img(:)) > 1
            % 假设是0-255范围的图像
            padValue = 114;  % YOLOv5默认值
        else
            % 假设是归一化的图像(0-1)
            padValue = 114/255;  % 约0.447
        end
    else
        padValue = p.Results.padValue;
    end
    
    % 计算缩放比例
    scale = min(targetH / h, targetW / w);
    
    % 新尺寸
    newH = round(h * scale);
    newW = round(w * scale);
    
    % 自动调整到stride的倍数（YOLOv5优化）
    if p.Results.auto
        newH = ceil(newH / p.Results.stride) * p.Results.stride;
        newW = ceil(newW / p.Results.stride) * p.Results.stride;
        % 重新计算缩放比例
        scale = min(newH / h, newW / w);
        newH = round(h * scale);
        newW = round(w * scale);
        
        % 确保不超过目标尺寸
        newH = min(newH, targetH);
        newW = min(newW, targetW);
    end
    
    % 计算填充
    padH = targetH - newH;
    padW = targetW - newW;
    
    % 应用填充（在YOLOv5中通常是左上角填充）
    padTop = 0;
    padBottom = padH;
    padLeft = 0;
    padRight = padW;
    
    % 或者可以选择居中填充（根据需求）
    useCenterPadding = false;  % 设置为true使用居中填充
    if useCenterPadding
        padTop = floor(padH / 2);
        padBottom = padH - padTop;
        padLeft = floor(padW / 2);
        padRight = padW - padLeft;
    end
    
    % 调整图像大小
    % 使用更高质量的重采样方法
    imgResized = imresize(img, [newH, newW], 'bilinear', 'Antialiasing', true);

    
    % 创建填充图像
    if isinteger(img)
        % 保持原始数据类型
        imgPadded = zeros(targetH, targetW, c, class(img));
        imgPadded = imgPadded + cast(padValue, class(img));
    else
        imgPadded = ones(targetH, targetW, c, class(img)) * padValue;
    end
    
    % 将调整大小后的图像放入填充图像的左上角
    imgPadded(1:newH, 1:newW, :) = imgResized;
    
    % 如果使用居中填充，需要调整位置
    if useCenterPadding
        imgPadded = zeros(targetH, targetW, c, class(imgPadded)) + padValue;
        rowStart = padTop + 1;
        rowEnd = padTop + newH;
        colStart = padLeft + 1;
        colEnd = padLeft + newW;
        imgPadded(rowStart:rowEnd, colStart:colEnd, :) = imgResized;
    end
    
    img = imgPadded;
    
    % 计算偏移量
    offset = [padLeft, padTop];
    
    % 输出调试信息（可选）
    debug = false;
    if debug
        fprintf('Letterbox处理:\n');
        fprintf('  原始尺寸: %d x %d\n', h, w);
        fprintf('  目标尺寸: %d x %d\n', targetH, targetW);
        fprintf('  缩放比例: %.4f\n', scale);
        fprintf('  调整后尺寸: %d x %d\n', newH, newW);
        fprintf('  填充: [上:%d, 下:%d, 左:%d, 右:%d]\n', ...
            padTop, padBottom, padLeft, padRight);
        fprintf('  偏移: [x:%d, y:%d]\n', offset(1), offset(2));
        fprintf('  填充值: %.3f\n', padValue);
    end
end

%% 兼容性包装函数
function [img, scale, offset] = letterbox_legacy(img, targetSize)
% 保持向后兼容的版本
    warning('使用letterbox_legacy，建议更新到letterbox函数');
    [img, scale, offset] = letterbox(img, targetSize, 'auto', false);
end