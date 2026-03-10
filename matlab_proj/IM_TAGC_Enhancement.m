function [img_out,Gamma] = IM_TAGC_Enhancement(Src)
    % Src  : uint8 RGB image, size [H, W, 3]
    % Dest : uint8 RGB image

    assert(ndims(Src) == 3 && size(Src,3) == 3, ...
        'Input must be an RGB image');
    % 转 double 并归一化
    Src_f = double(Src) / 255;
    % 分离通道
    B = Src_f(:,:,1);
    G = Src_f(:,:,2);
    R = Src_f(:,:,3);
    % -------- 亮度与平均值 --------
    L = 0.2126 .* R + 0.7152 .* G + 0.0722 .* B;
    A = (R + G + B) / 3;
    % -------- Gamma 计算 --------
    Gamma = 5.0 + (0.5 - L) .* (1 - A) - 2 .* L;
    % （保持与 C 代码一致，不做限幅）
    % 如果用于工程，建议加：
    

    % -------- Gamma 映射 --------
    expVal = 2 ./ Gamma;
    expVal = max(expVal, 0.3);
    B_out = B .^ expVal;
    G_out = G .^ expVal;
    R_out = R .^ expVal;

    % -------- 回到 uint8 --------
    Dest = uint8(min(255, max(0,cat(3, B_out, G_out, R_out) * 255 + 0.4999999)));
    Hist = imhist(Dest);
    % 生成映射表
    Map = MakeMapping_MATLAB(Hist, 0.03, 1.0);
    % 应用映射
    img_out = Map(double(Dest) + 1);

    function Map = MakeMapping_MATLAB(Histgram, CutLimit, Contrast)
        % Histgram : 1x256 或 256x1 的直方图
        % CutLimit : 裁剪比例（默认 0.01）
        % Contrast : 对比度系数（默认 1）
        % Map      : 1x256 灰度映射表（0~255）
        if nargin < 2
            CutLimit = 0.01;
        end
        if nargin < 3
            Contrast = 1;
        end
        Histgram = double(Histgram(:)');   % 保证行向量
        Level = 256;
        % -------- 总像素数 --------
        Amount = sum(Histgram);
        % -------- 非零最小 / 最大灰度 --------
        Min = 0; Max = 255;
        for I = 1:Level
            if Histgram(I) ~= 0
                Min = I-1;
                break;
            end
        end
        for I = Level:-1:1
            if Histgram(I) ~= 0
                Max = I-1;
                break;
            end
        end
        % -------- CutLimit 裁剪 --------
        Sum = 0;
        MinB = 0; MaxB = 255;
    
        for I = 1:Level
            Sum = Sum + Histgram(I);
            if Sum >= Amount * CutLimit
                MinB = I-1;
                break;
            end
        end
    
        Sum = 0;
        for I = Level:-1:1
            Sum = Sum + Histgram(I);
            if Sum >= Amount * CutLimit
                MaxB = I-1;
                break;
            end
        end
    
        % -------- 对比度扩展 --------
        Delta = (Max - Min) * Contrast * 0.5;
        Min = Min - Delta;
        Max = Max + Delta;
    
        % 限幅
        Min = max(Min, 0);
        Max = min(Max, 255);
    
        % -------- 生成映射表 --------
        Map = zeros(1, Level);
    
        if MaxB ~= MinB
            for I = 0:255
                if I < MinB
                    Map(I+1) = Min;
                elseif I > MaxB
                    Map(I+1) = Max;
                else
                    Map(I+1) = (Max - Min) * (I - MinB) / (MaxB - MinB) + Min;
                end
            end
        else
            % 图像非常平坦的情况
            Map(:) = MaxB;
        end
    
        % -------- 输出 uint8 映射 --------
        Map = uint8(round(Map));
    
    end

end
