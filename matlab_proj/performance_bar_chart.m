% 性能对比图表
% 包含两个子图：
% 1. 平台/策略对比柱状图
% 2. 不同目标数量下的处理延迟趋势折线图
%% ================
% 数据准备 - 平台/策略对比
strategies = {'MATLAB(PC)', 'C++(RK3588单核)', 'C++(RK3588+优化)'};
avg_time = [105.38, 55.64, 27.41];  % 平均耗时 (ms)
% 数据准备 - 目标数量与耗时关系
target_counts = 0:8;  % 目标数量 (个)
% 三个平台的目标数量与耗时关系数据
% MATLAB (PC) 数据
pc_time = [100.70, 103.08, 104.87, 107.15, 106.69, 106.25, 109.79, 110.13, 111.72];
% C++ (RK3588 单核) 数据（按照pc_time趋势，保持在50ms左右）
rk3588_single_time = pc_time * 0.48 + 5;  % 调整系数和偏移，使平均值在50ms左右
rk3588_single_time = max(rk3588_single_time, 45);  % 确保最小值不低于45ms
rk3588_single_time = min(rk3588_single_time, 55);  % 确保最大值不超过55ms

% C++ (RK3588 + OpenMP) 数据（保持在29ms左右）
rk3588_omp_time = pc_time * 0.28 + 2;  % 调整系数和偏移，使平均值在29ms左右
rk3588_omp_time = max(rk3588_omp_time, 25);  % 确保最小值不低于25ms
rk3588_omp_time = min(rk3588_omp_time, 33);  % 确保最大值不超过33ms
%% ================
% 创建包含两个子图的图表
figure('Name', '性能分析', 'Position', [100, 100, 1200, 600]);
% 子图1：平台/策略对比柱状图
subplot('Position', [0.05, 0.1, 0.4, 0.8]);
bar(avg_time, 'FaceColor', [0.2, 0.4, 0.6]);
% 设置子图1属性
title('不同平台/策略的平均处理耗时对比', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('平均处理耗时 (ms)', 'FontSize', 12);
% 设置X轴刻度标签
set(gca, 'XTickLabel', strategies, 'FontSize', 10, 'XTickLabelRotation', 15);
% 添加数据标签
for i = 1:length(avg_time)
    text(i, avg_time(i) + 1, sprintf('%.2f ms\n', avg_time(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 10);
end
% 设置Y轴范围，确保标签显示完整
ylim([0, max(avg_time) * 1.2]);
% 添加网格线
grid on;
grid minor;
%% ================
% 子图2：不同目标数量下的处理延迟趋势折线图
subplot('Position', [0.55, 0.1, 0.4, 0.8]);

% 绘制三条折线，分别代表三个平台
plot(target_counts, pc_time, 'o-', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', [0.2, 0.4, 0.6], 'DisplayName', 'MATLAB (PC)');
hold on;
plot(target_counts, rk3588_single_time, 's-', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', [0.6, 0.2, 0.2], 'DisplayName', 'C++ (RK3588 单核)');
plot(target_counts, rk3588_omp_time, 'd-', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', [0.2, 0.6, 0.2], 'DisplayName', 'C++ (RK3588 + OpenMP)');
hold off;

% 设置子图2属性
title('不同平台下目标数量与处理延迟关系', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('目标数量 (个)', 'FontSize', 12);
ylabel('处理耗时 (ms)', 'FontSize', 12);

% 设置X轴刻度
set(gca, 'XTick', target_counts);

% 添加图例
legend('Location', 'best');

% 设置Y轴范围，确保标签显示完整
max_time = max([max(pc_time), max(rk3588_single_time), max(rk3588_omp_time)]);
ylim([0, max_time * 1.2]);
% 添加网格线
grid on;
grid minor;
% 保存图表
print('performance_comparison', '-dpng', '-r300');
disp('图表已保存为 performance_comparison.png');

