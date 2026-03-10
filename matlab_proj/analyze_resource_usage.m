% 分析 resource_usage.log 文件并可视化数据

% 读取日志文件
log_file = 'sample-500/resource_usage.log';
fileID = fopen(log_file, 'r');

% 跳过前几行（标题和说明）
for i = 1:4
    fgetl(fileID);
end

% 初始化数据存储
samples = [];
times = [];
threads = {};
cores = [];
cpu_usages = [];
vmrss = [];

% 读取数据
while ~feof(fileID)
    line = fgetl(fileID);
    if isempty(line) || strcmp(line, '')
        continue;
    end
    
    % 解析行数据 - 使用正则表达式匹配各个字段
    try
        % 匹配整个行的模式
        pattern = 'sample=(\d+) t_ms=(\d+\.\d+) thread=(.*?) core=(\d+) cpu=(\d+\.\d+)% vmrss=(\d+\.\d+)KB';
        matches = regexp(line, pattern, 'tokens');
        
        if ~isempty(matches)
            tokens = matches{1};
            sample = str2double(tokens{1});
            time = str2double(tokens{2});
            thread = tokens{3};
            core = str2double(tokens{4});
            cpu = str2double(tokens{5});
            mem = str2double(tokens{6});
            
            % 存储数据
            samples = [samples; sample];
            times = [times; time];
            threads = [threads, thread];
            cores = [cores; core];
            cpu_usages = [cpu_usages; cpu];
            vmrss = [vmrss; mem];
        end
    catch ME
        % 跳过解析错误的行
        continue;
    end
end

fclose(fileID);

% 唯一线程列表
unique_threads = unique(threads);

% 1. 所有线程的 CPU 使用率随时间变化
figure('Name', '所有线程 CPU 使用率', 'Position', [100, 100, 1000, 600]);
hold on;
colors = jet(length(unique_threads));

for i = 1:length(unique_threads)
    thread_mask = strcmp(threads, unique_threads{i});
    plot(times(thread_mask), cpu_usages(thread_mask), '-', 'Color', colors(i,:), 'DisplayName', unique_threads{i});
end

xlabel('时间 (ms)');
ylabel('CPU 使用率 (%)');
title('各线程 CPU 使用率随时间变化');
legend('Location', 'Best');
grid on;
hold off;

% 2. 所有线程的内存使用随时间变化
figure('Name', '所有线程内存使用', 'Position', [100, 100, 1000, 600]);
hold on;

for i = 1:length(unique_threads)
    thread_mask = strcmp(threads, unique_threads{i});
    plot(times(thread_mask), vmrss(thread_mask), '-', 'Color', colors(i,:), 'DisplayName', unique_threads{i});
end

xlabel('时间 (ms)');
ylabel('内存使用 (KB)');
title('各线程内存使用随时间变化');
legend('Location', 'Best');
grid on;
hold off;

% 3. 各线程的平均 CPU 使用率
figure('Name', '各线程平均 CPU 使用率', 'Position', [100, 100, 800, 600]);
avg_cpu = zeros(length(unique_threads), 1);

for i = 1:length(unique_threads)
    thread_mask = strcmp(threads, unique_threads{i});
    avg_cpu(i) = mean(cpu_usages(thread_mask));
end

bar(1:length(unique_threads), avg_cpu);
set(gca, 'XTick', 1:length(unique_threads), 'XTickLabel', unique_threads, 'XTickLabelRotation', 45);
xlabel('线程');
ylabel('平均 CPU 使用率 (%)');
title('各线程平均 CPU 使用率');
grid on;

% 4. 各线程的平均内存使用
figure('Name', '各线程平均内存使用', 'Position', [100, 100, 800, 600]);
avg_mem = zeros(length(unique_threads), 1);

for i = 1:length(unique_threads)
    thread_mask = strcmp(threads, unique_threads{i});
    avg_mem(i) = mean(vmrss(thread_mask));
end

bar(1:length(unique_threads), avg_mem);
set(gca, 'XTick', 1:length(unique_threads), 'XTickLabel', unique_threads, 'XTickLabelRotation', 45);
xlabel('线程');
ylabel('平均内存使用 (KB)');
title('各线程平均内存使用');
grid on;

% 5. CPU 使用率分布
figure('Name', 'CPU 使用率分布', 'Position', [100, 100, 800, 600]);
histogram(cpu_usages, 20);
xlabel('CPU 使用率 (%)');
ylabel('频率');
title('CPU 使用率分布');
grid on;

% 6. 内存使用分布
figure('Name', '内存使用分布', 'Position', [100, 100, 800, 600]);
histogram(vmrss, 20);
xlabel('内存使用 (KB)');
ylabel('频率');
title('内存使用分布');
grid on;

% 7. 核心使用情况
figure('Name', '核心使用情况', 'Position', [100, 100, 800, 600]);

% 过滤掉 NaN 值
valid_cores = cores(~isnan(cores));
valid_cpu = cpu_usages(~isnan(cores));

if ~isempty(valid_cores)
    max_core = max(valid_cores);
    core_usage = zeros(max_core+1, 1);
    
    for i = 0:max_core
        core_mask = valid_cores == i;
        if any(core_mask)
            core_usage(i+1) = mean(valid_cpu(core_mask));
        else
            core_usage(i+1) = 0;
        end
    end
    
    bar(0:max_core, core_usage);
else
    % 如果没有有效核心数据，显示空图表
    bar(0, 0);
    title('无核心使用数据');
end
xlabel('核心编号');
ylabel('平均 CPU 使用率 (%)');
title('各核心使用情况');
grid on;

% 8. 时间序列中的 CPU 和内存使用趋势
figure('Name', 'CPU 和内存使用趋势', 'Position', [100, 100, 1000, 800]);

subplot(2, 1, 1);
% 计算每个时间点的总 CPU 使用率
[unique_times, ~, idx] = unique(times);
total_cpu = accumarray(idx, cpu_usages, [], @sum);
plot(unique_times, total_cpu);
xlabel('时间 (ms)');
ylabel('总 CPU 使用率 (%)');
title('总 CPU 使用率趋势');
grid on;

subplot(2, 1, 2);
% 计算每个时间点的内存使用（取最大值，因为所有线程共享内存空间）
max_mem = accumarray(idx, vmrss, [], @max);
plot(unique_times, max_mem);
xlabel('时间 (ms)');
ylabel('内存使用 (KB)');
title('内存使用趋势');
grid on;

% 显示所有图表
figures = findobj('Type', 'figure');
for i = 1:length(figures)
    figure(figures(i));
    drawnow;
end

% 输出统计信息
disp('=== 资源使用统计信息 ===');
disp(['总样本数: ', num2str(length(samples))]);
disp(['时间范围: ', num2str(min(times)), ' ms - ', num2str(max(times)), ' ms']);
disp(['平均总 CPU 使用率: ', num2str(mean(cpu_usages)), ' %']);
disp(['平均内存使用: ', num2str(mean(vmrss)), ' KB']);
disp(['最大内存使用: ', num2str(max(vmrss)), ' KB']);

% 按线程输出统计信息
disp('\n=== 各线程统计信息 ===');
for i = 1:length(unique_threads)
    thread_mask = strcmp(threads, unique_threads{i});
    disp(['\n', unique_threads{i}, ':']);
    disp(['  平均 CPU 使用率: ', num2str(mean(cpu_usages(thread_mask))), ' %']);
    disp(['  平均内存使用: ', num2str(mean(vmrss(thread_mask))), ' KB']);
    disp(['  最大 CPU 使用率: ', num2str(max(cpu_usages(thread_mask))), ' %']);
end
