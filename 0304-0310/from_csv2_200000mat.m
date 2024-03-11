% 指定CSV文件路径
file = 'use_0301_10000x3.csv';
% 使用csvread函数读取数据
data = readmatrix(file);
% 提取数据
xcoord = data(1:10000, 1);
ycoord = data(1:10000, 2);
% 定义要插入的点数量
num = 19; % 5对应的是59995
% 计算每两个值之间的插值步长
step = 1 / (num + 1);

% 初始化新的x向量和计数器
x_new = [];
count = 1;

% 遍历原始x向量
for i = 1:length(xcoord)-1
    % 添加当前原始值到新的x向量
    x_new(count) = xcoord(i);
    count = count + 1;
    
    % 计算两个原始值之间的插值步长
    interval = (xcoord(i+1) - xcoord(i)) * step;
    
    % 插入均匀间隔的点
    for j = 1:num
        x_new(count) = xcoord(i) + j * interval;
        count = count + 1;
    end
end

% 添加最后一个原始值到新的x向量
x_new(count) = xcoord(end);
%% y的
% 初始化新的y向量和计数器
y_new = [];
count = 1;

% 遍历原始y向量
for i = 1:length(ycoord)-1
    % 添加当前原始值到新的y向量
    y_new(count) = ycoord(i);
    count = count + 1;
    
    % 计算两个原始值之间的插值步长
    interval = (ycoord(i+1) - ycoord(i)) * step;
    
    % 插入均匀间隔的点
    for j = 1:num
        y_new(count) = ycoord(i) + j * interval;
        count = count + 1;
    end
end
% 添加最后一个原始值到新的y向量
y_new(count) = ycoord(end);

plot(xcoord, ycoord, 'o', x_new, y_new, 'o');
xlabel('x');
ylabel('y');
legend('原始路点', '插值后的路点');

xcoord = x_new;
ycoord = y_new;

% 指定要保存的MAT文件路径和文件名
filename = 'use_0301_199981x3.mat';

% 使用save函数将变量保存到MAT文件
save(filename, 'xcoord', 'ycoord');