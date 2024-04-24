% 指定要保存的MAT文件路径和文件名
filename = '0422_step_good.mat';

% 使用save函数将变量保存到MAT文件
save(filename, 'Statelog', 'carsim_out','t','auxdatalog');