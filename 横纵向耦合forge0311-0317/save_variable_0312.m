% 指定要保存的MAT文件路径和文件名
filename = '0312_y_u_changing_with_t_notgood_2.mat';

% 使用save函数将变量保存到MAT文件
save(filename, 'u', 'y');