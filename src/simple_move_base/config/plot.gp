# 设置输出文件
set terminal pngcairo enhanced font 'Arial,10'
set output 'line_plot_first_row.png'

# 设置标题和标签
set title "Line Plot using First Row of Data from Text File"
set xlabel "X-axis"
set ylabel "Y-axis"
set grid

# 读取和绘制数据
plot 'data.txt' using 1:2 with linespoints title 'Data Points'

# 保存图表并重置
unset output