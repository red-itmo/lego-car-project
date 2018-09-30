clear
data0 = read("~/Desktop/full.txt", -1, 7);
data1 = read("~/Desktop/first.txt", -1, 7);
data2 = read("~/Desktop/second.txt", -1, 7);

// Plotting of y(x) graphs
scf();
plot2d(data0(:,2), data0(:,3), 1);
plot2d(data1(:,2), data1(:,3), 2);
plot2d(data2(:,2), data2(:,3), 5);
xtitle("Зависимости y(x)");
legend("Выход getCoordinatesTime", "Первая клотоида", "Вторая", 2);


// Trying to integrate velocities from files
for i = 1:max(size(data1))
    x1_integrated(i) = inttrap(data1([1:i],1), data1([1:i],4));
    y1_integrated(i) = inttrap(data1([1:i],1), data1([1:i],5));
end
for i = 1:max(size(data2))
    x2_integrated(i) = inttrap(data2([1:i],1), data2([1:i],4));
    y2_integrated(i) = inttrap(data2([1:i],1), data2([1:i],5));

end
x = data0(:,2);
y = data0(:,3);

scf();
plot2d(x, y, 1);
plot2d(x1_integrated + x(1), y1_integrated + y(1), 2);
plot2d(x2_integrated + x1_integrated($) + x(1), y2_integrated + x1_integrated($) + y(1), 5);
xtitle("Зависимости y(x) (полученные интегрированием скоростей)");
legend("Выход getCoordinatesTime", "Первая клотоида", "Вторая", 2);


// Trying to integrate accelerations from files
for i = 1:max(size(data1))
    vx1_integrated(i) = inttrap(data1([1:i],1), data1([1:i],6));
    vy1_integrated(i) = inttrap(data1([1:i],1), data1([1:i],7));
end
for i = 1:max(size(data2))
    vx2_integrated(i) = inttrap(data2([1:i],1), data2([1:i],6));
    vy2_integrated(i) = inttrap(data2([1:i],1), data2([1:i],7));

end
vx = data0(:,4);
vy = data0(:,5);

scf();
plot2d(vx, vy, 1);
plot2d(vx1_integrated + vx(1), vy1_integrated + vy(1), 2);
plot2d(vx2_integrated + vx1_integrated($) + vx(1), vy2_integrated + vy1_integrated($) + vy(1), 5);
xtitle("Зависимости vy(vx) (полученные интегрированием ускорений)");
legend("Выход getCoordinatesTime", "Скорости у первой клотоиды", "Второй", 4);
