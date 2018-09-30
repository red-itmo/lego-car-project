L = 0.1884;
PHI_MAX = %pi / 4;
w = 4.8 / 2;
KP1 = w^2;
KP2 = w^2;
KD1 = 2*w;
KD2 = 2*w;
path = get_absolute_file_path('control_1.sce');
scheme_name = path + 'control_1.zcos';
importXcosDiagram(scheme_name);
xcos_simulate(scs_m, 4);

plot2d(ref.values(:,1), ref.values(:,2));
plot2d(car.values(:,1), car.values(:,2), 5);
legend("Желаемая", "Реальная",4);
a = gca();
a.title.text = "Траектории движения робота"
a.title.font_size = 3;
a.x_label.text = "$x,\text{ м}$";
a.x_label.font_size = 4;
a.y_label.text = "$y,\text{ м}$";
a.y_label.font_size = 4;
legend_ = a.children(1);
legend_.font_size = 3;
scf();
plot2d(car.time, car.values(:,3));

