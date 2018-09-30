// custom color (orange)
kolor = color(255,128,0);

// for plotting arcs
function plot_arc(x_c, y_c, x_1, y_1, x_f, y_f)

    alpha_1 = atan(y_1 - y_c, x_1 - x_c);
    alpha_2 = atan(y_f - y_c, x_f - x_c);

    delta_alpha = alpha_2 - alpha_1;
    if delta_alpha > %pi then
        delta_alpha = delta_alpha - 2*%pi;
    elseif delta_alpha < -%pi then
        delta_alpha = delta_alpha + 2*%pi;
    end

    alphas = alpha_1:sign(delta_alpha)*0.01:(alpha_1+delta_alpha);
    R = sqrt((x_1 - x_c)^2 + (y_1 - y_c)^2);
    x = x_c + R * cos(alphas);
    y = y_c + R * sin(alphas);

    plot2d(x ,y, kolor);

endfunction


///////////////////////////////////////////////////////
///////////////// BODY STARTS HERE/////////////////////
///////////////////////////////////////////////////////

// for definition of functions which find contact point
path = get_absolute_file_path('trajectory_planning.sce');
exec(path + 'contact_point.sci');

// input data
x_1 = 3;
y_1 = 1;
x_l = 1;
y_l = 0;
x_r = 2.0;
y_r = 0;
delta_1 = 0.1;
delta_2 = 0.3;
d = 0.1; // half of robot's width (or "depth")
R = 0.4;

// calculation of key points' coordinates
x_4 = x_r - delta_2;
y_4 = y_r + delta_2;
x_6 = x_l + delta_1;
y_6 = y_l - d;
x_7 = x_6;
y_7 = y_6 + R;

answ = contact_point([x_7, y_7], [x_4, y_4], R);
x_5 = answ(1);
y_5 = answ(2);

k = (y_4 - y_5) / (x_4 - x_5);
alpha = atan(k);
b = y_4 - k * x_4;
h = R / cos(alpha);
x_8 = 1 / k * (y_1 - R - b + h);
y_8 = y_1 - R;

x_2 = x_8;
y_2 = y_1;
x_3 = x_8 - R*cos(alpha);
y_3 = y_8 + R*sin(alpha);

// plotting (be careful because it is in reverse order)
scf(0);
// arc at the bottom
plot_arc(x_7, y_7, x_6, y_6, x_5, y_5);
// inclined line
plot2d([x_5:0.01:x_3], k*[x_5:0.01:x_3] + b, kolor);
// arc at the top
plot_arc(x_8, y_8, x_3, y_3, x_2, y_2);
//horizontal line
plot2d([x_2:0.01:x_1], y_1*ones([x_2:0.01:x_1]), kolor);
// plotting of initial points
plot(x_1, y_1,'g.');
plot(x_l, y_l,'g.');
plot(x_r, y_r,'g.');
// for beauty
plot(3.1,1.1);
plot(0.9,-0.2);
xgrid();
