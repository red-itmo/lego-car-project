////////////////////////////////////////////////////////
/// Files plots two consecutive compatible clothoids ///
////////////////////////////////////////////////////////

// initialization
clear x y x2 y2 x2_help y2_help
gamm = 1;
alpha = 0.1;
s_end = sqrt(2*%pi / abs(alpha));
i = 1;

// first clothoid calculation
for s = 0:0.01:s_end
    x(i) = X(gamm, alpha, s);
    y(i) = Y(gamm, alpha, s);
    i = i + 1;
end

// HT matrices calculation
theta = alpha * s_end^2 / 2;
T1 = [cos(theta), -sin(theta), x($); sin(theta), cos(theta), y($); 0, 0, 1];
T2 = [[cos(-theta), -sin(-theta), -x($); sin(-theta), cos(-theta), y($); 0, 0, 1]];

// second clothoid calcuation
i = 1;
alpha = -alpha;
for s = 0:0.01:s_end
    x2_help(i) = X(-gamm, alpha, (s_end - s));
    y2_help(i) = Y(-gamm, alpha, (s_end - s));
    new_cords = T1 * inv(T2) * [x2_help(i); y2_help(i); 1];
    x2(i) = new_cords(1);
    y2(i) = new_cords(2);
    i = i + 1;
end

//ploting
plot2d(x, y, 2);
plot2d(x2, y2, 5);
legend("First clothoid", "Second");
