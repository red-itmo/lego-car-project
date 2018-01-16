// for search of contact point of bottom arc and inclined line
function answ = contact_point(point_7, point_4, R)

    x_7 = point_7(1);
    y_7 = point_7(2);
    x_4 = point_4(1);
    y_4 = point_4(2);

    function [y] = f1(x)
        y(1) = (x(1) - x_7)^2 + (x(2) - y_7)^2 - R^2;
        y(2) = (x(1) - x_7)*(x_4 - x(1)) + (x(2) - y_7)*(y_4- x(2));
    endfunction

    answ = fsolve([x_7 + R/2, y_7 - R/2], f1);

endfunction


// gradient descent optimization algorithm
function answ = grad_search(point_7, point_4, R)

    x_7 = point_7(1);
    y_7 = point_7(2);
    x_4 = point_4(1);
    y_4 = point_4(2);

    // gamma coefficient
    GAMMA = 2.0;

    // some initial values
    i = 1;
    alpha(i) = %pi/4;
    e = %inf;

    // main loop
    while abs(e) > 0.001 then
        e = (y_4-y_7) - (x_4-x_7)*tan(alpha(i)) + R*cos(alpha(i)) + R*tan(alpha(i))*sin(alpha(i));
        deriv_e = -(x_4 - x_7)/cos(alpha(i))^2 + R*sin(alpha(i))/cos(alpha(i))^2;
        alpha(i+1) = alpha(i) - GAMMA * e * deriv_e;
        i = i + 1;
    end

    // contact point coordinates
    x_5 = x_7 + R * sin(alpha(i));
    y_5 = y_7 - R * cos(alpha(i));
    answ = [x_5, y_5];

    // some logs
    printf("\nGradient descent optimization algorithm ends search in %d steps.\n", i);
    scf(1);
    plot2d(alpha);
    xlabel("Step number");
    ylabel("Value of alpha angle");

endfunction


// Newton's method
function answ = newton_method(point_7, point_4, R)

    x_7 = point_7(1);
    y_7 = point_7(2);
    x_4 = point_4(1);
    y_4 = point_4(2);

    // some initial values
    i = 1;
    alpha(i) = %pi/4;
    e = %inf;

    // main loop
    while abs(e) > 0.001 then
        e = (y_4-y_7) - (x_4-x_7)*tan(alpha(i)) + R*cos(alpha(i)) + R*tan(alpha(i))*sin(alpha(i));
        deriv_e = - (x_4 - x_7) / cos(alpha(i))^2 + R*sin(alpha(i))/cos(alpha(i))^2;
        alpha(i+1) = alpha(i) - e / deriv_e;
        i = i + 1;
    end

    // contact point coordinates
    x_5 = x_7 + R * sin(alpha(i));
    y_5 = y_7 - R * cos(alpha(i));
    answ = [x_5, y_5];

    // some logs
    printf("\nNewton method ends search in %d steps.\n", i);
    scf(1);
    plot2d(alpha);
    xlabel("Step number");
    ylabel("Value of alpha angle");

endfunction
