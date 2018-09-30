//////////////////////////////////////////////////////////////
//// Files plots several clothoids with identical "phase" ////
//// and different alphas ////////////////////////////////////
//////////////////////////////////////////////////////////////

// different clothoids calculating
k = 1;
gamm = 1;
for j = 1:20
    alpha = j / 10 / 2;
    clear x y
    i = 1;
    for s = 0:0.01:(sqrt(2*%pi / abs(alpha)))
        x(i) = X(gamm, alpha, s);
        y(i) = Y(gamm, alpha, s);
        i = i+1;
    end
    xe(k) = x($);
    ye(k) = y($);
    k = k + 1;
    plot2d(x, y, j);
end

// lime through last points
regr = [xe, ones(xe)];
plot2d(xe, ye);
disp(pinv(regr)*ye, "k1, k0 =");
