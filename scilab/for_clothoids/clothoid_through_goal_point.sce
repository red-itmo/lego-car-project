/////////////////////////////////////////////////////////////////
//// Files finds a clothoid which goes through specified point //
/////////////////////////////////////////////////////////////////

// goal coordinates (INPUT)
x_des = 4;
y_des = 5;


// check for input data validity
k = y_des / x_des;
if k > 1.34 then
    disp("Invalid input data!!!");
    abort;
end


// some constants' values search
if x_des > 0 then
    gamm = 1;
else
    gamm = -1;
end
if y_des > 0 then
    bet = 1;
else
    bet = -1;
end
if x_des*y_des > 0 then
    alpha_sign = 1;
else
    alpha_sign = -1;
end
printf("\nValues of needed constants:\n");
printf("gamma = %f\nbeta = %f\nalpha_sign = %f\n", gamm, bet, alpha_sign);


//// e(s) function examination
//i = 1;
//clear x y
//for s = 0:0.1:5
//    x(i) = X_appr(gamm, 1, s);
//    y(i) = Y_appr(gamm, 1, s);
//    i = i + 1;
//end
//e = k * x - y;
//s = 0:0.1:5;
//plot2d(s, e', 2);
//plot2d(s, e.^2', 5);
//xlabel("$s$",'fontsize',4);
//ylabel("$e$",'fontsize',4);
//legend("$e(s)$", "$e^2(s)$");


// finding of initial value for s for gradient descent
e = sign(bet);
s = 0.0;
while sign(bet)*e > 0
    s = s + 0.1;
    x = X_appr(gamm, alpha_sign, s);
    y = Y_appr(gamm, alpha_sign, s);
    e = k * x - y;
end
printf("\nPreprocessed initial value for s: %f\n", s);


// finding of s_end for clothoid with abs(alpha) = 1
while abs(e) > 0.001 then
    x = X_appr(gamm, alpha_sign, s);
    y = Y_appr(gamm, alpha_sign, s);
    e = k * x - y;
    deriv_e = gamm * (k*cos(s^2/2) - sign(alpha_sign) * sin(s^2/2));
    s = s - 0.1 * e * deriv_e;
end


// scale the s_end to true s_end and find alpha
x = gamm * sqrt(%pi) * Cf(sqrt(1/ %pi)*s);
phi = s^2 /2;
alpha = (x / x_des)^2 * sign(alpha_sign);
s = sqrt(2*phi / abs(alpha));
printf("\nValues for s and alpha: %f, %f\n", s, alpha);


// calculating for plotting
clear x y
for se = 0:0.01:s
    x(i) = X_appr(gamm, alpha, se);
    y(i) = Y_appr(gamm, alpha, se);
    i = i + 1;
end

//plotting
plot(x_des,y_des,'r.')
plot(x_des+sign(x_des)*0.1*abs(x_des), y_des+sign(y_des)*0.1*abs(y_des),'w')
plot2d(x,y,2)
