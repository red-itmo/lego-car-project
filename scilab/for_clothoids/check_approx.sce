/////////////////////////////////////////////////////////////////
/// Files checks quality of Fresnel integrals approximation /////
/////////////////////////////////////////////////////////////////

clear x y x2 y2
i = 1;
gamm = 1;
alpha = 0.1;
for s = 0:0.01:(sqrt(2*%pi / abs(alpha)))
    x(i) = X(gamm, alpha, s);
    y(i) = Y(gamm, alpha, s);
    x2(i) = X_appr(gamm, alpha, s);
    y2(i) = Y_appr(gamm, alpha, s);
    i = i + 1;
end
plot2d(x, y);
plot2d(x2, y2, 2);
legend("Клотоида", "Ее аппроксимация", 2);
