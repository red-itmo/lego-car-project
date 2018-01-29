//////////////////////////////////////////////////////////
//// The file contains some functions for calculating ////
////// Fresnel integrals, its approximation and //////////
///////////////// some other things //////////////////////
//////////////////////////////////////////////////////////

function y=cosF(t)
  y = cos(%pi/2 * t^2);
endfunction

function y=sinF(t)
  y = sin(%pi/2 * t^2);
endfunction

function y=Cf(r)
  y = intc(0, r, cosF);
endfunction

function y=Sf(r)
  y = intc(0, r, sinF);
endfunction

function y=CfAppr(x)
    y = 0.5 + fAppr(x)*sin(%pi/2*x^2) - gAppr(x)*cos(%pi/2*x^2);
endfunction

function y=SfAppr(x)
    y = 0.5 - fAppr(x)*cos(%pi/2*x^2) - gAppr(x)*sin(%pi/2*x^2);
endfunction

function y=fAppr(x)
    y = (1 + 0.926*x) / (2 + 1.792 * x + 3.104*x^2);
endfunction

function y=gAppr(x)
    y = 1 / (2 + 4.142*x + 3.492*x^2 + 6.670*x^3);
endfunction

function x=xCoord(gamm, alpha, s)
    x = gamm * sqrt(%pi / abs(alpha)) * Cf(sqrt(abs(alpha) / %pi)*s);
endfunction

function y=yCoord(gamm, alpha, s)
    y = gamm * sign(alpha) * sqrt(%pi / abs(alpha)) * Sf(sqrt(abs(alpha) / %pi)*s);
endfunction

function x=xCoordAppr(gamm, alpha, s)
    x = gamm * sqrt(%pi / abs(alpha)) * CfAppr(sqrt(abs(alpha) / %pi)*s);
endfunction

function y=yCoordAppr(gamm, alpha, s)
    y = gamm * sign(alpha) * sqrt(%pi / abs(alpha)) * SfAppr(sqrt(abs(alpha) / %pi)*s);
endfunction

function y = X(b)
    y = sign(b) * sqrt(%pi * abs(b)) * Cf(sqrt(abs(b) / %pi));
endfunction

function y = Y(b)
    y = sqrt(%pi * abs(b)) * Sf(sqrt(abs(b) / %pi));
endfunction

function y = A(b1, b2)
    if argn(2) == 1 then
        y = real(X(b1) * (1 + cos(b1)) + Y(b1) * sin(b1));
    else
        y = real(X(b1) * (1 + cos(b1 + b2)) + Y(b1) * sin(b1 + b2) + sin(0.5*b1 + b2) - sin(0.5*b1));
    end
endfunction

function y = B(b1, b2)
    if argn(2) == 1 then
        y = real(X(b1) * sin(b1) + Y(b1) * (1 - cos(b1)));
    else
        y = real(X(b1) * sin(b1 + b2) + Y(b1) * (1 - cos(b1 + b2)) - cos(0.5*b1 + b2) + cos(0.5*b1));
    end
endfunction

function y = C(b, psi)
    if max(size(b)) == 1 then
        y = real(A(b) * cos(psi) - B(b) * sin(psi));
    else
        y = real(A(b(1),b(2)) * cos(psi) - B(b(1),b(2)) * sin(psi));
    end
endfunction

function y = D(b, psi)
    if max(size(b)) == 1 then
        y = real(A(b) * sin(psi) + B(b) * cos(psi));
    else
        y = real(A(b(1),b(2)) * sin(psi) + B(b(1),b(2)) * cos(psi));
    end
endfunction

function y = G(b, psi)
    y = real( B(b + psi) + D(b, psi) );
endfunction
