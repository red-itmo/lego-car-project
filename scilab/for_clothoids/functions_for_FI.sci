//////////////////////////////////////////////////////////
//// The file contains some functions for calculating ////
//// Fresnel integrals and its approximation /////////////
//////////////////////////////////////////////////////////

function y=C(t)
  y = cos(%pi/2 * t^2);
endfunction

function y=S(t)
  y = sin(%pi/2 * t^2);
endfunction

function y=Cf(r)
  y = intc(0, r, C);
endfunction

function y=Sf(r)
  y = intc(0, r, S);
endfunction

function y=Cf_appr(x)
    y = 0.5 + fa(x)*sin(%pi/2*x^2) - ga(x)*cos(%pi/2*x^2);
endfunction

function y=Sf_appr(x)
    y = 0.5 - fa(x)*cos(%pi/2*x^2) - ga(x)*sin(%pi/2*x^2);
endfunction

function y=fa(x)
    y = (1 + 0.926*x) / (2 + 1.792 * x + 3.104*x^2);
endfunction

function y=ga(x)
    y = 1 / (2 + 4.142*x + 3.492*x^2 + 6.670*x^3);
endfunction

function x=X(gamm, alpha, s)
    x = gamm * sqrt(%pi / abs(alpha)) * Cf(sqrt(abs(alpha) / %pi)*s);
endfunction

function y=Y(gamm, alpha, s)
    y = gamm * sign(alpha) * sqrt(%pi / abs(alpha)) * Sf(sqrt(abs(alpha) / %pi)*s);
endfunction

function x=X_appr(gamm, alpha, s)
    x = gamm * sqrt(%pi / abs(alpha)) * Cf_appr(sqrt(abs(alpha) / %pi)*s);
endfunction

function y=Y_appr(gamm, alpha, s)
    y = gamm * sign(alpha) * sqrt(%pi / abs(alpha)) * Sf_appr(sqrt(abs(alpha) / %pi)*s);
endfunction
