clear
kth=20;
ke=-50;
L=0.165;
fi0=0;
col=5;
xrobot=5;
yrobot=5;
x0=0;
y0=3;
theta0=0;
xc=0;
yc=0;
direction='cw';
R=sqrt((x0-xc)^2+(y0-yc)^2);
if direction=='cw' then
    ks=1/R;
else
    ks=-1/R;
end

alp0=atan((y0-yc),(x0-xc));

path=get_absolute_file_path("path_controler_circle_graph.sce");
importXcosDiagram(path+'path_controler_circle.zcos');
xcos_simulate(scs_m,4);
scf();
subplot (321); plot2d(w.time,w.values,col); title('$\omega_*$','fontsize',3);
subplot (322); plot2d(e.time,e.values,col); title('e','fontsize',3);
subplot (323); plot2d(theta.time,theta.values,col); title('theta','fontsize',3);
subplot (324); plot2d(V.time,V.values,col); title('V','fontsize',3);
subplot (325); plot2d(fi.time,fi.values,col); title('fi','fontsize',3);
subplot (326); plot2d(alp.time,alp.values,col); title('alp','fontsize',3);
scf();
subplot (131); plot2d4(x_r.values,y_r.values,col); title('x,y','fontsize',3);
subplot (132); plot2d4(xref.values,yref.values,col); title('xref,yref','fontsize',3);
subplot (133); plot2d4(S.time,yref.values,col); title('xref,yref','fontsize',3);