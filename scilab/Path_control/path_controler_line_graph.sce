clear
kth=30;
ke=-50;
ks=0;
L=0.165;
fi0=0;
col=5;
xrobot=-1;
yrobot=-1;
x0=0;
y0=0;
theta0=%pi/5;
alp=%pi/2;
Vx=-0.3;

//if Vx<0 then
//    Vx=abs(Vx);
//    if theta0>=0 then
//        theta0=theta0-%pi;
//    else
//        theta0=theta0+%pi;
//    end
//end


ts=[-cos(alp);-sin(alp)];

aaatan=atan(ts(2),ts(1));



path=get_absolute_file_path("path_controler_line_graph.sce");
importXcosDiagram(path+'path_controler_line3.zcos');
xcos_simulate(scs_m,4);
scf();
subplot (241); plot2d(w.time,w.values,col); title('$\omega_*$','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (242); plot2d(e.time,e.values,col); title('e','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (243); plot2d(theta_e.time,theta_e.values,col); title('theta_e','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (244); plot2d(theta.time,theta.values,col); title('theta','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (245); plot2d(fi.time,fi.values,col); title('Fi','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (246); plot2d(fi2.time,fi2.values,col); title('Fi2','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (247); plot2d(S.time,S.values,col); title('S','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (248); plot2d(V.time,V.values,col); title('V','fontsize',3);
g=get('current_axes');
g.grid=[0 0];

scf();
subplot (121); plot2d4(x_r.values,y_r.values,col); title('x,y','fontsize',3);
g=get('current_axes');
g.grid=[0 0]; 
subplot (122); plot2d4(xref.values,yref.values,col); title('xref,yref','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
scf();
subplot (121); plot2d4(xref.time,xref.values,col); title('xref','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (122); plot2d4(x_r.time,x_r.values,col); title('x_r','fontsize',3);
g=get('current_axes');
g.grid=[0 0];

scf();
subplot (121); plot2d4(yref.time,yref.values,col); title('yref','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
subplot (122); plot2d4(y_r.time,y_r.values,col); title('y_r','fontsize',3);
g=get('current_axes');
g.grid=[0 0];
