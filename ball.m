clear all;
clc;

xb=-1;
yb=3;
zb=5;
rb=242/2;
rw=37.5/2;

wb=[xb,yb,zb];

deg120=2/3*pi;
deg90=1/2*pi;
a_a=35.3*pi/180;
b_a=0*pi/180;

a_s=35.3*pi/180;
b_s=0*pi/180;

a1=[sin(a_a)*cos(b_a), sin(a_a)*sin(b_a), cos(a_a)];
a2=[sin(a_a)*cos(b_a+deg120), sin(a_a)*sin(b_a+deg120), cos(a_a)];
a3=[sin(a_a)*cos(b_a-deg120), sin(a_a)*sin(b_a-deg120), cos(a_a)];

ra1=[sin(a_a+deg90)*cos(b_a), sin(a_a+deg90)*sin(b_a), cos(a_a+deg90)];
ra2=[sin(a_a+deg90)*cos(b_a+deg120), sin(a_a+deg90)*sin(b_a+deg120), cos(a_a+deg90)];
ra3=[sin(a_a+deg90)*cos(b_a-deg120), sin(a_a+deg90)*sin(b_a-deg120), cos(a_a+deg90)];

wa1=dot(cross(wb,ra1),cross(a1,ra1))%*rb/rw
wa2=dot(cross(wb,ra2),cross(a2,ra2))%*rb/rw
wa3=dot(cross(wb,ra3),cross(a3,ra3))%*rb/rw

wa1=sin(a_a)*cos(b_a)*wb(1)+sin(a_a)*sin(b_a)*wb(2)+cos(a_a)*wb(3);
wa2=sin(a_a)*cos(b_a+deg120)*wb(1)+sin(a_a)*sin(b_a+deg120)*wb(2)+cos(a_a)*wb(3);
wa3=sin(a_a)*cos(b_a-deg120)*wb(1)+sin(a_a)*sin(b_a-deg120)*wb(2)+cos(a_a)*wb(3);

ws1=wa1;
ws2=wa2;
ws3=wa3;

wbx=(cos(b_s)/(sin(a_s)*(1-cos(deg120))))*ws1+(1/(2*sin(a_s)*sin(deg120)*(1-cos(deg120))))*((sin(b_s-deg120)-sin(b_s))*ws2+(sin(b_s)-sin(b_s+deg120))*ws3)
wby=(sin(b_s)/(sin(a_s)*(1-cos(deg120))))*ws1+(1/(2*sin(a_s)*sin(deg120)*(1-cos(deg120))))*((-cos(b_s-deg120)+cos(b_s))*ws2+(-cos(b_s)+cos(b_s+deg120))*ws3)
wbz=(ws1+ws2+ws3)/(3*cos(a_a))