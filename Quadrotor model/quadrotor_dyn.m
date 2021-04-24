function dr_ = quadrotor_dyn(t,r_)

%% Vehicle parameters
m = 1.215; % toatl mass

% mass moment of inertial
Ixx = 0.01455;
Iyy = 0.0134;
Izz = 0.022;

Ixy = 0;
Iyx = Ixy;

Iyz = 0;
Izy = Iyz;

Izx = 0;
Ixz = Izx;

I = [Ixx Ixy Ixz;
     Iyx Iyy Iyz;
     Izx Izy Izz];

L = 0.45; % distance between diagonally opposite motor shafts
%%

%% Environment parameters
g = 9.81; % magnitude of acceleratopm due to gravity

% coefficient of linear drag
bx = 0;
by = 0;
bz = 0;
b = diag([bx,by,bz]);

% coefficient of angular drag
cx = 0;
cy = 0;
cz = 0;
c = diag([cx,cy,cz]);
%%


%% System of linear equations
x   = r_(1);
y   = r_(2);
z   = r_(3);
dx  = r_(4);
dy  = r_(5);
dz  = r_(6);
phi = r_(7);
th  = r_(8);
psi = r_(9);
p   = r_(10);
q   = r_(11);
r   = r_(12);

J = [1,         0,         -sin(th);
     0,  cos(phi), cos(th)*sin(phi);
     0, -sin(phi), cos(th)*cos(phi)];

Q = QFb(phi,th,psi);

% input thrust and moment in body frame
V = 11.1;

pw1 = 1000;
pw2 = 1000;
pw3 = 1000;
pw4 = 1000;

rpm1 = pwV_to_rpm(pw1,V)
rpm2 = pwV_to_rpm(pw2,V)
rpm3 = pwV_to_rpm(pw3,V)
rpm4 = pwV_to_rpm(pw4,V)

F1 = F(rpm1);
F2 = F(rpm2);
F3 = F(rpm3);
F4 = F(rpm4);

M1 = M(rpm1);
M2 = M(rpm2);
M3 = M(rpm3);
M4 = M(rpm4);

u1 = F1 + F2 + F3 + F4;
u2 = (-F1 - F2 + F3 + F4)*L/(2*sqrt(2));
u3 = (-F1 + F2 + F3 - F4)*L/(2*sqrt(2));
u4 = -M1 + M2 - M3 + M4;

% translational dynamics
ddxyz = [0; 0; -g] + (1/m)*Q*[0; 0; u1] - (1/m)*Q*b*Q'*[dx; dy; dz];
ddx = ddxyz(1);
ddy = ddxyz(2);
ddz = ddxyz(3);

% rotational dynamics
dpqr = I\( [u2; u3; u4] - cross([p; q; r],I*[p; q; r]) - c*[p; q; r] );
dp = dpqr(1);
dq = dpqr(2);
dr = dpqr(3);

% rotational kinematics
dPSI = J\[p; q; r];
dphi = dPSI(1);
dth  = dPSI(2);
dpsi = dPSI(3);

dr_(1)  = dx;
dr_(2)  = dy;
dr_(3)  = dz;
dr_(4)  = ddx;
dr_(5)  = ddy;
dr_(6)  = ddz;
dr_(7)  = dphi;
dr_(8)  = dth;
dr_(9)  = dpsi;
dr_(10) = dp;
dr_(11) = dq;
dr_(12) = dr;

dr_ = dr_';
%%

end

