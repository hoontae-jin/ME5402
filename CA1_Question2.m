clc; clear; close all;
%% Question 2
% a) Find the Jacobian matrix that relates the joint velocities to the linear velocity of the endpoint
syms t1 l1 d2 d3 % unknown parameters in the DH table
% The DH Convention Table
%-------------------------------------------------------------------------%
%Joint   |   Joint Angle   |   Link Offset   |   Link Length  | Link Twist%
%  1     |       t1        |         l1      |        0       |     0     %        
%  2     |     pi/2        |         d2      |        0       |   pi/2    %
%  3     |       0         |         d3      |        0       |     0     %
%-------------------------------------------------------------------------%
link(1).dh = [t1 l1 0 0]; % ".dh" creates a matrix containing the DH details for a certain link
link(2).dh = [pi/2 d2 0 pi/2];
link(3).dh = [0 d3 0 0];
% With the details given above, we use the forward kinematic to create
% matrices as below.
for i = 1:3 % function for the matrix creation can be found at the bottom (TransformMat)
    link(i).mat = DH_mat(link(i)); %link(1-3).mat contains the link matrices (A0_1, A1_2 and A2_3).
end 
% Construct T0_2 and T0_3 to obtain r(i-1)_e and b(i-1)_e subsequently
T0_2 = link(1).mat*link(2).mat;
T0_3 = T0_2*link(3).mat;
% b and r
b0 = [0 ; 0 ; 1];
b1 = link(1).mat(1:3,3);
b2 = T0_2(1:3,3);

r2_end = d3*T0_3(1:3,1)
r1_end = d2*T0_2(1:3,1) + r2_end ;
r0_end = l1*b0 + r1_end;

% Substitute all the vectors into the Jacobian formula
% Prismatic : [b(i-1) ; 0], Revolute : [b(i-1)xr(i-1)_end ; b(i-1)]
% Joint 1: Revolute, Joint 2: Prismatic, Joint 3: Prismatic
J = [cross(b0,r0_end),b1,b2;b0,[0;0;0],[0;0;0]]; % Answer
% b)Write a program to compute the equivalent joints’ torques/forces corresponding to the endpoint force
J_numeric = subs(J,[t1, d2, d3], [0, 1, 1]);
syms f1 f2 f3 n1 n2 n3
F = subs([f1;f2;f3;n1;n2;n3],[f1, f2, f3, n1, n2, n3], [1, 2, 3, 0, 0, 0]);
torque = J_numeric.'*F % Answer

function TransformMat = DH_mat(link)
t = link.dh(1); 
d = link.dh(2);
a = link.dh(3);
alpha = link.dh(4);
TransformMat = [cos(t)  ,  -sin(t)*cos(alpha)  ,  sin(t)*sin(alpha)  ,  a*cos(t);
                sin(t)  ,   cos(t)*cos(alpha)  , -cos(t)*sin(alpha)  ,  a*sin(t);
                   0    ,         sin(alpha)    ,       cos(alpha)   ,      d       ;
                   0    ,             0          ,            0      ,      1       ;];
end