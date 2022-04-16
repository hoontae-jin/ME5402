clc; clear; close all;
%% Question 1 - a) 
% This section determines the link parameters and derive the kinematic equation of the robot.
% The DH Parameters
%-------------------------------------------------------------------------%
%Joint   |   Joint Angle   |   Link Offset   |   Link Length  | Link Twist%
%  1     |       t1        |        d0       |        0       |    +90    %        
%  2     |       t2        |    -d(offset)   |        l1      |     0     %
%  3     |       t3        |         0       |        0       |    +90    %
%  4     |       t4        |         d2      |        0       |    -90    %
%  5     |       t5        |         0       |        0       |    +90    %
%  6     |       t6        |         d3      |        0       |     0     %
%-------------------------------------------------------------------------%
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1 % t(n): angle theta
% The matrices were hand-calculated based on the DH parameters
A0_1 = [cos(t1) 0 sin(t1) 0; sin(t1) 0 -cos(t1) 0; 0 1 0 d0; 0 0 0 1];
A1_2 = [cos(t2) -sin(t2) 0 l1*cos(t2); sin(t2) cos(t2) 0 l1*sin(t2); 0 0 1 d_off; 0 0 0 1];
A2_3 = [cos(t3) 0 sin(t3) 0; sin(t3) 0 -cos(t3) 0; 0 1 0 0; 0 0 0 1];
A3_4 = [cos(t4) 0 -sin(t4) 0; sin(t4) 0 cos(t4) 0; 0 -1 0 d2; 0 0 0 1];
A4_5 = [cos(t5) 0 sin(t5) 0; sin(t5) 0 -cos(t5) 0; 0 1 0 0; 0 0 0 1];
A5_6 = [cos(t6) -sin(t6) 0 0; sin(t6) cos(t6) 0 0; 0 0 1 d3; 0 0 0 1];

% Answer %
T0_6 = simplify(A0_1*A1_2*A2_3*A3_4*A4_5*A5_6);

n = T0_6(1:3,1); % nx, ny, nz 
a = T0_6(1:3,2); % ax, ay ,az
o = T0_6(1:3,3); % ox, oy, oz
p = T0_6(1:3,4); % px, py, pz

%% Q-b1) Write a MATLAB function to compute the inverse kinematics of the PUMA 600.
% This section computes the angles by using the inverse kinematic
% transformation
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
% Create the numeric matrix of T0_6 by inputting the given values
d0 = 500; d_off = -200; l1 = 500; d2 = 400; d3 = 150; 
t1 = pi/6; t2 = pi/6 ;t3 = pi/6; t4 = pi/6; t5 = pi/6; t6 = pi/6;
T0_6_numeric = eval(T0_6); % Solvability for T0_6
% First approach of the inverse kinematic : Moving each A(n-1,n)
% matrix to the LHS (next to T0_6) one by one
% e.g. T1_6 = A(0_1)^-1*T(0_6) = A(1_2)*A(2_3)*A(3_4)*A(4_5)*A(5_6)

% T(n_m)_inv = An inverse kinematic matrix with the actual distances (LHS)
% T(n_m) = An inverse kinematic matrix only with symbols (RHS)
T1_6_inv = simplify(inv(A0_1)*T0_6_numeric); % LHS
T1_6 = simplify(A1_2*A2_3*A3_4*A4_5*A5_6); %RHS

% Based on the matrix obtained above, t1 angle can be obtain by looking at
% (3,3) and (3,4) 
calc_t1 = solve(T1_6_inv(3,4)==T1_6(3,4)); % theta 1
real_t1 = eval(calc_t1);
joint_rot1 = real(double(real_t1(2))); % The one with a positive sign was chosen

% Likewise, For the calculation of t2
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
T2_6_inv = simplify(subs((inv(A0_1*A1_2)*T0_6_numeric),t1,joint_rot1)); 
T2_6 = simplify(A2_3*A3_4*A4_5*A5_6);

calc_t2 = solve(T2_6_inv(2,3)==T2_6(2,3));
t3 = pi/6; t4 = pi/6; t5 = pi/6;
real_t2 = eval(calc_t2);
joint_rot2 = real(double(real_t2(1)));

% Same procedure as above to calculate t3,4,5 and 6
% t3
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
T3_6_inv = simplify(subs((inv(A0_1*A1_2*A2_3)*T0_6_numeric),[t1, t2],[joint_rot1, joint_rot2])); 
T3_6 = simplify(A3_4*A4_5*A5_6);

calc_t3 = solve(T3_6_inv(1,1)==T3_6(1,1));
t4 = pi/6; t5 = pi/6; t6 = pi/6;
real_t3 = eval(calc_t3);
joint_rot3 = real(double(real_t2(1)));

% t4
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
T5_6_inv = simplify(subs(inv(A0_1*A1_2*A2_3*A3_4*A4_5)*T0_6_numeric,[t1, t2, t3],[joint_rot1, joint_rot2, joint_rot3])); 
T5_6 = simplify(A5_6);
calc_t4 = solve(T5_6_inv(2,4)==T5_6(2,4));
d0 = 500; d_off = -200; l1 = 500;
real_t4 = eval(calc_t4);
joint_rot4 = real(double(real_t4(1)));

% t5
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
T5_6_inv = simplify(subs(inv(A0_1*A1_2*A2_3*A3_4*A4_5)*T0_6_numeric,[t1, t2, t3, t4],[joint_rot1, joint_rot2, joint_rot3, joint_rot4])); 
T5_6 = simplify(A5_6);
calc_t5 = solve(T5_6_inv(1,1)==T5_6(1,1));
t6 = pi/6;
real_t5 = eval(calc_t5);
joint_rot5 = real(double(real_t5(1)));

% t6
syms t1 t2 t3 t4 t5 t6 d0 d_off d2 d3 l1
T3_6_new = subs(T3_6,[t2, t3, t4, t5], [joint_rot2, joint_rot3, joint_rot4, joint_rot5]);
T3_6_inv_new = simplify(subs((inv(A0_1*A1_2*A2_3)*T0_6_numeric),[t1, t2, t3],[joint_rot1, joint_rot2, joint_rot3]));
calc_t6 = solve(T3_6_inv_new(1,1)==T3_6_new(1,1));
joint_rot6 = double(calc_t6(2));
% The angle results from the inverse kinematic transformation

% Answer %
fprintf('\n the joint variables are:\n %s \n %s \n %s \n %s \n %s \n %s \n',...
    num2str(joint_rot1),num2str(joint_rot2)...
    ,num2str(joint_rot3) ,num2str(joint_rot4) ,num2str(joint_rot5) ,num2str(joint_rot6))
%% Q-b2) Plot the manipulator
% The DH Parameters
%-------------------------------------------------------------------------%
%Joint   |   Joint Angle   |   Link Offset   |   Link Length  | Link Twist%
%  1     |       t1        |        500      |        0       |    +90    %        
%  2     |       t2        |       -200      |       500      |     0     %
%  3     |       t3        |         0       |        0       |    +90    %
%  4     |       t4        |        400      |        0       |    -90    %
%  5     |       t5        |         0       |        0       |    +90    %
%  6     |       t6        |        150      |        0       |     0     %
%-------------------------------------------------------------------------%
PUMA600(30,30,30,30,30,30) %the plot function is located at the bottom

%% Q-c) Solutions for the inverse kinematics (IN PROGRESS)
%d0 = 500; d_off = -200; l1 = 500; d2 = 400; d3 = 150;
%t1 = pi/6; t2 = pi/6 ;t3 = pi/6; t4 = pi/6; t5 = pi/6; t6 = pi/6;
% According to the figure given, the wrist point is at the fourth frame.
% Hence, we compute T0_4 matrix by forward kinmatic transformation as below
T0_4 = simplify(A0_1*A1_2*A2_3*A3_4);
T0_4_numeric = eval(T0_4);
T0_3 = simplify(A0_1*A1_2*A2_3);
T0_2 = simplify(A0_1*A1_2);
% In order to find out the position of the wrist, we use the (4,1),(4,2) and (4,3) in the matrix.
p0_4a = T0_4(1:3,4); % the position of the wrist from T0_4
d3 = 150; %mm
p0_4b = T0_6(1:3,4) - d3*T0_6(1:3,3); % pw = p - d3a

R3_6 = simplify(transpose(T0_3(1:3,1:3))*T0_6(1:3,1:3));

% Number of solutions for t1
t1_eq = sin(t1)*575.0000 - cos(t1)*562.9165 == -200;
t1_sol = eval(solve(t1_eq,t1));
% Number of solutions for t2
X = cos(t1_sol(2))*575.0000 + cos(t1_sol(2))*562.9165;
Y = 500 - 550;
t2_eq = (400^2 - 500^2 - X^2 - Y^2)/((2*Y*500)-(2*X*500)) == cos(t2) + sin(t2);
t2_sol = eval(solve(t2_eq,t2))
% Number of solutions for t2
%LHS (R3_0*R0_6)
syms nx ny nz ax ay az ox oy oz
R0_6 = [nx ax ox; ny ay oy; nz az oz];
LHS_3_6 = transpose(T0_3(1:3,1:3))*R0_6
%% Function for plotting
function PUMA600(t1,t2,t3,t4,t5,t6)
%input: joint angles at each frames
%ouput: end-effector position
format compact
format short
%DH parameters
t = [t1 t2 t3 t4 t5 t6]; %joint angles
alpha =  [90 0 90 -90 90 0] ; %twist angle
a = [0 500 0 0 0 0]; %offset as to xn
d = [500 -200 0 400 0 150]; %offset as to z(n-1)
%forward kinematics
if t(1,1) >= -160 && t(1,1) <= 160 && t(1,2)>= -225 ...
        && t(1,2) <= 45 && t(1,3) >= -45 && t(1,3) <= 225 ...
        && t(1,4) >= -110 && t(1,4) <= 170 && t(1,5) >= -100 ...
        && t(1,5) <= 100 && t(1,6) >= -266 && t(1,6) <= 266
    T = [];
    %Homogeneus Transformation
    for n = 1:6
        MatTransform = [cosd(t(n)), -sind(t(n))*cosd(alpha(n)), sind(t(n))*sind(alpha(n)), a(n)*cosd(t(n));
            sind(t(n)), cosd(t(n))*cosd(alpha(n)), -cosd(t(n))*sind(alpha(n)), a(n)*sind(t(n));
            0, sind(alpha(n)), cosd(alpha(n)), d(n);
            0 0 0 1];
        T = [T; {MatTransform}];
    end
    P = [];
    %Joint Positions
    for i = 1:6
        if i == 1
            P = [P,{T{i}}];
        else 
            matP = P{i-1}*T{i};
            P = [P, {matP}];
        end
    end
    %plotting the joint positions
    x = [0 P{1}(1,4) P{1}(1,4) P{2}(1,4) P{3}(1,4) P{4}(1,4) P{5}(1,4) P{6}(1,4)];
    y = [0 P{1}(2,4) P{2}(2,4) P{2}(2,4) P{3}(2,4) P{4}(2,4) P{5}(2,4) P{6}(2,4)];
    z = [0 P{1}(3,4) P{1}(3,4) P{2}(3,4) P{3}(3,4) P{4}(3,4) P{5}(3,4) P{6}(3,4)];
    hold on
    grid on
    rotate3d on
    axis([0,1000,0,1000,0,1000])
    plot3([x(1) x(2)],[y(1) y(2)],[z(1) z(2)]...
        ,'Color','b','LineWidth',10)
    plot3([x(2) x(3)],[y(2) y(3)],[z(2) z(3)]...
        ,'Color','r','LineWidth',10)
    plot3([x(3) x(4)],[y(3) y(4)],[z(3) z(4)]...
        ,'Color','g','LineWidth',10)
    plot3([x(4) x(5)],[y(4) y(5)],[z(4) z(5)]...
        ,'Color','c','LineWidth',10)
    plot3([x(5) x(6)],[y(5) y(6)],[z(5) z(6)]...
        ,'Color','m','LineWidth',10)
    plot3([x(6) x(7)],[y(6) y(7)],[z(6) z(7)]...
        ,'Color','y','LineWidth',10)
    plot3([x(7) x(8)],[y(7) y(8)],[z(7) z(8)]...
        ,'Color','k','LineWidth',10)
    plot3(x,y,z,'o','Color','k')
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    title('Forward Kinematics of PUMA 600 Manipulator')
    view(0,0)
    disp('The end-effector position is ')
    fprintf('at Px = %f, Py = %f, and Pz = %f'...
        ,P{6}(1,4),P{6}(2,4),P{6}(3,4));
    disp('  ')
    disp('The DH model is  ')
    disp(P{6})
else
    disp('Joint angle out of range');
end
end
