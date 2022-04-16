clc; clear; close all;
%% Question 3
% a)Derive the 6 x 3 Jacobian matrix associated with the relationship between joint
% displacements and the position and orientation of the tool at point A.
% The DH Convention Table
%-------------------------------------------------------------------------%
%Joint   |   Joint Angle   |   Link Offset   |   Link Length  | Link Twist%
%  1     |       t1        |        400      |        0       |   -pi/2   %        
%  2     |       t2        |         0       |        0       |    pi/2   %
%  3     |       t3        |        100      |        0       |     0     %
%  4     |        0        |        50       |       100      |     0     %
%-------------------------------------------------------------------------%
syms t1 t2 t3 
link(1).dh = [t1 400 0 -pi/2]; % ".dh" creates a matrix containing the DH details for a certain link
link(2).dh = [t2  0  0  pi/2];
link(3).dh = [t3 100 0 0];
link(4).dh = [0 50 100 0]; % A new link between the frame 3 and the new frame located at the grinding tool
for i = 1:4
    link(i).mat = DH_mat(link(i));
end
% Jacobian
T0_2 = link(1).mat*link(2).mat;
T0_3 = T0_2*link(3).mat;
T0_4 = T0_3*link(4).mat;

% b and r
b0 = [0 ; 0 ; 1];
b1 = link(1).mat(1:3,3);
b2 = T0_2(1:3,3);

r2_end = 100*T0_3(1:3,1);
r1_end = 0*T0_2(1:3,1) + r2_end ;
r0_end = 400*b0 + r1_end;
% Substitute all the vectors into the Jacobian formula
% Prismatic : [b(i-1) ; 0], Revolute : [b(i-1)xr(i-1)_end ; b(i-1)]
% Joint 1: Prismatic, Joint 2: Prismatic, Joint 3: Prismatic
J = simplify([cross(b0,r0_end),cross(b1,r1_end),cross(b2,r2_end);b0,b1,b2]); % Answer
%% b) During the grinding operation, reaction forces and moments act on the tool tip A.
%Represent the force and moments by a 6 x 1 vector F, derive the equivalent joint
%torques.
% To obtain the numeric magnitudes, we need to obtain some configurations
a = atan(100/150);
c = sqrt(100^2+150^2); 
b = (5*pi/18) - a; % (5*pi/18), assumed angle based on the figure
r = c*sin(b); 
h = c*cos(b);
h_total = 400 + h;

% Assumptions to be made: f(t)=15 and f(N)=15, and the resultant force of
% The resultant force of f(t) and f(N) is in the opposite direction of the z-axis of the frame 3.
% Hence, f_direction = [0, 0, -1] was also made.
ft = 15; fN = 15; Nx3 = 0.04; % moment and forces pre-defined
ft_direction = -link(1).mat(1:3,3); fN_direction = [0;0;-1];
R0_3 = T0_3(1:3,1:3); % Rotation Matrix of the frame 3
P0_3 = T0_3(1:3,4); % Position at the frame 3
fN = [(ft*ft_direction)+(fN*fN_direction); Nx3*R0_3(:,1)]; %R0_3(:,1) : first column because the moment is along x-axis of the frame 3

% Computation of the initial joint angles by using the optimisation method
x0 = [0 0 0]; % Random values for the optimisation of the nonliear equations of P0_4
initial_joint_angles = double(fsolve(@optim,x0,optimoptions('fsolve','Display','iter')));

R0_4 = simplify(T0_4(1:3,1:3));
P0_4 = simplify(T0_4(1:3,4));
T3_4 = simplify(link(3).mat*link(4).mat);
P3_4 = T3_4(1:3,4);

% Force computation
Ft = double(subs(fN,[t1,t2,t3],initial_joint_angles)); % The force and moments on the end-effector
% Force Transformation
FM_mat = double(subs(R0_3*P3_4,[t1,t2,t3],initial_joint_angles));
sym_mat = symmetric_mat(FM_mat); %Skew-symmetric matrix
F_sym = [eye(3,3),zeros(3,3);
sym_mat,eye(3,3)]*Ft; %force/moment on the tool
joint_torques = vpa((subs(J.'*F_sym,[t1,t2,t3],initial_joint_angles)/1000),3)
%% c) Write a program to move the tool counter-clockwise round a circle on the work
% Position and Velocity of the end tool before constructing simulation
% constant and the manipulator rotates about z0 axis only.
syms t % t for simulation (each angle varies with respect to t)
% X : position of the frame 3 with time (t)
X = 100*[cos(t*(pi/180))*sin(t*(pi/180)) ; sin(t*(pi/180))*sin(t*(pi/180)) ; cos(t*(pi/180)) + 4];
P_real = simplify(P0_3);
% Construct equations for the position of the frame 3 to obtain the
% answers of "t" 
equation = [X(1)==P_real(1) X(2)==P_real(2) X(3)==P_real(3)];
answer = solve(equation,[t1, t2, t3]);

% from the answer, we obtained the following joint angles with respect to time. 
% t1_new will be used for plotting 
t1_new = 2*atan(tan((pi*t)/360)); %From the answer obtained above

%% Plot 1: The path of the tool tip on the work surface (A, B, C and D);
j = 1;
Position = subs(P0_4,[t1,t2,t3],[initial_joint_angles(1)+t1_new,initial_joint_angles(2),initial_joint_angles(3)]);
for i = 1:1:60
    Position_x(j) = double(subs(Position(1),t,i-1));
    Position_y(j) = double(subs(Position(2),t,i-1));
    j = j+1;
end
figure(1); plot(Position_x,Position_y,'LineWidth',2)
xlabel("Displacement in x-axis"); ylabel("Displacement in y-axis")
title("The path of the tool tip on the work surface")
hold on; plot(0,max(Position_y),'r*'); hold on; plot(min(Position_x),0,'k*');
hold on; plot(0,min(Position_y),'b*'); hold on; plot(max(Position_x),0,'g*');
legend("Path of the tool tip","A","B","C","D"); grid on
hold off
%% Plot 2: The respective joint angles versus time
% An assumption: theta 2 and theta 3 remain rigid while in motion
j=1;
for i = 1:1:60
    theta1(j) = double(initial_joint_angles(1)+2*atan(tan((pi*i)/360)));
    theta2(j) = double(initial_joint_angles(2));
    theta3(j) = double(initial_joint_angles(3));
    j = j+1;
end
figure(2); plot(1:length(theta1),theta1,'LineWidth',3)
hold on; plot(1:length(theta2),theta2,'LineWidth',3)
hold on; plot(1:length(theta3),theta3,'LineWidth',3)
title("Joint angles versus Time"); xlabel("Time"); ylabel("Joint angle")
legend("Theta1","Theta2","Theta3"); grid on; hold off
%%  Plot 3 (1 of 2): Calculate the velocity
J_new = simplify(subs(J,[t1,t2,t3],[initial_joint_angles(1)+t1_new,initial_joint_angles(2),initial_joint_angles(3)]));
J_inv = pinv(J_new); % Pseudo-inverse so that we can calculate dq=J^-1*dp
P0_3_change = subs(P0_3,[t1, t2,t3],[initial_joint_angles(1)+t1_new, initial_joint_angles(2),initial_joint_angles(3)]);
Velocity = diff(P0_3_change,t); % Velocity at the frame 3
q = simplify(J_inv*[Velocity;0;0;1]); % Joint rate
%% Plot 3 (2 of 2): Plot the respective joint rates versus time
joint_rate = subs(q,t1,t1_new);
j = 1;
for i = 1:1:60
    j_rate1(j) = double(simplify(subs(joint_rate(1),t,i-1)));
    j_rate2(j) = 0;
    j_rate3(j) = 0;
    j = j+1;
end
figure(3); plot(1:length(j_rate1),j_rate1,'LineWidth',3)
hold on; plot(1:length(j_rate2),j_rate2,'LineWidth',3)
hold on; plot(1:length(j_rate3),j_rate3,'c','LineWidth',3)
title("Joint rates versus Time"); xlabel("Time"); ylabel("Joint rate")
legend("joint rate1","joint rate2","joint rate3"); grid on; hold off
xlim([0 60]); ylim([-0.5 1.5])
hold off
%% Plot 4: The respective joint torques versus time
f0_3 = F_sym;
R3_0 = transpose(R0_3);
f3_3 = [R3_0 zeros(3,3) ; zeros(3,3) R3_0]*f0_3;
f3_3 = eval(subs(f3_3,[t1, t2,t3],[initial_joint_angles(1)+t1_new, initial_joint_angles(2),initial_joint_angles(3)]));
torque_tooltip = transpose(J_new)*f3_3;
j = 1;
for i = 1:1:60
    torque1(j) = double(simplify(subs(torque_tooltip(1),t,i-1)))/1000;
    torque2(j) = double(subs(torque_tooltip(2),t,i-1))/1000;
    torque3(j) = double(subs(torque_tooltip(3),t,i-1))/1000;
    j = j+1;
end
figure(4); plot(1:length(torque1),torque1,'LineWidth',3)
hold on; plot(1:length(torque2),torque2,'LineWidth',3)
hold on; plot(1:length(torque3),torque3,'LineWidth',3)
title("Moments versus Time"); xlabel("Time"); ylabel("Moment")
legend("torque1","torque2","torque3"); grid on; hold off
%% 3D-plot of PUMA 600
PUMA600(-26.89,-37.58,19.11,0)
%% Functios used to solve the question
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
function PUMA600(t1,t2,t3,t4)
%input: joint angles at each frames
%ouput: end-effector position
format compact
format short
%DH parameters
t = [t1 t2 t3 t4]; %joint angles
alpha =  [-90 90 0 0] ; %twist angle
a = [0 0 0 100]; %offset as to xn
d = [400 0 100 50]; %offset as to z(n-1)
%forward kinematics
if t(1,1) >= -160 && t(1,1) <= 160 && t(1,2)>= -225 ...
        && t(1,2) <= 45 && t(1,3) >= -45 && t(1,3) <= 225 ...
        && t(1,4) >= -110 && t(1,4) <= 170
    T = [];
    %Homogeneus Transformation
    for n = 1:4
        MatTransform = [cosd(t(n)), -sind(t(n))*cosd(alpha(n)), sind(t(n))*sind(alpha(n)), a(n)*cosd(t(n));
            sind(t(n)), cosd(t(n))*cosd(alpha(n)), -cosd(t(n))*sind(alpha(n)), a(n)*sind(t(n));
            0, sind(alpha(n)), cosd(alpha(n)), d(n);
            0 0 0 1];
        T = [T; {MatTransform}];
    end
    P = [];
    %Joint Positions
    for i = 1:4
        if i == 1
            P = [P,{T{i}}];
        else 
            matP = P{i-1}*T{i};
            P = [P, {matP}];
        end
    end
    %plotting the joint positions
    x = [0 P{1}(1,4) P{1}(1,4) P{2}(1,4) P{3}(1,4) P{4}(1,4)];
    y = [0 P{1}(2,4) P{2}(2,4) P{2}(2,4) P{3}(2,4) P{4}(2,4)];
    z = [0 P{1}(3,4) P{1}(3,4) P{2}(3,4) P{3}(3,4) P{4}(3,4)];
    hold on
    grid on
    rotate3d on
    axis([-300,300,-300,300,0,1000])
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
    plot3(x,y,z,'o','Color','k')
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    title('Forward Kinematics of PUMA 600 Manipulator')
    view(0,0)
    disp('The end-effector position is ')
    fprintf('at Px = %f, Py = %f, and Pz = %f'...
        ,P{4}(1,4),P{4}(2,4),P{4}(3,4));
    disp('  ')
    disp('The DH model is  ')
    disp(P{4})
else
    disp('Joint angle out of range');
end
end