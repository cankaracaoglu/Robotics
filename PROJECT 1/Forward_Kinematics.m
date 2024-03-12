%% Function definition

function [Px,Py, Pz] = Forward_Kinematics(th1,th2,th3)
%Define a symbol theta for rotation definitions

syms theta

%Classic one axis rotation definitions
Xrot = [1 0 0;
    0 cos(theta) -sin(theta); 
    0 sin(theta) cos(theta)];
Yrot = [cos(theta) 0 sin(theta);
    0 1 0; 
    -sin(theta) 0 cos(theta)];
Zrot = [cos(theta) -sin(theta) 0; 
    sin(theta) cos(theta) 0;
    0 0 1];

%Revoloute joint and length parameters of the robot 
syms theta1 theta2 theta3
L1 = 140;
L2 = 140;

%Transformation matrix from frame zero to frame 1
%Substitute rotation angles into theta values in standart one axis rotation
%Matrix completed with transition and classic 4th row.
zero_to_one = [subs(Xrot,theta,-pi/2)*subs(Zrot,theta,-pi/2)*subs(Zrot,theta,-theta1) [0;L2;-L1]; 0 0 0 1];

rotation_one_to_two = subs(Xrot,theta,pi/2) * subs(Zrot,theta,theta2);

%Transformation from 1 to 2
one_to_two = [rotation_one_to_two [0;0;0]; 0 0 0 1];

%Transformation from 2 to 3
two_to_three = [subs(Zrot,theta,(theta3 -theta2- pi/2)) [L1;0;0]; 0 0 0 1];

%Transformation from 3 to tip
three_to_tip = [eye(3,3) [L2;0;0]; 0 0 0 1];

%Forward kinematics transformation (Matrix multipication)
FK = zero_to_one * one_to_two * two_to_three * three_to_tip;


%Resulting matrix
FK_val = double(subs(FK,{theta1,theta2,theta3},{th1,th2,th3}));

Px = FK_val(1,4);
Py = FK_val(2,4);
Pz = FK_val(3,4);
return 

end
