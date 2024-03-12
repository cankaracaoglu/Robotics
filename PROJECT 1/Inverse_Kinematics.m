
function [sol1,sol2,sol3,sol4] = Inverse_Kinematics(Px,Py,Pz)
L1 = 140;
L2 = 140;

%% Information keeper arrays
theta1s = [0 0 0 0];
theta2s = [0 0 0 0];
theta3s = [0 0 0 0];

%% theta1
%There are two theta one values which depends on x and z values in of the
%end effector. The seconde one is direct opposite of the first one with
%pi radians rotaiton.

theta1 = - atan2(Px,Pz + L1);

%remainder operator is used with the 2*pi to keep the rotation value
%between 0 and 2pi.
theta1_2 = rem((pi + theta1),2*pi);

%Put the values into array
theta1s(1) = theta1;
theta1s(2) = theta1;
theta1s(3) = theta1_2;
theta1s(4) = theta1_2;

r1 = sqrt(Px^2 + (Pz+L1)^2);

r2 = sqrt(r1^2 + (Py-L2)^2);    


%% theta2

alpha = atan2(Py-L1,r1);
cos_beta = (r2^2 + L1^2 - L2^2) / (2 * r2 * L1);
beta_1 = atan2(sqrt(1-cos_beta^2),cos_beta);
beta_2 = atan2(-sqrt(1-cos_beta^2),cos_beta);
theta2_1 = alpha + beta_1;
theta2_2 = alpha + beta_2;

theta2s(1) = theta2_1;
theta2s(2) = theta2_2;
theta2s(3) = pi - theta2_1 ;
theta2s(4) = pi - theta2_2 ;

%% theta3
cos_psi = ((L1^2 + L2^2) - r2^2) / (2 * L1 * L2);
theta3s(1) = theta2_1 - pi/2 + atan2(sqrt(1-cos_psi^2),cos_psi);
theta3s(2) = mod(theta2_2 - pi/2 + atan2(-sqrt(1-cos_psi^2),cos_psi),2*pi);
theta3s(3) = mod(theta2s(3) - pi/2 + atan2(-sqrt(1-cos_psi^2),cos_psi),2*pi);
theta3s(4) = theta2s(4) - pi/2 + atan2(sqrt(1-cos_psi^2),cos_psi);

sol1 = transpose([theta1s(1) theta2s(1) theta3s(1)]);
sol2 = transpose([theta1s(2) theta2s(2) theta3s(2) ]);
sol3 = transpose([theta1s(3) theta2s(3) theta3s(3) ]);
sol4 = transpose([theta1s(4) theta2s(4) theta3s(4) ]);
end