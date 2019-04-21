function [T1, T2, T3, T4] = finger_transforms(q, L)

% These are better DH parameters than the paper uses
theta1 = q/2;
theta2 = 0;
theta3 = 0; 
theta4 = q/2;
a1 = 0;
a2 = L*sin(q/2)/q;
a3 = L*sin(q/2)/q;
a4 = 0;

T1 = dhp(theta1, 0, a1, 0);
T2 = dhp(theta2, 0, a2, 0);
T3 = dhp(theta3, 0, a3, 0);
T4 = dhp(theta4, 0, a4, 0);

end


