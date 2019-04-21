function [T] = dhp(theta, d, a, alpha)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Td = [eye(3), [0; 0; d]; [0, 0, 0, 1]];
Ta = [eye(3), [a; 0; 0]; [0, 0, 0, 1]];
Ttheta = [cos(theta), -sin(theta), 0, 0;
          sin(theta), cos(theta), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
Talpha = [1, 0, 0, 0;
          0, cos(alpha), -sin(alpha), 0;
          0, sin(alpha), cos(alpha), 0;
          0, 0, 0, 1];

T = Td*Ttheta*Ta*Talpha;

end

