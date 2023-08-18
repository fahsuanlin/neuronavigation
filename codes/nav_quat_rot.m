function [M,euler]=nav_quat_rot(q,varargin)
%
%   nav_quat_rot    determine a rotation matrix and euler angles by
%   quarternions
%
%q:[q0, qx, qy, qz]'; input quaternion
%
% M: 3x3 rotation matrix
% euler: 3x1 Euler angle (roll, pitch, yaw)
%
% fhlin@May 22, 2023
%

try
    M=zeros(3,3); %rotation matrix
    M(1,1)=q(1)*q(1)+q(2)*q(2)-q(3)*q(3)-q(4)*q(4);
    M(1,2)=2*(q(2)*q(3)-q(1)*q(4));
    M(1,3)=2*(q(2)*q(4)+q(1)*q(3));
    M(2,1)=2*(q(2)*q(3)+q(1)*q(4));
    M(2,2)=q(1)*q(1)-q(2)*q(2)+q(3)*q(3)-q(4)*q(4);
    M(2,3)=2*(q(3)*q(4)-q(1)*q(2));
    M(3,1)=2*(q(2)*q(4)-q(1)*q(3));
    M(3,2)=2*(q(3)*q(4)+q(1)*q(2));
    M(3,3)=q(1)*q(1)-q(2)*q(2)-q(3)*q(3)+q(4)*q(4);

    %determine Euler angles
    euler=zeros(3,1);
    euler(1)=atan2(M(2,1),M(1,1)); %roll
    euler(2)=atan2(-M(3,1), cos(euler(1))*M(1,1)+sin(euler(1))*M(2,1)); %pitch
    euler(3)=atan2(sin(euler(1))*M(1,3)-cos(euler(1))*M(2,3), -sin(euler(1))*M(1,2)+cos(euler(1))*M(2,2)); %yaw


catch
    M=[];
    euler=[];

end;

