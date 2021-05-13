function [angle_load] = EtoR(motor_position)
%EtoR:  converts the accumulated value of the motor shaft position into angular position of the load shaft in radians. 

angle_motor = deg2rad(motor_position/2048 * 360); %solves for the angle of the motor in terms of radians
angle_load = angle_motor * 1 / 17.2; %from excersize B.1, the equaion of N1/N2 is 1/17.2


end
