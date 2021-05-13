function [DA_value] = VtoD(voltage)
%VtoD converts a voltage value into the equivalent D/A repersentation
Vmin = -3; %min voltage
Vmax = 3; %max voltage
Size = 16; %size of the D/A converter

Q = (abs(Vmin) + abs(Vmax)) / 2 ^ Size; %Value of the resolution
if (voltage <= 3 && voltage >= -3) %checking to make sure the voltage is within the allowed range
    
DA_value = voltage / Q; %prints the value of the D/A converter

else
    
fprintf("error, range is outside bounds \n"); %if outside the range

end

