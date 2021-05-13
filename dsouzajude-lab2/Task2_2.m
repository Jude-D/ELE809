Tf = 10.0;
Fs = 150.0;
Ts = 1.0/Fs;
Kp = 15.383;
Ki = 332.98;
Kd = 0.20302;
Beta = 0.0055;

t  = 0:Ts:Tf;
r = 50*pi/180*square(2*pi*0.25*t);
Ns = length(t);

num = [4.225 -3.226];
den = [1 -1.765 0.765];
Gp = tf(num, den);
Gc = pid(Kp, Ki, Kd);
H = [1];
M = feedback(Gc*Gp, H);

theta = [];
radfac = 180.0/pi;

servo = Initialize(Fs, 1, Tf); 

for k = 1: Ns,
    motor_acc = ReadEncoder(servo);
    
    motor_rad = EtoR(motor_acc);
    
    Va = M*r(k);
    
    DtoA(servo, VtoD(Va));
    
    theta(k) = motor_rad;
    
end;

Terminate(servo);

plot(t,radfac*r, t, radfac*theta);