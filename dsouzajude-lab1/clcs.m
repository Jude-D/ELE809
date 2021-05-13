Tf = 3.0;
Fs = 150.0;
Ts = 1.0/Fs;
Kp = 1.5;

t  = 0:Ts:Tf;
Ns = length(t);

theta = [];
radfac = 180.0/pi;
ref = 50.0/radfac*ones(Ns,1);

servo = Initialize(Fs, 1, Tf); 

for k = 1,
    motor_acc = ReadEncoder(servo);
    
    motor_rad = EtoR(motor_acc);
    
    Va = Kp*ref(k);
    
    DtoA(servo, VtoD(Va));
    
    theta(k) = motor_rad;
    
end;

for k = 2: Ns,
    motor_acc = ReadEncoder(servo);
    
    motor_rad = EtoR(motor_acc);
    
    Va = Kp*ref(k)-theta(k-1);
    
    DtoA(servo, VtoD(Va));
    
    theta(k) = motor_rad;
    
end;

Terminate(servo);

plot(t,radfac*ref, t, radfac*theta);