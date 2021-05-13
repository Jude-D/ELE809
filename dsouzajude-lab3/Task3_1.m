%% Part A ELE 809 Lab 3

%Defining Variables
n = 1/17.2;
Jm = 0.25 * 10 ^-6; 
Jl = 2 * 10 ^ -5;
Jeq = Jm + n^2 *(Jl);
Bm = 1.23 * 10 ^ -7;
Bl = 0; %negliable
Beq = Bm + n^2 *(Bl);
K = 6.09 * 10 ^ -3;
L = 0.15 * 10 ^ -3;
R = 3;
Wn = 35.6;
Tau = 0.5169;
T = 1/150;

%Defining Matrixs
A = [0 1 0; 0 -Beq/Jeq K/Jeq; 0 -K/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];
D = [0];

%create the state system
sysc = ss(A, B, C, D);
sysd = c2d(sysc, 1/150);

%Solve for the values
[phi,gamma] = ssdata(sysd);
poles = eig(phi);

%Check for Stability in the System
stability = eig(phi);

%check for controllability 
Wc = [gamma phi*gamma phi^2*gamma];
controllability = det(Wc);

%check for observability
Wo = [C; C*phi; C*phi^2];
observability = det(Wo); 

%% Part B ELE 809 Lab 3

po = 15/100;
ts1 = 0.25;
z = -log(po)/sqrt(pi^2 + log(po)^2);
wn = 4.6/z/ts1;
sdc = bessp(3)*wn;
%sd = -26.5398+25.3187j;
%sd2 = -33.5352;
%sd = [sd2 sd conj(sd)];
zdc = exp(sd*T);
phiCLd = poly(zd);

K = [0 0 1]*inv(Wc)*polyvalm(phiCLd, phi);

M = [phi-eye(3), gamma; C, D];
R = [0; 0; 0; 1];
N = inv(M)*R;

Nx = N(1:3);
Nu = N(4);
Nbar = Nu + K*Nx;

sysCL = ss(phi-gamma*K, gamma*Nbar, C, 0, T);


%% Part C ELE 809 Lab 3

phiA = [1 -C; zeros(3,1) phi];
gammaA = [0 gamma']';
CA = [0 C];
WcA = ctrb(phiA, gammaA);
rank(WcA);
sdc = [wn*bessp(3)];
se = -4*(z)*wn;
ze = exp(se*T);
zdc = [exp(sdc*T) ze];
pdc = poly(zdc);
KA = [0 0 0 1]*inv(WcA)*polyvalm(pdc, phiA);
K = KA(2:4);
Ki = KA(1);

%3 posiblities for Nbar
Nbar1 = Nu+K*Nx;

Nbar3 = Ki/(ze-1);


%% Task 3.1

Tf = 3.0;
Fs = 150.0;
Ts = 1.0/Fs;
Kp = 1.5;

t  = 0:Ts:Tf;
Ns = length(t);

%Defining Matrixs
A = [0 1 0; 0 -Beq/Jeq K/Jeq; 0 -K/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];
D = [0];

% Initial conditions
x0 = [0; 0; 0];
xd0 = [0; 0; 0];

% Predefine the size of the state/ouput vectors
x = zeros(3, Ns);
xd = zeros(3, Ns);
y = zeros(size(t));

theta = [];
radfac = 180.0/pi;
ref = 50.0/radfac*ones(Ns,1);

servo = Initialize(Fs, 1, Tf); 

for k = 1: Ns,
    motor_acc = ReadEncoder(servo);
 
    motor_rad = EtoR(motor_acc);
    
    Va = Kp*ref(k);
    
    DtoA(servo, VtoD(Va));
    
    theta(k) = motor_rad;
   
if k == 1
        x(:, k) = x0;
        xd(:,k) = A*x0 + B*motor_rad(k);
        y(k) = C*x0 + D*motor_rad(k);
else
    x(:,k) = x(:,k-1) + Ts*xd(:,k-1);       % x = int(xd*dt)
    xd(:,k) = A*x(:,k) + B*motor_rad;        % State equation
    y(k) = C*x(:,k) + D*theta(k);           % Output equation
end
    
end

Terminate(servo);

plot(t,radfac*ref, t, radfac*y);











