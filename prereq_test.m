%ECE480 Pretest script
%Daniel Vijayasegar Spring 22

%PROBLEM 1
clc;
clear;
close all;

a = -2 + 2i;
b = 2 - 2i;
c = 2 - 3i;
d = -5 - 4i;
e = 2 - 1i;
f = 0.20788;  %real value of i^i
g = ((d^2)*(e^2)*(f))/((a^3)*(b)*(c^4));
[theta, rho] = cart2pol(real(g), imag(g));
rho
phase=rad2deg(theta)
real(g)
imag(g)


%% Problem 3A
clc;
clear;
close all;
syms x 
f = 1/(1+0.5*x.^2)^1.5
T_A = taylor(f,x,6)


%% Problem 3B
clc;
clear;
close all;
syms XX
YY=cosh(XX)
YY_TAY=taylor(YY,'order',12)
YY_TAY_1=subs(YY_TAY,XX,XX^-1)


%% Problem 3C
clc;
clear;
close all;
syms x
T3 = taylor(2.302585*log(7.389056-3.694528*x))

%% Problem 3D
syms x
T4 = taylor(20.0855^x)


%% Problem 4A
clc;
clear;
close all;
syms n;
S1 = symsum(-0.9^(2*n), n,0,inf)


%% Problem 4B
clc;
clear;
close all;
syms n;
S2 = symsum((0.85*n), n,-4,5)

%% Problem 4C
clc;
clear;
close all;
syms n;
S3 = symsum((0.5*(-n)), n,0,6)

%% Problem 4D
clc;
clear;
close all;
syms n;
S4 = symsum((n*(n-2)), n,0,8)


%% Problem 6

clc;
clear;
close all;

syms x y;
EQ1 = y-50*atan(x/2);
EQ2 = y-(x-4)^2;
ss = solve(EQ1, EQ2)
x_num = double(ss.x)
y_num = double(ss.y)



%% Problem 7A
clc;
clear;
close all;
syms t
x_at=-5+(cos(40*pi*4))^3+2*cos(60*pi*t-pi)*sin(140*pi*t)
figure('Name','Problem 7A','NumberTitle','off');
xlabel('Time');
ylabel('Amplitude');
ezplot(x_at)


%% Problem 7D
clc;
clear;
close all;
b = [1 -200*pi];
a = [1 200*pi];

syms t
x_at=-5+(cos(40*pi*4))^3+2*cos(60*pi*t-pi)*sin(140*pi*t);
figure('Name','Problem 7D Mag & Phase Spectra','NumberTitle','off');

freqs(b,a)

%% Testing
clc;
clear;
close all;
[x1,y1] = pol2cart(deg2rad(95.73),1.25e4)

[x2,y2] = pol2cart(deg2rad(-95.73),1.25e4)

x1+x2
y1+y2





