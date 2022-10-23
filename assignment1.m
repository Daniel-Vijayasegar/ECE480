%% Lab assignment #1
%Daniel Vijayasegar
%ECE 480
clear;
clc;
close all;


%% PROBLEM 1
clear;
clc;
close all;

x=[-500:0.1:500];
y1=20-(((x/20)-5)/5)-(((x/20)-5).^2)/25;
y2=4+(((15-x)/20).^2)/25;
y=sqrt(y1./y2);

figure('Name','Problem 1B','NumberTitle','off');
plot(x,y);
xlabel('x');
ylabel('Amplitude');
grid on;
axis([-500,600,-0.5,3]);
[ymax,ind]=max(y); %finding the index and value of the max
x(ind);  %x value of the max
%The range of x that makes y real is [-400 to 500]
%The global maximum is ~2.2443 and it occurs at x=20.8



%% Problem 2

clear;
clc;
close all;

t=[0:0.001:20];
y1=6*exp(-t/2);
y2=12*exp(-t/4);
y3=y2.*sin(6*t-3*pi/4)+y1;
y=[y1,y2,y3];

figure('Name','Problem 2','NumberTitle','off');
hold on
axis([0,20,-10,20]);
plot(t,y1,'Linewidth',2,'Color','k'); grid on
plot(t,y2,'Linewidth',2,'Color','m');
plot(t,y3,'Linewidth',2,'Color','#964B00');
xlabel('Time in sec');
ylabel('Amplitude');
legend('y1','y2','y3');
hold off;

[max_y3,ind_max_y3]=max(y3)
y3(ind_max_y3)
[min_y3,ind_min_y3]=min(y3)
y3(ind_min_y3)
y3_at3=y3(3)
y3_at5=y3(5)
y2(t==9)



%% Problem 3

clear;
clc;
close all;

dt=0.01;
t=[-1:dt:50];
y1=64*t.^2;
y2=exp(-0.34662*t);
y3=sin(0.3*pi*t-pi/3);
y4=heaviside(t);
x_t=y1.*y2.*y3.*y4;

%filling area array with the partial sums of the reimann slices
area=[];
for idx1=1:length(x_t)
    area(idx1)=sum(abs(x_t(1:idx1)).^2*dt);
end
%filling area array with the running total reimann sum area

idx2=1
while (area(idx2)<=area(length(area))*0.6)
    idx2=idx2+1;
end


energy=area(length(area))
sixty_percent_energy_time=t(idx2)

figure('Name','Problem 3','NumberTitle','off');
axis([-10,40,-100,600]);
plot(t,x_t);
xlabel('Time in sec');
ylabel('x(t)');
grid on;

[max_xt,ind_max_xt]=max(x_t)
x_t(ind_max_xt)
max_time=t(ind_max_xt)




%% Problem 4
clear;
clc;
close all;

syms x
n=20;
T0 = taylor(log10(1-x), x, 'Order', n);
true_val1 = -23.025*log10(1.4)
true_val2 = log10(1.4)
fplot(T0);

subs(T0,x,-0.4)

accuracy=1;
n=1  %order
while accuracy>0.001 && n<20
syms x;
n=n+1;
T1 = taylor(log10(1-x), x, 'Order', n);
fprintf("Order: ")
n
subs(T1,x,-0.4)
accuracy=abs(vpa((true_val2-subs(T1,x,-0.4)))/true_val2)*100
end




%% Problem 5A
clear;
clc;
close all;
n=[-6:10]';
x1=zeros(size(n))
x2=zeros(size(n))

x1(n==4)=1
x2(n==-4)=1

w=conv(x1,x2)
n1=1:length(w)

figure('Name','Problem 5A','NumberTitle','off');
stem(n1,w)
xlabel('Time in sec');
ylabel('x(t)');
title('Part 5A')



%% Problem 5B
clear;
clc;
close all;

n=[300:350]';
x2=zeros(size(n))
x2(n==315)=4

figure('Name','Problem 5B','NumberTitle','off');
stem(n,x2)
title('Part 5B')
xlabel('Time in sec');
ylabel('x2(t)');


%% Problem 5C
clear;
clc;
close all;

n=[-12:8]';
x3=zeros(size(n))
x3(n==6)=7.3
x3(n==-5)=4.8
figure('Name','Problem 5C','NumberTitle','off');
stem(n,x3)
title('Part 5C')
xlabel('Time in sec');
ylabel('x3(t)');



%% Problem 5D
clear;
clc;
close all;

n=[-10:20]';
x4=zeros(size(n))
x4(n==-5)=1
figure('Name','Problem 5D','NumberTitle','off');
stem(n,x4)
title('Part 5D')
xlabel('Time in sec');
ylabel('x4(t)');


%% Problem 5E
clear;
clc;
close all;

n=[-10:14]';
x1=zeros(size(n))
x2=zeros(size(n))

x1(n==0)=1
x1(n==1)=1
x1(n==2)=1

x2(n==0)=1
x2(n==1)=1
x2(n==2)=1
x2(n==3)=1
x2(n==4)=1
x2(n==5)=1
x2(n==6)=1

x5=conv(x1,x2)
n2=1:length(x5)
figure('Name','Problem 5E','NumberTitle','off');
stem(n2,x5)
title('Part 5E')
xlabel('Time in sec');
ylabel('x5(t)');




%% Problem 6A
clear;
clc;
close all;

n=[0:95];
x1=2*(cos(pi.*n/9+pi/3)).^3;
figure('Name','Problem 6A','NumberTitle','off');
stem(n,x1)
title('Part A')
xlabel('n')
ylabel('amplitude')

N=13;
sum(abs(x1(15:33)).^2)



%% Problem 6B
clear;
clc;
close all;

n=[-20:130];
x2=sin(pi.*n/15-pi/5);
figure('Name','Problem 6B','NumberTitle','off');
stem(n,x2)
xlabel('n')
ylabel('amplitude')
title('Part B')


N=30;
sum(abs(x2(11:41)).^2)



%% Problem 6C
clear;
clc;
close all;


n=[0:12];
x3=4*sin(3*pi.*n-pi/2);
figure('Name','Problem 6C','NumberTitle','off');
stem(n,x3)
xlabel('n')
ylabel('amplitude')
title('Part C')

N=2;
sum(abs(x3(3:5)).^2)



%% Problem 6D
clear;
clc;
close all;

n=[0:175];
x4=cos(pi.*n/sqrt(15));
figure('Name','Problem 6D','NumberTitle','off');
stem(n,x4)
xlabel('n')
ylabel('amplitude')
title('Part D')
grid on;




%% Problem 6E
clear;
clc;
close all;

n=[0:135];
x5=sin(pi.*n/4)+3*cos(pi.*n/3-pi/3);
figure('Name','Problem 6E','NumberTitle','off');
stem(n,x5)
xlabel('n')
ylabel('amplitude')
title('Part E')

N=24
sum(abs(x5(25:49)).^2)




%% Problem 6F
clear;
clc;
close all;

n=[0:80];
x6=4.*sin(pi.*n/5+pi/4).*cos(pi.*n/3);
figure('Name','Problem 6F','NumberTitle','off');

stem(n,x6)
xlabel('n')
ylabel('amplitude')
title('Part F')

N=15
sum(abs(x6(12:29)).^2)



%% Problem 6G
clear;
clc;
close all;

n=[-200:200];
x7=2.*sinc(pi.*n/15);
figure('Name','Problem 6G','NumberTitle','off');
stem(n,x7)
xlabel('n')
ylabel('amplitude')
title('Part G')

sum(abs(x7(1:401)).^2)




%% Problem 7
clear;
clc;
close all;

fsig=0.25*1000;
fsamp=16000;
T=10*0.001;
t=0:1/fsamp:T;
theta=deg2rad(45);
amp=5;
x=amp*(sin((2*pi*fsig*t)+theta));
figure('Name','Problem 7','NumberTitle','off');
subplot(211)
plot(t/(0.001),x);
title('Continuous')
xlabel('t');
ylabel('x(t)');
grid on;

Ts=1/fsamp;
n=0:1:T/Ts;
x=amp*(sin((2*pi*fsig*n/fsamp)+theta));
subplot(212);
stem(n,x);
title('Discrete')
xlabel('n');
ylabel('x(n)');
grid on;



%% Problem 8
clear;
clc;
close all;

n=-40:1:40; 
a=0.49;
x=5*a.^(abs(n)/2).*(1-rem(abs(n),2));

figure('Name','Problem 8A','NumberTitle','off');
subplot(211);
stem(n,x);
ylabel('amplitude');
xlabel('time');
title('x(n)');
energy_x = sum(abs(x).^2)
%energy of x=40.7981

y=n.*x;
subplot(212);
stem(n,y);
ylabel('amplitude');
xlabel('time');
title('y(n)');

[y_min,n_min]=min(y);
[y_max,n_max]=max(y);

energy_y=sum(abs(y).^2)
%energy of y(n)=211.95



%% Problem 9
clear;
clc;
close all;

n=[0:20]';
x = 75*(-0.8).^n
x(n<3)=0
figure('Name','Problem 9','NumberTitle','off');
stem(n,x)
title('Part A x(n)')
xlabel('n')
ylabel('x(n)')

sum(x)
syms n
x2 = 75*(-0.8).^n;
symsum(75*(-0.8).^n,n,3,Inf)
energy = sum(abs(x).^2)



%% Problem 10
clear;
clc;
close all;

k1=400; k2=-2;
alpha1=57.5364; alpha2=21.0721;
f1=200;
Tsamp=0.005;
syms t
x_at = k1*t*exp(-alpha1*t)*heaviside(-t) + k2*exp(alpha2*t)*sin(2*pi*f1*t + pi/4)*heaviside(t)
figure('Name','Problem 10','NumberTitle','off');
fplot(x_at)
title('Part A x_a(t)')
xlabel('t')
ylabel('x_a(t)')


