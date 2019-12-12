%ENPM 667 - Control of robotic systems - Project 1
%Submitted by Vishnu Sashank Dorbala, UID 116907569 and Kulbir Singh Ahluwalia, UID 116836050
%This is a program used to simulate the Lyapunov Controller used for the Autonomous Doorway Passing task.
%
%clc, to clear the command window, clear, to clear the workspace
clc, clear      
sympref('FloatingPointOutput',true) 
%to declare symbolic variables
syms xd h zd m v omega l2 w2 w 

%Initialising variables
l2 = 1.4; % distance of camera from robot frame in length in m
w2 = -3.5; % distance of camera from robot frame in width in m
v = 0.1; % constant velocity (translational) in m/s
m = 0.3; % Margin for doorway passing in metres
h = 0.5; % height of the camera from the ground in metres
k = 1; % gain of the Lyapunov controller
t = 0; % Time variable to keep track of number of loops
theta2 = -0.8; % Initial angle that the camera makes with the doorpost in radians
r = 0.5; % Initializing the r value
padding = zeros([1, 10]); % Padding zeros for appending arrays after loop

% Initializing arrays to store graphs.
valarr = {};
zdarr = {};
xdarr = {};
phidarr = {};
phidesarr = {};
rarray = {};
diffarr = {};
omegarr = {};

% Simulation parameters. As we do not have an estimate of zd and xd, we start with an initial value for each, and decrease steadily in steps towards 0, the target point.
zd = -1.4;
stepx = 0.01;
stepz = 0.01;
xd = -1.5;            

% Running a while loop till r=m, when the switching motion begins to take place.
% After this, the motion remains only angular.
while r > m
%Changing the iterables representing desired coordinates xd and zd in steps. 
% The time t keeps track of the number of loops.
xd = xd + stepx;
zd = zd + stepz;
t = t+1;

% Determining the distance to the doorpost r
r = sqrt(xd^2 + zd^2);

% The current phi_d feature extracted for servoing.
phid = atan2(xd,zd);

% Defining the same r in different manner. Gives the exact same value.
r = zd/(cos(phid));

% Computing the desired trajectory angle that phid has to follow.
phid_des = theta2 + asin(m/r);

% The Lyapunov Function
V = 0.5*(phid - phid_des)^2;

% The A and B functions described in the paper.
Afunc = simplify(subs ( sin(phid - phid_des)/(r*cos(phid_des - theta2))));

Bfunc = simplify(subs ( l2*cos(phid - phid_des) - w2*sin(phid - phid_des))/r*cos(phid_des - theta2));

% Derivative of the Lyapunov Function
Vdot = (phid - phid_des)*(v*Afunc + w*(1+Bfunc));

% Angular velocity value omega
omega = simplify (subs ( ( (-k*(phid - phid_des) - Afunc*v)/(1 + Bfunc))));

% Storing all the values to be plotted in an array.
zdarr = [zdarr, double(zd)];
xdarr = [xdarr, double(xd)];
xdarr = [xdarr, padding];
phidarr = [phidarr, double(phid)];
phidesarr = [phidesarr, double(phid_des)];
valarr = [valarr, double(omega)];
rarray = [rarray , double(r)];
diffarr = [diffarr, double(phid_des-phid)];
omegarr = [omegarr, double(omega)];
end


% Padding the arrays with zeros
valarr = [valarr, padding];
zdarr = [zdarr, padding];
phidarr = [phidarr, phid*zeros([1, 10])];
phidesarr = [phidesarr, phid_des*zeros([1, 10])];
rarray = [rarray, padding];

% Creating a padded and non padded time array for plotting.
time = 0: t +length(padding)-1;
time_nopad = 0:t-1;

% Converting all the arrays into ones suitable for plotting.
valarr = [valarr{:}];
zdarr = [zdarr{:}];
xdarr = [xdarr{:}];
phidarr = [phidarr{:}];
phidesarr = [phidesarr{:}];
rarray = [rarray{:}];
diffarr = [diffarr{:}];
omegarr = [omegarr{:}];

% Plotting the graphs in subplots.
subplot(3,1,1)
plot(time,rarray)
title("R value")
legend R-value

subplot(3,1,2)
plot(time,valarr');
title("Omega");
legend \omega

subplot(3,1,3)
plot(time, phidarr', 'black');
hold on
plot(time, phidesarr', 'red');
title("\PhiD and \PhiD^* Value Comparison");
legend \PhiD \PhiD^*