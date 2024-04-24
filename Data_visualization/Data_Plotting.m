clc, clear, close all

% This script was written by Mohammed Buhlaigah in Mobile Robotics (EECS568),
% Winter 2024 semester.

% This script plots data taken from the app as a CSV file.
% Instructions: Save your CSV file as: TrialName followed by trial number
% e.g: forward1, forward2 ...etc, where you move the phone ...
% forward and record the data 

% -------------------------------------------------------
experiment = 'forwardFinal';
init_trial = 1;    % number of trial to be plotted
end_trial = 5;     % number of final trial to be plotted
% variable to plot:
variable_name = 'r';

% For Forward/Backward: variable = r
% For Up/Down: variable = phi
% For Right/Left: variable = theta
% For Clockwise/Counterclockwise: variable = psi

% Note: If the needed variable is other than r, phi, theta, psi, x, y, z ...
% you need to change it manually below
% --------------------------------------------------------





% Number of trials to plot: 
num_trials = end_trial - init_trial + 1;

% Initialize Linear fit vectors
vel = zeros(num_trials,2);

% initialize the figure
figure(1)
tcl = tiledlayout(ceil((end_trial-init_trial+1)/2),2);

for i = init_trial:end_trial

    % import the csv file
    T = readtable(strcat(experiment,num2str(i),'.csv'));

    % convert csv file to a readable matrix
    M = table2array(T);

    % extract variables
    t = (M(:,1)-M(1,1))/1000;    % time [s]
    x_no_kal = M(:,2);         % camera linear position (no kalman filtering) [cm]
    y_no_kal = M(:,3);         % camera linear position (no kalman filtering) [cm]
    z_no_kal = M(:,4);         % camera linear position (no kalman filtering) [cm]
    phi_no_kal = M(:,5);       % camera angle (no kalman filtering) [deg]
    theta_no_kal = M(:,6);     % camera angle (no kalman filtering) [deg]
    psi_no_kal = M(:,7);       % camera angle (no kalman filtering) [deg]
    ax = M(:,8);        % IMU acceleration [m/s^2]
    ay = M(:,9);        % IMU acceleration [m/s^2]
    az = M(:,10);       % IMU acceleration [m/s^2]
    ang_vel_x = M(:,11);% gyro angular acceleration [rad/s]
    ang_vel_y = M(:,12);% gyro angular acceleration [rad/s]
    ang_vel_z = M(:,13);% gyro angular acceleration [rad/s]
    phi_imu = M(:,14);  % phone angle using IMU [deg]
    theta_imu = M(:,15);  % phone angle using IMU [deg]
    psi_imu = M(:,16);  % phone angle using IMU [deg]
    centroid_x = M(:,17);   % x location of object centroid on the phone screen [pixels] 
    centroid_y = M(:,18);   % y location of object centroid on the phone screen [pixels] 
    x_kal = M(:,19);    % x position with Kalman filter [cm]
    y_kal = M(:,20);    % y position with Kalman filter [cm]
    z_kal = M(:,21);    % z position with Kalman filter [cm]
    phi_kal = M(:,22);    % phi with Kalman filter [deg]
    theta_kal = M(:,23);    % theta with Kalman filter [deg]
    psi_kal = M(:,24);    % psi with Kalman filter [deg]

    r_no_kal = sqrt(x_no_kal.^2+y_no_kal.^2+z_no_kal.^2);   % Euclidian distance (no kalman filtering) [cm]
    r_kal = sqrt(x_kal.^2+y_kal.^2+z_kal.^2);   % Euclidian distance with kalman filter [cm]

    r = [r_no_kal, r_kal];
    phi = [phi_no_kal, phi_kal];
    theta = [theta_no_kal, theta_kal];
    psi = [psi_no_kal, psi_kal];
    x = [x_no_kal, x_kal];
    y = [y_no_kal, y_kal];
    z = [z_no_kal, z_kal];


    % Variable to plot:
    switch variable_name
        case 'r'
            variable = r;
            variable_unit = ': [cm]';
        case 'phi'
            variable = phi;
            variable_unit = ': [deg]';
        case 'theta'
            variable = theta;
            variable_unit = ': [deg]';
        case 'psi'
            variable = psi;
            variable_unit = ': [deg]';
        case 'x'
            variable = x;
            variable_unit = ': [cm]';
        case 'y'
            variable = y;
            variable_unit = ': [cm]';
        case 'z'
            variable = z;
            variable_unit = ': [cm]';
        otherwise
            disp('Please choose variable to plot')
            variable = 0*r;
            variable_unit = 'CHOOSE VARIABLE';
    end


    nexttile
    hold on
    plot(t,variable(:,1),t,variable(:,2))
    xlabel('time [s]')
    ylabel(strcat(variable_name,variable_unit))
    legend('Raw Data','With Kalman','Location','best')
    title(strcat('trial #',num2str(i)))
    

%     % Add Linear fit to the data
%     hold on
%     j = i-init_trial+1;
%     vel(j,:) = polyfit(t,variable(:,2),1);
%     plot(t,vel(j,1)*t + vel(j,2))
%     legend('Raw Data','With Kalman','Linear Fit')

end
title(tcl,experiment)


