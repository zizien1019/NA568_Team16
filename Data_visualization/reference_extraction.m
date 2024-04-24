clc, clear, close all

% This script was written by Mohammed Buhlaigah in Mobile Robotics (EECS568),
% Winter 2024 semester.

% This script extracts reference values from an image
reference_file = 'ref1';

% import the csv file
T = readtable(strcat(reference_file,'.csv'));

% convert csv file to a readable matrix
M = table2array(T);

% extract variables

x = M(:,19);         % camera linear position [cm]
y = M(:,20);         % camera linear position [cm]
z = M(:,21);         % camera linear position [cm]
phi = M(:,22);       % camera angle [deg]
theta = M(:,23);     % camera angle [deg]
psi = M(:,24);       % camera angle [deg]
r = sqrt(x.^2+y.^2+z.^2);   % Euclidian distance [cm]

ref_r = mean(r)
ref_phi = mean(phi)
ref_theta = mean(theta)
ref_psi = mean(psi)



