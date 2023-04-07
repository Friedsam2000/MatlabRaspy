addpath(pwd);
addpath("CAD");
% smimport Assembly.xml
% save as .mdl

[robot,importInfo] = importrobot('robotic_arm.mdl');

