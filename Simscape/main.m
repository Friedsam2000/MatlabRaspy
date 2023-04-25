path = mfilename('fullpath');
path = erase(path,'\main');
addpath(path);

path = [path,'\CAD'];
addpath(path);

clear path

% smimport Assembly.xml
% save as .mdl

[robot,importInfo] = importrobot('robotic_arm.mdl');