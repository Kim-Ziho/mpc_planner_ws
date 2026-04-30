function SetSearchPaths

%{
try
    
    addpath([pwd,'']);
    addpath([pwd,'/ElementAndWorld']);
    addpath([pwd,'/Particle']);
    
    addpath([pwd,'/Tools']);
    addpath([pwd,'/Tools/GenericTools']);
    addpath([pwd,'/Tools/ProcessingTools']);
    addpath([pwd,'/Tools/InterfaceTools']);
    
    addpath([pwd,'/Utilities']);
    addpath([pwd,'/Utilities/GazeboTools']);
    
    addpath([pwd,'/Control']);
    addpath([pwd,'/Control/ObstacleAndVGraph']);
    addpath([pwd,'/Control/Coverage']);
    
catch
    disp('ERROR: Local directories not found!');
end
%}

try
    addpath('/storage/shared/users/subhrabh/GazeboAPI-0.2.4/src');
    addpath('/storage/shared/users/subhrabh/GazeboAPI-0.2.4/lib');
    addpath('/opt/matlab/playerAPI/lib');
    addpath('GazeboTools/');
catch
    disp('Warning: Local installation of GazeboAPI not found. Relying on other installation.');
end
