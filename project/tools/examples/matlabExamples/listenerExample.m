%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de> 
% Functionity: Creates an rsb listener which listens on the '/' scope and
%              displays a SickLdMRS400102 array in a plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; clear java

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rstsandbox-0.11.2_SICK.jar'])
javaaddpath([PWD '/rsb-matlab-0.11.0_SICK.jar'])

% Converter registration
rsb.matlab.ConverterRegistration.register('rst.claas.SickLdMRS400102Type', 'SickLdMRS400102')

% Create the listener
factory = rsb.Factory.getInstance();
listener = factory.createListener('/');

% Create the queue with an 100 element buffer
scanData = rsb.matlab.SickLdMRS400102Queue(100,true);
listener.activate()
listener.addHandler(scanData, true)

while(1)
    % Blocks 1 seconds if no data arives
    data = scanData.take(int32(1000));
    datetime
    if (~isempty(data))
        pointsList = data.getMeasuredPointsList;
        pointsArray = pointsList.toArray;
        plot(cell2mat(cell(pointsArray)))
        getframe;
    end
end

listener.deactivate();
