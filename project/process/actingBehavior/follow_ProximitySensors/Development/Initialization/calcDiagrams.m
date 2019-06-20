%% start

clear all;
close all;

%% constants

robotDiameter = 0.1; % m;
guideDistFix = 0.02; % m;
guideAngleFix = 135.0 *pi/180.0; % rad

sensorOffset = 7.0/8.0 * pi; % rad
sensorDist = pi/4.0; % rad
sensorPos = zeros(8,3);
for sen=1:8
    sensorAngle = sensorOffset - (sen-1)*sensorDist;
    if (sensorAngle < 0)
        sensorAngle = 2*pi + sensorAngle;
    end
    sensorPos(sen,1) = robotDiameter*0.5 * cos(sensorAngle);
    sensorPos(sen,2) = robotDiameter*0.5 * sin(sensorAngle);
    sensorPos(sen,3) = sensorAngle;
end

alpha = 0.942693757414292;
beta = -16.252241893638708;
delta = 0;
xi = 1.236518540376969;
variance = 20.886268537074187;

robotTurn = 5.0 * pi/180.0; % rad
turnSteps = guideAngleFix/robotTurn;

%% functions

invSensorModel = @(sensorValue, angle) sqrt(alpha.*cos(xi.*angle) ./ (sensorValue-beta) + delta.*cos(xi.*angle));
distError = @(dist, angle) (dist.^2./cos(xi.*angle) - delta).^2 ./ (2.*dist.*alpha.*sqrt(1./cos(xi.*angle))) .* sqrt(variance);
minAngle = @(angle1, angle2) min(abs(angle1-angle2), 2*pi-abs(angle1-angle2));

%% calculation

for step = 0:turnSteps
    figure(step+1)
    
    % circle samples
    circlePoints = 0:pi/50.0:2.0*pi;
    
    % draw robot with direction line
    robotCircleX = robotDiameter*0.5 .* cos(circlePoints);
    robotCircleY = robotDiameter*0.5 .* sin(circlePoints);
    plot(robotCircleX, robotCircleY, 'k-'); hold on;
    plot([0, robotDiameter*0.5], zeros(2,1), 'k-'); hold on;
    
    % draw guide
    guideDir = guideAngleFix - step*robotTurn;
    guideX = (robotDiameter+guideDistFix) * cos(guideDir);
    guideY = (robotDiameter+guideDistFix) * sin(guideDir);
    robotCircleX = robotDiameter*0.5 .* cos(circlePoints) + guideX;
    robotCircleY = robotDiameter*0.5 .* sin(circlePoints) + guideY;
    plot(robotCircleX, robotCircleY, 'b-'); hold on;
    
    % diagram discription
    hold off;
    grid on;
    axisRange = 2*robotDiameter + guideDistFix;
    axis([-axisRange, axisRange, -axisRange, axisRange]);
    axis square;
    xlabel('x-Koordinate [m]');
    ylabel('y-Koordinate [m]');
    title(['Initialisierung (Schritt ', int2str(step+1), ')']);
    
    % save diagram
    saveas(step+1, ['./diagrams/step_', int2str(step), '.png']);
end

close all;