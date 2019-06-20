%%
clear all
close all

%% constants
alpha = 0.942693757414292;
beta = -16.252241893638708;
delta = 0;
xi = 1.236518540376969;
variance = 20.886268537074187;

robotDiameter = 10;
distToRef = 5;
distToEdge = 10;
drivingStep = 5;
drivingCount = 18;

xPos = 0.5*robotDiameter+distToEdge:drivingStep:drivingCount*drivingStep+0.5*robotDiameter+distToEdge;
yPos = ones(1, 19) .* xPos(round(0.5*length(xPos)));
robotLineX = [xPos', xPos'+robotDiameter*0.5];
robotLineY = [yPos', yPos'];

xPosRef = xPos+robotDiameter+distToRef;
yPosRef = yPos;
robotLineXRef = [xPosRef', xPosRef'+robotDiameter*0.5];
robotLineYRef = [yPosRef', yPosRef'];

diaRange = [0, 2*distToEdge+robotDiameter+drivingCount*drivingStep];
diaRange = [diaRange, diaRange];

objectStart = 25 + xPos(1);
objectLength = 40;
objectWidth = 10;
objectDiameter = 6.7;

distsDiff = zeros(6,1);
angleDiff = zeros(6,1);
sideMeas = zeros(6,1);
worldPos = 0;

%% functions
invSensorModel = @(sensorValue, angle) sqrt(alpha.*cos(xi.*angle) ./ (sensorValue-beta) + delta.*cos(xi.*angle));
distError = @(dist, angle) (dist.^2./cos(xi.*angle) - delta).^2 ./ (2.*dist.*alpha.*sqrt(1./cos(xi.*angle))) .* sqrt(variance);

minAngle = @(angle1, angle2) min(abs(angle1-angle2), 2*pi-abs(angle1-angle2));

distAppr = @(lDist, rDist) sqrt((lDist*cos(pi/8.0) + rDist*cos(-pi/8.0))^2 + (lDist*sin(pi/8.0) + rDist*sin(-pi/8.0))^2) - 1.7*robotDiameter;
angleAppr = @(lDist, rDist) -atan((lDist*sin(pi/8.0) + rDist*sin(-pi/8.0))/(lDist*cos(pi/8.0) + rDist*cos(-pi/8.0)));
%distAppr = @(lDist, rDist) sqrt(((lDist+robotDiameter/2)*cos(pi/8.0) + (rDist+robotDiameter/2)*cos(-pi/8.0))^2 + ((lDist+robotDiameter/2)*sin(pi/8.0) + (rDist+robotDiameter/2)*sin(-pi/8.0))^2) - 1.7*robotDiameter;
%angleAppr = @(lDist, rDist) -atan(((lDist+robotDiameter/2)*sin(pi/8.0) + (rDist+robotDiameter/2)*sin(-pi/8.0))/((lDist+robotDiameter/2)*cos(pi/8.0) + (rDist+robotDiameter/2)*cos(-pi/8.0)));

%% World without object

worldPos = 1;
world = 'followAlone';
object1X = -objectDiameter;
object1Y = -objectDiameter;
object2X = -objectDiameter;
object2Y = -objectDiameter;

dataName = 'follow_alone.txt';
importFollowData(dataName);
calcDiagrams

oriDist = distsDiff(1);
oriAngle = angleDiff(1);

%% Worlds with objects

object2X = 0;
object2Y = 0;
for worldPos=1:6
    world = ['follow', int2str(worldPos*10), 'mm'];
    object1X = [objectStart, objectStart, objectStart+objectLength, objectStart+objectLength];
    object1Y = [yPos(1)+robotDiameter*0.5+worldPos+objectWidth, yPos(1)+robotDiameter*0.5+worldPos, yPos(1)+robotDiameter*0.5+worldPos, yPos(1)+robotDiameter*0.5+worldPos+objectWidth];
    object2X = xPos(1)+82.5;
    object2Y = yPos(1)+15;
    
    dataName = ['follow_wall', int2str(worldPos), '0.txt'];
    importFollowData(dataName);
    calcDiagrams
end

%% calculate differences

distsDiff = flipud(oriDist - distsDiff);

if oriAngle < 0
    oriAngle = 2*pi + oriAngle;
end
for idx=1:6
    if angleDiff(idx) < 0
        angleDiff(idx) = 2*pi + angleDiff(idx);
    end
end
angleDiff = flipud(minAngle(angleDiff, oriAngle));

%% plot differences
x = (1:1:6)';

figure(1)
md = sum((x-mean(x)).*(distsDiff-mean(distsDiff)))/sum((x-mean(x)).^2);
yd = mean(distsDiff) - md*mean(x);
plot(flipud(distsDiff), 'b-'); hold on;
plot(flipud(x.*md+yd), 'r-'); hold off;
xlabel('Distanz zum Objekt [cm]');
ylabel('Abweichung [cm]');
title(['Distanz-Ausgleichsgerade: f(x) = ', num2str(md,5), '*x + ', num2str(yd,5)]);
saveas(1, 'diagrams/AusgleichDist.png');

figure(2)
ma = sum((x-mean(x)).*(angleDiff-mean(angleDiff)))/sum((x-mean(x)).^2);
ya = mean(angleDiff) - ma*mean(x);
plot(flipud(angleDiff), 'b-'); hold on;
plot(flipud(x.*ma+ya), 'r-'); hold off;
xlabel('Distanz zum Objekt [cm]');
ylabel('Abweichung [rad]');
title(['Winkel-Ausgleichsgerade: f(x) = ', num2str(ma,5), '*x + ', num2str(ya,5)]);
saveas(2, 'diagrams/AusgleichAngle.png');

figure(3)
mm = sum((x-mean(x)).*(sideMeas-mean(sideMeas)))/sum((x-mean(x)).^2);
ym = mean(sideMeas) - mm*mean(x);
plot(sideMeas, 'b-'); hold on;
plot(x.*mm+ym, 'g-'); hold off;
%axis([1,6,0,6]);
xlabel('Distanz zum Objekt [cm]');
ylabel('Gemessene Distanz [cm]');
title(['Fehler der seitliche Distanzmessung: f(x) = ', num2str(mm,5), '*x + ', num2str(ym,5)]);
saveas(3, 'diagrams/AusgleichSeitenMessung.png');



