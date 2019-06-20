%% initialize input

stepCount = length(PRV0);
senValues = [PRV0, PRV1, PRV2, PRV3, PRV4, PRV5, PRV6, PRV7];

%% constants

circle = 0:pi/50:2*pi;
robotX = robotDiameter*0.5*cos(circle);
robotY = robotDiameter*0.5*sin(circle);
robotDirX = [10, 10+robotDiameter*0.5];
robotDirY = [15, 15];

lengthO = length(object1X);
drawObject1 = lengthO > 1;
lengthO = length(object2X);
drawObject2 = lengthO > 1;

senCircleStart = 202.5*pi/180.0;
senCircleStep = pi/4.0;
senCircleR = 0:pi/50:pi/3;
senCircle = [-1*fliplr(senCircleR), senCircleR];

distsO = 0;
distsD = 0;
angleO = 0;
angleD = 0;
sideMeasD = 0;

%% plot diagrams

for idx = 1:stepCount
    figure(idx)
    
    % draw robot
    plot(robotX+xPos(idx), robotY+yPos(idx), 'k-'); hold on;
    plot(robotLineX(idx,:), robotLineY(idx,:), 'k-'); hold on;
    
    % draw reference
    plot(robotX+xPosRef(idx), robotY+yPosRef(idx), 'k-'); hold on;
    plot(robotLineXRef(idx,:), robotLineYRef(idx,:), 'k-'); hold on;
    
    % draw objects
    plot(object1X, object1Y, 'k-'); hold on;
    objX = objectDiameter*0.5*cos(circle)+object2X;
    objY = objectDiameter*0.5*sin(circle)+object2Y;
    plot(objX, objY, 'k-'); hold on;
    
    %draw sensor values
    for senIdx=0:7
        distsP = invSensorModel(senValues(idx, senIdx+1), senCircleR).*100;
        dists = [fliplr(distsP), distsP];
        linePoints = [robotDiameter*0.5, robotDiameter*0.5+invSensorModel(senValues(idx, senIdx+1), 0).*100];
        circlePoint = 2*pi-senCircleStart-senCircleStep*senIdx;
        plot(linePoints.*cos(circlePoint)+xPos(idx), linePoints.*sin(circlePoint)+yPos(idx), 'r-'); hold on;
        edgeX = robotDiameter*0.5*cos(circlePoint)+dists.*cos(senCircle+circlePoint) + xPos(idx);
        edgeY = robotDiameter*0.5*sin(circlePoint)+dists.*sin(senCircle+circlePoint) + yPos(idx);
        plot(edgeX, edgeY, 'r-', 'LineWidth', 2); hold on;
    end

    % calculate direction vector
    lDist = invSensorModel(senValues(idx, 4), 0)*100;
    rDist = invSensorModel(senValues(idx, 5), 0)*100;
    %xRef = (lDist*cos(pi/8.0) + rDist*cos(-pi/8.0))/2.0;
    %yRef = (lDist*sin(pi/8.0) + rDist*sin(-pi/8.0))/2.0;
    distRef = distAppr(lDist, rDist); %sqrt(xRef^2+yRef^2);
    angleRef = angleAppr(lDist, rDist); %-atan(yRef/xRef);
    plot([xPos(idx)+robotDiameter/2.0, xPos(idx)+robotDiameter/2.0+distRef*cos(angleRef)], [yPos(idx), yPos(idx)+distRef*sin(angleRef)], 'g-', 'LineWidth', 2);
    %[xPos(idx)+0.5*robotDiameter, xPos(idx)+0.5*robotDiameter+distRef*cos(angleRef)]
    hold off;
    grid on;
    
    axis(diaRange);
    axis square;
    
    xlabel('x-Koordinate [cm]');
    ylabel('y-Koordinate [cm]');
    title(['World ', world, ', step ', int2str(idx), ' [dist=', num2str(distRef,3), ' cm, angle=', num2str(angleRef,3), ' rad]']);
    
    saveas(idx, ['./diagrams/', world, '_', int2str(idx), '.png']);

    switch idx
        case {1,15,16,17}
            distsO = distsO + distRef;
            angleO = angleO + angleRef;
        case {6,7,8,9,10,11}
            distsD = distsD + distRef;
            angleD = angleD + angleRef;
            sideMeasD = sideMeasD + invSensorModel(senValues(idx, 3), 0)*100;
        otherwise
            % do nothing
    end
end

close all;

distsOri(worldPos) = distsO/4.0;
distsDiff(worldPos) = distsD/6.0;
angleOri(worldPos) = angleO;
angleDiff(worldPos) = angleD/6.0;
sideMeas(worldPos) = sideMeasD/6.0;