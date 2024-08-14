function circleEquation = findCircleEquation(startPos, endPos, tmin, tinc, tmax)
% Input Validation
if ~isnumeric(startPos) || ~isnumeric(endPos) || numel(startPos) ~= 2 || numel(endPos) ~= 2
    error('Inputs must be numeric arrays of length 2 (x,y).');
end

ID = 0.45;
OD = 1.5;

x1 = startPos(1); y1 = startPos(2);
x2 = endPos(1); y2 = endPos(2);

% Calculate Midpoint of the chord (Not necessarily the center)
midpoint = [(x1 + x2)/2, (y1 + y2)/2];

% Direction vector of the chord
chordVector = [x2 - x1, y2 - y1];

% Perpendicular Bisector's Direction Vector (Normal to the chord)
perpBisectorVector = [-chordVector(2), chordVector(1)];

theta = linspace(0, 2*pi, 100);
semiCircle = linspace (-pi/2,pi/2,50);

% Calculate and Plot Circles for t = 1:30
for t = tmin:tinc:tmax
    fprintf('-------t = %.2f-------\n', t);
    % Plotting Setup
    figure;
    hold on;
    axis equal; % Ensure circles appear circular (not elliptical)

    % Parameterize the Center along the Perpendicular Bisector
    center = midpoint + t * perpBisectorVector;

    % Calculate Radius
    radius = norm(center - startPos);

    % Calculate midjoint at start position based on 1.3 mm radius loop
    midjoint = [endPos(1)-1.3,endPos(2)];
    midjointRadius = norm(midjoint-center);
    circleEquation = sprintf('Equation: (x - %.2f)^2 + (y - %.2f)^2 = %.2f', center(1), center(2), radius^2);
    disp(circleEquation);
    
    % Generate Points for Plotting Circle
    
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    mx = center(1) + midjointRadius * cos(theta);
    my = center(2) + midjointRadius * sin(theta);

    % Translate Start and End points so that center is the origin
    startPosTrans = [startPos(1)-center(1),startPos(2)-center(2)];
    endPosTrans = [endPos(1)-center(1),endPos(2)-center(2)];
    midjointTrans = [midjoint(1)-center(1),midjoint(2)-center(2)];
    
    % Solve for angle of rotation between start and end points about origin
    syms thetaSym
    eqn = startPosTrans(1) == endPosTrans(1)*cos(thetaSym) - endPosTrans(2)*sin(thetaSym);
    rotation = vpasolve(eqn,thetaSym);

    % Plot midjoint at end point based on solved rotation
    midjointPrimeX = midjointTrans(1)*cos(rotation)-midjointTrans(2)*sin(rotation);
    midjointPrimeY = midjointTrans(1)*sin(rotation)+midjointTrans(2)*cos(rotation);
    midjointPrime = [midjointPrimeX+center(1),midjointPrimeY+center(2)];

    % Calculate angle between links at midjoint
    midjointAngleRad = max(min(dot(midjointPrime,startPosTrans)/(norm(midjointPrime)*norm(startPosTrans)),1),-1);
    midjointAngle = real(acosd(midjointAngleRad));

    % Plot hinge 0.45ID 1.5OD
    x_hingeJoint = center(1) + ID/2 * cos(theta);
    y_hingeJoint = center(2) + ID/2 * sin(theta);
    plot(x_hingeJoint, y_hingeJoint, 'k', 'LineWidth', 1);
    
    x_hinge = center(1) + OD/2 * cos(semiCircle);
    y_hinge = center(2) + OD/2 * sin(semiCircle);
    plot(x_hinge, y_hinge, 'k', 'LineWidth', 1);

    % Top Surface of Hinge
    plot([x_hinge(1),x_hinge(1)-5],[y_hinge(50),y_hinge(50)], 'k', 'LineWidth', 1);
    % Bottom Surface of Hinge
    plot([x_hinge(1),x_hinge(1)-5],[y_hinge(1),y_hinge(1)], 'k', 'LineWidth', 1);

    % Find point of intersection of arm and hinge at starting position
    [xInt, yInt] = lineIntersection(x_hinge(1),y_hinge(50),x_hinge(1)-5,y_hinge(50),center(1),center(2),midjoint(1), midjoint(2));
    plot(xInt, yInt, 'bo', 'MarkerSize', 8);

    % Plot End Effector Range of Motion
    plot(x, y); 
    % Plot midjoint Range of Motion
    plot(mx, my); 
    % Plot first link at start position (RED)
    plot([center(1), midjoint(1)], [center(2), midjoint(2)], 'r-', 'LineWidth', 2); 
    % Plot second link at start position (RED)
    plot([midjoint(1), endPos(1)], [midjoint(2), endPos(2)], 'r-', 'LineWidth', 2); 
    % Plot first link at end position (BLUE)
    plot([center(1), midjointPrime(1)], [center(2), midjointPrime(2)], 'b-', 'LineWidth', 2); 
    % Plot second link at end position (BLUE)
    plot([midjointPrime(1), startPos(1)], [midjointPrime(2), startPos(2)], 'b-', 'LineWidth', 2);
    
    title(['t = ', num2str(t)]);
    xlabel('x'); ylabel('y');
    grid on;
    hold off;
    fprintf('Center Position: (%.4f,%.2f) mm\n',center(1),center(2));
    fprintf('Arm Start Angle: %.4f deg\n', 180-midjointAngle);
    fprintf('Arm End Angle: %.4f deg\n', 180-midjointAngle-rad2deg(rotation));
    fprintf('Midjoint Angle: %.4f deg\n', midjointAngle);
    fprintf('Range of Motion: %.4f deg\n', rad2deg(rotation));
    fprintf('End Effector Radius: %.4f mm\n', radius);
    fprintf('Arm Length: %.4f mm\n', norm(center-midjoint));
    fprintf('Distance to top surface: %.4f mm\n', norm([xInt,yInt]-center));

end

hold off;
end