function circleEquation = findCircleEquation(point1, point2)
% Input Validation
if ~isnumeric(point1) || ~isnumeric(point2) || numel(point1) ~= 2 || numel(point2) ~= 2
    error('Inputs must be numeric arrays of length 2 (x,y).');
end

x1 = point1(1); y1 = point1(2);
x2 = point2(1); y2 = point2(2);

% Calculate Midpoint of the chord (Not necessarily the center)
midpoint = [(x1 + x2)/2, (y1 + y2)/2];

% Direction vector of the chord
chordVector = [x2 - x1, y2 - y1];

% Perpendicular Bisector's Direction Vector (Normal to the chord)
perpBisectorVector = [-chordVector(2), chordVector(1)];

% Plotting Setup
figure;
hold on;
axis equal; % Ensure circles appear circular (not elliptical)

% Calculate and Plot Circles for t = 1:30
for t = 1.3
    % Parameterize the Center along the Perpendicular Bisector
    center = midpoint + t * perpBisectorVector;

    % Calculate Radius
    radius = norm(center - point1);

    
    midjoint = [point2(1)-1.3,point2(2)];
    midjointRadius = norm(midjoint-center);
    midjointEquation = sprintf('t = %d: (x - %.2f)^2 + (y - %.2f)^2 = %.2f', t, center(1), center(2), midjointRadius^2);
    circleEquation = sprintf('t = %d: (x - %.2f)^2 + (y - %.2f)^2 = %.2f', t, center(1), center(2), radius^2);
    disp(circleEquation);

    
    
    % Generate Points for Plotting Circle
    theta = linspace(0, 2*pi, 100);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    mx = center(1) + midjointRadius * cos(theta);
    my = center(2) + midjointRadius * sin(theta);
    point1Trans = [point1(1)-center(1),point1(2)-center(2)];
    point2Trans = [point2(1)-center(1),point2(2)-center(2)];
    midjointTrans = [midjoint(1)-center(1),midjoint(2)-center(2)];
    syms theta
    eqn = point1Trans(1) == point2Trans(1)*cos(theta) - point2Trans(2)*sin(theta);
    rotation = vpasolve(eqn,theta);
    midjointPrimeX = midjointTrans(1)*cos(rotation)-midjointTrans(2)*sin(rotation);
    midjointPrimeY = midjointTrans(1)*sin(rotation)+midjointTrans(2)*cos(rotation);
    midjointPrime = [midjointPrimeX+center(1),midjointPrimeY+center(2)];

    CosTheta = max(min(dot(midjointPrime,point1Trans)/(norm(midjointPrime)*norm(point1Trans)),1),-1);
    midjointAngle = real(acosd(CosTheta));

    % Plot End Effector Range of Motion
    plot(x, y); 
    % Plot midjoint Range of Motion
    plot(mx, my); 
    % Plot first link position one (RED)
    plot([center(1), midjoint(1)], [center(2), midjoint(2)], 'r-', 'LineWidth', 2); 
    % Plot second link position one (RED)
    plot([midjoint(1), point2(1)], [midjoint(2), point2(2)], 'r-', 'LineWidth', 2); 
    % Plot first link position two (BLUE)
    plot([center(1), midjointPrime(1)], [center(2), midjointPrime(2)], 'b-', 'LineWidth', 2); 
    % Plot second link position two (BLUE)
    plot([midjointPrime(1), point1(1)], [midjointPrime(2), point1(2)], 'b-', 'LineWidth', 2);
    fprintf('%.2f', midjointAngle);
    
end

% Additional Plotting Elements
plot(point1(1), point1(2), 'ro', 'MarkerSize', 8); % Plot Point 1 (Red)
plot(point2(1), point2(2), 'bo', 'MarkerSize', 8); % Plot Point 2 (Blue)
title('Circles Passing Through Two Points');
xlabel('x'); ylabel('y');
grid on;

hold off;
end