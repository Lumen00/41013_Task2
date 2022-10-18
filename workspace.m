clear all;
close all;
hold on;

%% GENERAL TO DO LIST:
% - Make environment
% - Make items
% - Make grippers
% - Visual servoing
% - System stop
% - Collision detectiono
% - GUI Movement/Controller Movement
% - Colour Rozum Pulse 75
% - Plan meal packaging
    % + Bagging vs... 
    % + Tray Placement?
    % + Cutlery
    % + Drinks
    % + Passing between robots
    % + Handover to customer/driver


%% Plot Critical Points

% Handover point - where bag is passed between robots.


% Dropoff point - where Dobot releases bag for collection.


% Cutlery points:
    % Cutlery pickup point - where Rozum collects cutlery package.

    % Cutlery dropoff point - where Rozum drops 

% Drink points:
    % Drink cup pickup point - where Rozum 


    % Drink cup fill point - where cup is placed to be filled

%% Generate Environment

% Set up robots.
r = RozumPulse75();
r.model.base = r.model.base*transl(0,-1.2,0);

d = Dobot(false);
d.model.base = r.model.base*trotx(pi/2)*transl(0,0.04,-0.8); % (x,z,y)
d.model.animate(zeros(1,6));


% Soft drinks dispenser.
    
    % Cola (legally distinct)
    softDrinkColaCoords = r.model.base*transl(-0.4,-80/1000,0.3);
    softDrinkCola_h = PlaceObject('Items\SoftDrink_Cola.ply', softDrinkColaCoords(1:3,4)'); 
    
    softDrinkDispenserColaCoords = softDrinkColaCoords*transl(0,0,-60/1000);
    softDrinkDispenserCola_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserColaCoords(1:3,4)'); 

    % Orange 
    softDrinkOrangeCoords = softDrinkColaCoords*transl(0,80/1000,0);
    softDrinkOrange_h = PlaceObject('Items\SoftDrink_Orange.ply', softDrinkOrangeCoords(1:3,4)'); 
    
    softDrinkDispenserOrangeCoords = softDrinkOrangeCoords*transl(0,0,-60/1000);
    softDrinkDispenserOrange_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserOrangeCoords(1:3,4)'); 

    % Lemon 
    softDrinkLemonCoords = softDrinkOrangeCoords*transl(0,80/1000,0);
    softDrinkCola_h = PlaceObject('Items\SoftDrink_Lemon.ply', softDrinkLemonCoords(1:3,4)'); 
    
    softDrinkDispenserLemonCoords = softDrinkLemonCoords*transl(0,0,-60/1000);
    softDrinkDispenserLemon_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserLemonCoords(1:3,4)'); 
%% Perform Task

axis equal;

q1 = zeros(1,6);
q2 = ones(1,6);

view(3);

%r.Travel(q1, q2, 100); % Only for independent movement of this robot.
%DTravel(d, q1, [-1, ones(1,5)], 100);

UniversalTravel(r, q1, q2, [20, 80], d, q1, [-1, ones(1,5)], [1, 50]);

%% UniversalTravel

function UniversalTravel(r1_h ,r1q1, r1q2, r1Steps, r2_h, r2q1, r2q2, r2Steps)

% Format args as:
% robot handle, q1, q2, [starting step, steps]... 

    % TO DO LIST: 
    % - Add choice between trapezoidal, quintic polynomial, RMRC.
    % - Manipulability measure for singularities and DLS
    % - Move simultaneously with other robots and all other ibjects with
    % this universal motion function.
    % - Attach points for items and grippers.

    % Build trajectories for Rozum and Dobot.

    % Building the Rozum trajectory:
    s = lspb(0,1,r1Steps(2));    % Create matrix describing trapezoidal trajectory 
    %                           movement varying smoothly from S0 to SF (time)
    %                           in M steps. V can be specified, but is computed
    %                           automatically. 
    qr1Matrix = nan(r1Steps(2), r1_h.model.n);
    for i=1:r1Steps(2)
      qr1Matrix(i,:) = (1-s(i))*r1q1 + s(i)*r1q2;
    end

    % Building the Dobot trajectory:

    s = lspb(0,1,r2Steps(2));    % Create matrix describing trapezoidal trajectory 
    %                           movement varying smoothly from S0 to SF (time)
    %                           in M steps. V can be specified, but is computed
    %                           automatically. 
    qr2Matrix = nan(r2Steps(2), r2_h.model.n);
    for i=1:r2Steps(2)
      qr2Matrix(i,:) = (1-s(i))*r2q1 + s(i)*r2q2;
    end

    % Decide the maximum step number for this specific function call.

    if (r1Steps(1) + r1Steps(2)) > (r2Steps(1) + r2Steps(2)) % r1 has the last step.
        maxSteps = r1Steps(1) + r1Steps(2);
    else % r2 has the last step.
        maxSteps = r2Steps(1) + r2Steps(2);
    end

    %size(qr1Matrix)
    %size(qr2Matrix)

    % Animate the movement.
    r1Index = 1;
    r2Index = 1;

    for i=1:maxSteps
    % Check if the starting step of each point has been reached.
        if (r1Steps(1) <= i) && (r1Index <= r1Steps(2)) % If the step is equal to or exceeds r1 starting step.
            r1_h.model.animate(qr1Matrix(r1Index,:));
            r1Index = r1Index + 1;
        end
        if (r2Steps(1) <= i) && (r2Index <= r2Steps(2)) % If the step is equal to or exceeds r2 starting step.
            r2_h.model.animate(qr2Matrix(r2Index,:));
            r2Index = r2Index + 1;
        end
    pause(0.01);
    end
end

%% Dobot Travel Method Outside of Class 
% Only for independent movement of the robot.
function DTravel(robot, q1, q2, steps)

            % TO DO LIST:
            % - Add choice between trapezoidal, quintic polynomial, RMRC.
            % - Manipulability measure for singularities and DLS
            % - Move simultaneously with the other robot and all other
            % objects with some universal motion function. 
            % - Attach points for items and grippers.



            s = lspb(0,1,steps);    % Create matrix describing trapezoidal trajectory 
            %                           movement varying smoothly from S0 to SF (time)
            %                           in M steps. V can be specified, but is computed
            %                           automatically. 
            qMatrix = nan(steps,robot.model.n);
            for i=1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end

            for i=1:steps
                robot.model.animate(qMatrix(i,:));
                pause(0.01);
            end


        end