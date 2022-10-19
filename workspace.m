clear all;
close all;
hold on;

%% Dobot testing for real use:
 
% d = Dobot(false);
% d.model.base = d.model.base*trotx(pi/2); % (x,z,y)
% drawnow;

% - Pickup: [0, 90, 30, 10, -30, 85]
% - Dropoff 1: [-1.33, 0, 0.5236, 0.1745, -0.5236, 1.4835]
% - Dropoff 2: [-1.33, 0, 0.5236, 0.8727, -0.5236, 1.4835]

% Pickup -> Open Gripper -> Close Gripper -> Dropoff 1 -> Dropoff 2 ->
% Open Gripper -> Dropoff 1 -> Close Gripper -> Pickup

% pickup = deg2rad([0, 90, 30, 10, -30, 85]);
% dropoff1 = [-1.33, 0, 0.5236, 0.1745, 0.8657, 1.4835];
% dropoff2 = [-1.33, 0, 0.5236, 0.8727, 0.1745, 1.4835];
% 
% DTravel(d, pickup, dropoff1, 50, 1);
% DTravel(d, dropoff1, dropoff2, 50, 1);
% DTravel(d, dropoff2, dropoff1, 50, 1);
% DTravel(d, dropoff1, pickup, 50, 1);
% d.model.teach;

%DTravel(d, q1, q2, 1);

%% GENERAL TO DO LIST:
% - Make environment
% - Make items
% - Make grippers
% - Visual servoing
% - System stop
% - Collision detection
% - GUI Movement/Controller Movement
% - Colour Rozum Pulse 75
% - Plan meal packaging
    % + Bagging vs... 
    % + Tray Placement?
    % + Cutlery
    % + Drinks
    % + Passing between robots
    % + Handover to customer/driver
% - Create pickup status for each robot and each item to indicate what each
% robot is holding. Travel functions will check for each robots' status and
% move whatever is set to true.
% - Safety


%% Plot Critical Points

% Handover point - where bag is passed between robots.


% Dropoff point - where Dobot releases bag for collection.


% Cutlery points:
    % Cutlery pickup point - where Rozum collects cutlery package.

    % Cutlery dropoff point - where Rozum drops 

% Drink points:
    % Drink cup pickup point - where Rozum 


    % Drink cup fill point - where cup is placed to be filled

%% Test gripper plotting
L1 = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 
gripper = SerialLink([L1], 'name', 'master');

% Plot the base and the master finger of the gripper.

            for linkIndex = 0:0
                %display(['Link',num2str(linkIndex),'.ply']);
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                gripper.faces{linkIndex + 1} = faceData;
                gripper.points{linkIndex + 1} = vertexData;
            end

% Plot the slave finger of the gripper.

% Animate both master and slave gripper into the starting configuration.

%gripper.plot(zeros(1,1));

%% Generate Environment

% Set up robots.
r = RozumPulse75();
r.model.base = r.model.base*transl(0,-0.8,0.5+0.35);
r.model.animate(ones(1,6));

d = Dobot(false);
d.model.base = r.model.base*trotx(pi/2)*transl(0,0.04,-1.0); % (x,z,y)
d.model.animate(zeros(1,6));

% Soft drinks dispenser. When picking one up, make a new instance of
% object.
    % Cola (legally distinct)
    softDrinkColaCoords = r.model.base*transl(-0.4,-80/1000,0.3);
    softDrinkCola_h = PlaceObject('Items\SoftDrink_Cola.ply', softDrinkColaCoords(1:3,4)'); 
    
    softDrinkDispenserColaCoords = softDrinkColaCoords*transl(0,0,-60/1000);
    softDrinkDispenserCola_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserColaCoords(1:3,4)'); 

    % Orange 
    softDrinkOrangeCoords = softDrinkColaCoords*transl(0,100/1000,0);
    softDrinkOrange_h = PlaceObject('Items\SoftDrink_Orange.ply', softDrinkOrangeCoords(1:3,4)'); 
    
    softDrinkDispenserOrangeCoords = softDrinkOrangeCoords*transl(0,0,-60/1000);
    softDrinkDispenserOrange_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserOrangeCoords(1:3,4)'); 

    % Lemon 
    softDrinkLemonCoords = softDrinkOrangeCoords*transl(0,100/1000,0);
    softDrinkCola_h = PlaceObject('Items\SoftDrink_Lemon.ply', softDrinkLemonCoords(1:3,4)'); 
    
    softDrinkDispenserLemonCoords = softDrinkLemonCoords*transl(0,0,-60/1000);
    softDrinkDispenserLemon_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserLemonCoords(1:3,4)'); 

% Plot bench space.
    
    % Countertop of the robots.
    counterRobotCoords = r.model.base*transl(0,2.4,-0.3325);
    counterRobot_h = PlaceObject('Items\Counter.ply', counterRobotCoords(1:3,4)'); 
    
    % Countertop of the meal pickup.
    counterCollection_h = PlaceObject('Items\Counter.ply', counterRobotCoords(1:3,4)'); 
    counterCollectionVerts = get(counterCollection_h,'Vertices');
    counterCollectionAdjust = trotz(pi/2)*transl(1.19,2-0.1,0);
    counterCollectionCoords = [counterCollectionVerts, ones(size(counterCollectionVerts,1),1)]*counterCollectionAdjust';
    set(counterCollection_h, 'Vertices', counterCollectionCoords(:,1:3));

    
% Plot safety barriers on countertops.



% Plot Meal Box next to Rozum bot.

    % Code Snippet for the closed box to be used after pushed into folding
    % space.
%     mealBox_ClosedCoords = r.model.base*transl(-0.4,0,0);
%     mealBox_Closed_h = PlaceObject('Items\MealBox_Closed.ply', mealBox_ClosedCoords(1:3,4)'); 

    % Place the open box model. 
    mealBox_OpenCoords = r.model.base*transl(0,-0.55,0);
    mealBox_Open_h = PlaceObject('Items\MealBox_Open.ply', mealBox_OpenCoords(1:3,4)'); 

% Plot cutlery boxes.



% Plot box folding space (Rozum bot is to push box in and pull it back
% out with the handles).



% Plot walls for holding equipment.



% Sensor in the meal box placement area to protect personnel and shut off
% the Rozum's movement (if it's doing anything).



drawnow;
    %% Perform Task

axis equal;

q1 = zeros(1,6);
q2 = ones(1,6);

view(3);

pickup = deg2rad([0, 90, 30, 10, -30, 85]);
dropoff1 = [-1, 0, 0.5236, 0.1745, 0.8657, 1.4835];
dropoff2 = [-1, 0, 0.5236, 0.8727, 0.1745, 1.4835];

% ROZUM Assembles the following into bag held by dobot.
% Cutlery
% Meal Box
% Drink

DTravel(d, pickup, dropoff1, 50, 1);
DTravel(d, dropoff1, dropoff2, 50, 1);
DTravel(d, dropoff2, dropoff1, 50, 1);
DTravel(d, dropoff1, pickup, 50, 1);
d.model.teach;

%RMRC(r,)

%UniversalTravel(r, q1, q2, [20, 80], d, q1, [-1, ones(1,5)], [1, 50], 1);

%% UniversalTravel
% Function to move both robots at once at varying time steps.
function UniversalTravel(r1_h ,r1q1, r1q2, r1Steps, r2_h, r2q1, r2q2, r2Steps, mode)

% Format args as:
% robot handle, q1, q2, [starting step, steps]... , mode

    % mode:
    %   1 - Trapezoidal
    %   2 - Quintic Polynomial

    % TO DO LIST: 
    % - Manipulability measure for singularities and DLS
    % - Move simultaneously with other robots and all other ibjects with
    % this universal motion function.
    % - Attach points for items and grippers.

    % Build trajectories for Rozum and Dobot.
    if mode == 1
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

    elseif mode == 2
        qr1Matrix = jtraj(r1q1, r1q2, r1Steps(2));
        qr2Matrix = jtraj(r2q1, r2q2, r2Steps(2));      
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

            % Move the designated object that has been picked up. Pass the
            % object's handle to the code.

            verticesAdjustTr = (r1_h.model.fkine(qr1Matrix(r1Index,:))); % Alters last position by this amount.
            transformedVertices = [self.vertices(:,:,brick),ones(size(self.vertices(:,:,brick),1),1)] * verticesAdjustTr'; 
            set(self.brick_h(brick,1), 'Vertices', transformedVertices(:,1:3));
            drawnow;

        end
        if (r2Steps(1) <= i) && (r2Index <= r2Steps(2)) % If the step is equal to or exceeds r2 starting step.
            r2_h.model.animate(qr2Matrix(r2Index,:));
            r2Index = r2Index + 1;

            % Move the designated object that has been picked up. Pass the
            % object's handle to the code.

            verticesAdjustTr = (r2_h.model.fkine(qr2Matrix(r2Index,:))); % Alters last position by this amount.
            transformedVertices = [self.vertices(:,:,brick),ones(size(self.vertices(:,:,brick),1),1)] * verticesAdjustTr'; 
            set(self.brick_h(brick,1), 'Vertices', transformedVertices(:,1:3));
            drawnow;

        end
    pause(0.01);
    end
end

%% RMRC
% Given a handle for a robot, coordinates
function RMRC(robot, T, q0, steps)

    % Resolved Motion Rate Control
    % Set steps
    steps = 100;
    
    % Set cartesian start and end points.
    x1 = [1.6, 1.005, ]';
    x2 = [1.1, -0.45, ]';
    deltaT = 0.5;                                        % Discrete time step
    
    % Create Cartesian waypoints for trajectory.
    x = zeros(2,steps);
    s = lspb(0,1,steps);                                 % Create interpolation scalar
    for i = 1:steps
        x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
    end
    
    % Create pose matrix.
    qRMatrix = nan(steps,2);
    T1 = [eye(3) [0 x1']'; zeros(1,3) 1];
    qRMatrix(1,:) = p2.ikcon(T,q0);                 % Solve for joint angles
    
    % Use RMRC to move between set Cartesian points.
    for i = 1:steps-1
        xdot = (x(:,i+1) - x(:,i))/deltaT;              % Calculate velocity at discrete time step
        J = robot.jacob0(qRMatrix(i,:));                    % Get the Jacobian at the current state

        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse


        qdot = inv(J)*xdot;                                                 % Solve velocitities via RMRC
        qRMatrix(i+1,:) =  qRMatrix(i,:) + deltaT*qdot';                    % Update next joint state
    
        J = robot.jacob0(qRMatrix(i,:));
  
        for j = 1:6                                                         % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)             % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)         % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end

    end

    %T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    %q0 = zeros(1,6);                                                            % Initial guess for joint angles
    qMatrix(1,:) = robot.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint    
end

%% Dobot Travel Method Outside of Class 
% Only for independent movement of the robot.
function DTravel(robot, q1, q2, steps, mode)

    % mode:
    %   1 - Trapezoidal
    %   2 - Quintic Polynomial

    % TO DO LIST:
    % - Add choice between trapezoidal, quintic polynomial, RMRC.
    % - Manipulability measure for singularities and DLS
    % - Move simultaneously with the other robot and all other
    % objects with some universal motion function. 
    % - Attach points for items and grippers.

    if mode == 1
        s = lspb(0,1,steps);    % Create matrix describing trapezoidal trajectory 
        %                           movement varying smoothly from S0 to SF (time)
        %                           in M steps. V can be specified, but is computed
        %                           automatically. 
        qMatrix = nan(steps,robot.model.n);
        for i=1:steps
            qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
       end
    elseif mode == 2
        qMatrix = jtraj(q1, q2, steps);
    end
       for i=1:steps
           robot.model.animate(qMatrix(i,:));
           pause(0.01);
       end
end