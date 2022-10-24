clear all;
close all;
hold on;

%% Set up controller for input (disable if you don't have the joystick)
global enableJoy
enableJoy = true;
id = 1;
global joy;
if enableJoy
    joy = vrjoystick(id);
else
    buttons = zeros(1,16);
end

%% Plot the collision rectangular prism for dobot. Change its position to test trajectory pathing.
placeObstacle = true;
if placeObstacle
    global vertex;
    global faces;
    global faceNormals;
    global rect_h;    
    centrePoint = [0.5, 0.3, 1];
    plotOptions.plotFaces = true;
    [vertex, faces, faceNormals, rect_h] = RectangularPrism(centrePoint-0.3, centrePoint+0.3, plotOptions);
end

%% Set up item pickup status globally.
global itemPickup; % 0 - none, 1 - cola, 2 - cutlery, 3 - sauce
itemPickup = 0;

global mealPickup;
mealPickup = 0;

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
% - Visual servoing
% - Collision detection
% - GUI Movement/Controller Movement
% - Plan meal packaging
    % + Passing between robots
% - Create pickup status for each robot and each item to indicate what each
% robot is holding. Travel functions will check for each robots' status and
% move whatever is set to true.


%% Gripper plotting
% L1 = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 
% gripper = SerialLink([L1], 'name', 'master');

% Plot the base and the master finger of the gripper.

%             for linkIndex = 0:0
%                 %display(['Link',num2str(linkIndex),'.ply']);
%                 [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
%                 gripper.faces{linkIndex + 1} = faceData;
%                 gripper.points{linkIndex + 1} = vertexData;
%             end

gripperOffset = 0.15;

%% Generate Environment

% Set up robots.
global r;
r = RozumPulse75();
r.model.base = r.model.base*transl(0,-0.8,0.85);
r.model.animate(zeros(1,6));

global d;
d = DobotMagician();
d.model.base = r.model.base*transl(0,0.9,0); % Globally: (x,z,y)
dNeutral = deg2rad([-88, 5, 0, 107, 0]);
d.model.animate(dNeutral);

% Soft drinks and other items dispenser. 
    % Cola (legally distinct)
    softDrinkColaCoords = r.model.base*transl(-0.4,-80/1000,0.3);
    global softDrinkCola_h;
    softDrinkCola_h = PlaceObject('Items\SoftDrink_Cola.ply', softDrinkColaCoords(1:3,4)'); 
    
    softDrinkDispenserColaCoords = softDrinkColaCoords*transl(0,0,-60/1000);
    softDrinkDispenserCola_h = PlaceObject('Items\SoftDrinkDispenser.ply', softDrinkDispenserColaCoords(1:3,4)'); 

    % Cutlery Container 
    cutleryCoords = softDrinkColaCoords*transl(0,100/1000,0);
    global cutlery_h;
    cutlery_h = PlaceObject('Items\CutleryContainer.ply', cutleryCoords(1:3,4)'); 
    
    cutleryDispenserCoords = cutleryCoords*transl(0,0,-60/1000);
    cutleryDispenser_h = PlaceObject('Items\SoftDrinkDispenser.ply', cutleryDispenserCoords(1:3,4)'); 

    % Sauce Container)
    sauceCoords = cutleryCoords*transl(0,100/1000,0);
    global sauce_h;    
    sauce_h = PlaceObject('Items\SauceContainer.ply', sauceCoords(1:3,4)'); 
    
    sauceDispenserCoords = sauceCoords*transl(0,0,-60/1000);
    sauceDispenser_h = PlaceObject('Items\SoftDrinkDispenser.ply', sauceDispenserCoords(1:3,4)'); 

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

    
% Plot Meal Box next to Rozum bot.

    % Code Snippet for the closed box to be used after pushed into folding
    % space.
%     mealBox_ClosedCoords = r.model.base*transl(-0.4,0,0);
%     mealBox_Closed_h = PlaceObject('Items\MealBox_Closed.ply', mealBox_ClosedCoords(1:3,4)'); 

    % Place the open box model. 
    mealBox_OpenCoords = r.model.base*transl(0,-0.55,0);
    mealBox_Open_h = PlaceObject('Items\MealBox_Open.ply', mealBox_OpenCoords(1:3,4)'); 


% Plot walls for holding equipment.

yConstant = r.model.base(2,4)-1;
xHighBound = r.model.base(1,4)+1.5;
xLowBound = r.model.base(1,4)-0.9;
zHighBound = 2;
zLowBound = 0;
wall1_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yConstant,yConstant;yConstant,yConstant],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\wall.jpg'),'FaceColor','texturemap');

xConstant = r.model.base(1,4)-0.9;
yHighBound = r.model.base(2,4)+2.4;
yLowBound = r.model.base(2,4)-1;
zHighBound = 2;
zLowBound = 0;
wall2_h = surf([xConstant,xConstant;xConstant,xConstant],[yLowBound,yLowBound;yHighBound,yHighBound],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\wall.jpg'),'FaceColor','texturemap');

% Customer-Employee Dividing Wall

yConstant = r.model.base(2,4)+1;
xHighBound = r.model.base(1,4)+1.5;
xLowBound = r.model.base(1,4)+0.4;
zHighBound = 0.7;
zLowBound = 0;
wall3_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yConstant,yConstant;yConstant,yConstant],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\wall.jpg'),'FaceColor','texturemap');

yConstant = r.model.base(2,4)+1;
xHighBound = r.model.base(1,4)+1.5;
xLowBound = r.model.base(1,4)+0.4;
zHighBound = 2;
zLowBound = 0.7;
LookingGlass_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yConstant,yConstant;yConstant,yConstant],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\SafetyGlass.png'),'FaceColor','texturemap', 'FaceAlpha', 0.2);

% Plot safety glass walls. Use these for link collision checks later.

yConstant = r.model.base(2,4)+2.4;
xHighBound = r.model.base(1,4)+0.4;
xLowBound = r.model.base(1,4)-0.9;
zHighBound = 2;
zLowBound = r.model.base(3,4);
SafetyGlass1_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yConstant,yConstant;yConstant,yConstant],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\SafetyGlass.png'),'FaceColor','texturemap', 'FaceAlpha', 0.2);

xConstant = r.model.base(1,4)+0.4;
yHighBound = r.model.base(2,4)+2.4;
yLowBound = r.model.base(2,4)+1.8;
zHighBound = 2;
zLowBound = r.model.base(3,4);
SafetyGlass2_h = surf([xConstant,xConstant;xConstant,xConstant],[yLowBound,yLowBound;yHighBound,yHighBound],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\SafetyGlass.png'),'FaceColor','texturemap', 'FaceAlpha', 0.2);

% Plot lego man as an employee. Robots will stop if moved into hazard area.
person_Coords = r.model.base*transl(1.7,0,-0.85);
global person_h;
person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');

% Sensor in the meal box placement area to protect personnel and shut off
% the Rozum's movement (if it's doing anything).

    % PID
    PID_h = PlaceObject('Items\Sensor.ply'); 

    PIDVerts = get(PID_h,'Vertices');
    PIDAdjust = r.model.base*troty(pi/2)*transl(0.3,0,0.4);
    PIDTransformedVerts = [PIDVerts, ones(size(PIDVerts,1),1)]*PIDAdjust';
    set(PID_h, 'Vertices', PIDTransformedVerts(:,1:3));

    % Emergency STOP (In hazard zone)
    EStop_h = PlaceObject('Items\Emergency_Stop.ply'); 

    EStopVerts = get(EStop_h,'Vertices');
    EStopAdjust = r.model.base*transl(0.3,-0.4,0);
    EStopTransformedVerts = [EStopVerts, ones(size(EStopVerts,1),1)]*EStopAdjust';
    set(EStop_h, 'Vertices', EStopTransformedVerts(:,1:3));


    % Emergency STOP (Outside of hazard zone)
    EStop_Outside_h = PlaceObject('Items\Emergency_Stop.ply'); 

    EStop_OutsideVerts = get(EStop_Outside_h,'Vertices');
    EStop_OutsideAdjust = r.model.base*trotx(pi/2)*transl(1.3,0,1.08);
    EStop_OutsideTransformedVerts = [EStop_OutsideVerts, ones(size(EStop_OutsideVerts,1),1)]*EStop_OutsideAdjust';
    set(EStop_Outside_h, 'Vertices', EStop_OutsideTransformedVerts(:,1:3));


    % Siren/Alarm
    Alarm_h = PlaceObject('Items\Alarm.ply'); 

    AlarmVerts = get(Alarm_h,'Vertices');
    AlarmAdjust = r.model.base*trotz(-pi)*transl(0.7,-1.5,0.4);
    AlarmTransformedVerts = [AlarmVerts, ones(size(AlarmVerts,1),1)]*AlarmAdjust';
    set(Alarm_h, 'Vertices', AlarmTransformedVerts(:,1:3));    

    
    % Fire Extinguisher (Inside hazards zone)
    FireExt_h = PlaceObject('Items\fireextinguisherco2.ply'); 

    FireExtVerts = get(FireExt_h,'Vertices');
    FireExtAdjust = r.model.base*transl(1.2,-0.9,-0.4);
    FireExtTransformedVerts = [FireExtVerts, ones(size(FireExtVerts,1),1)]*FireExtAdjust';
    set(FireExt_h, 'Vertices', FireExtTransformedVerts(:,1:3));        


    % Fire Extinguisher (Outside hazard zone)
    FireExt_Outside_h = PlaceObject('Items\fireextinguisherco2.ply'); 

    FireExt_OutsideVerts = get(FireExt_Outside_h,'Vertices');
    FireExt_OutsideAdjust = r.model.base*transl(0.7,-1.1,-0.4);
    FireExt_OutsideTransformedVerts = [FireExt_OutsideVerts, ones(size(FireExt_OutsideVerts,1),1)]*FireExt_OutsideAdjust';
    set(FireExt_Outside_h, 'Vertices', FireExt_OutsideTransformedVerts(:,1:3));   
   
    % Signage
    xConstant = r.model.base(1,4)-0.8;
    yHighBound = r.model.base(2,4)-0.15-0.6;
    yLowBound = r.model.base(2,4)+0.15-0.6;
    zHighBound = r.model.base(3,4)+0.8;
    zLowBound = r.model.base(3,4)+0.3;
    sign1_h = surf([xConstant,xConstant;xConstant,xConstant],[yLowBound,yLowBound;yHighBound,yHighBound],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\cobotSign.jpg'),'FaceColor','texturemap');

    xConstant = r.model.base(1,4)+0.41;
    yHighBound = r.model.base(2,4)+2.3;
    yLowBound = r.model.base(2,4)+2.1;
    zHighBound = r.model.base(3,4)+0.8;
    zLowBound = r.model.base(3,4)+0.3;
    sign2_h = surf([xConstant,xConstant;xConstant,xConstant],[yHighBound,yHighBound;yLowBound,yLowBound],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\cobotSign.jpg'),'FaceColor','texturemap');

    % Set up hazard area for robot
    floor = r.model.base(3,4)-0.84;
    xHighBound = r.model.base(1,4)+1.5;
    xLowBound = r.model.base(1,4)-1.5;
    yHighBound = r.model.base(2,4)+2.4;
    yLowBound = r.model.base(2,4)-1;
    hazardFloor_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yLowBound,yHighBound;yLowBound,yHighBound],[floor,floor;floor,floor],'CData',imread('Items\hazardFloor.jpg'),'FaceColor','texturemap');

drawnow;
%% Perform Task

axis equal;

q1 = zeros(1,6);
q2 = ones(1,6);

view(142.5, 30);
keyboard; % Wait for approval to begin task.
%% Navigation for ROZUM picking up soft drink. 

% Extract the transform of the attach point of the soft drink.

softDrinkTr = softDrinkColaCoords*troty(-pi/2)*transl(0,0,-gripperOffset);

% Define the approach transform to the soft drink (away by 0.1 m in x axis)

softDrinkApproachTr = softDrinkTr*transl(0,0,-0.1-gripperOffset);

% Define the initial pose to select the drink before approach.

qGuess = deg2rad([0, 0, 120, 60, 90,-180]);

% For the approach and pickup transforms, get their poses. Use initial
% guesses for best results.

qApproach = r.model.ikcon(softDrinkApproachTr, qGuess);
qPickup = r.model.ikcon(softDrinkTr, qApproach);

% Complete pickup travel moves.

r.Travel(q1, qApproach , 50);

r.Travel(qApproach, qPickup, 30);

% Take the pickup transform and lift it by 0.1 m in the local x.
drinkLiftTr = softDrinkTr*transl(0.1,0,0);

% Move to above the meal box for insertion. May require initial guess for
% good joint lengths to avoid funky collisions.
drinkBoxAboveTr = mealBox_OpenCoords*transl(0.07,0,0.35)*trotx(pi)*trotz(pi/2);

qGuess = deg2rad([80, 20, 90, -20, -90, 0]);

% Animate RMRC computed

% Move the soft drink out of the dispenser. Use RMRC to control velocity.
itemPickup = 1;
lastPose = RMRC(r, softDrinkTr, drinkLiftTr, qPickup, 40, 0.2);

% Move the soft drink to over the meal box.
qAbove = r.model.ikcon(drinkBoxAboveTr, qGuess);
r.Travel(lastPose, qAbove, 100);

% Move the soft drink into the meal box and deposit it. Use RMRC to control
% velocity.
lastPose = RMRC(r, drinkBoxAboveTr, drinkBoxAboveTr*transl(0,0,0.5), qAbove, 40, 0.2);
itemPickup = 0;

% Move the robot out of the box.
lastPose = RMRC(r, drinkBoxAboveTr*transl(0,0,0.5), drinkBoxAboveTr, lastPose, 40, 0.2);

% Return to neutral pose.

r.Travel(lastPose, q1, 50);

%% Navigation for ROZUM picking up cutlery

% Transform of cutlery container's attach point.

cutleryTr = cutleryCoords*troty(-pi/2)*transl(0,0,-gripperOffset);

% The approach transform to the container.

cutleryApproachTr = cutleryTr*transl(0,0,-0.05-gripperOffset);

% Iniitial pose guess.

qGuess = deg2rad([-155, 14.4, -128, -66, -64.8, -90]);

% Get the poses for the approach and pickup.

qApproach = r.model.ikcon(cutleryApproachTr, qGuess);
qPickup = r.model.ikcon(cutleryTr, qApproach);

% Complete pickup travel moves.

r.Travel(q1, qApproach , 50);

r.Travel(qApproach, qPickup, 30);

% Next stage: Lift and put in box.
% Lift container
itemPickup = 2;
cutleryLiftTr = cutleryTr*transl(0.1,0,0);

lastPose = RMRC(r, cutleryTr, cutleryLiftTr, qPickup, 40, 0.2);

% Move to above box.
qGuess = deg2rad([80, 20, 90, -20, -90, 0]);

cutleryBoxAboveTr = mealBox_OpenCoords*transl(0,0,0.35)*trotx(pi)*trotz(pi/2);

qAbove = r.model.ikcon(cutleryBoxAboveTr, qGuess);
r.Travel(lastPose, qAbove, 100);

% Insert into box and release
lastPose = RMRC(r, cutleryBoxAboveTr, cutleryBoxAboveTr*transl(0,0,0.5), qAbove, 40, 0.2);
itemPickup = 0;

% Move out of box
lastPose = RMRC(r, cutleryBoxAboveTr*transl(0,0,0.5), cutleryBoxAboveTr, lastPose, 40, 0.2);

% Return to neutral pose.
r.Travel(lastPose, q1, 50);

%% Navigation for ROZUM picking up sauce
% Transform of sauce container's attach point.

sauceTr = sauceCoords*troty(-pi/2)*transl(0,0,-gripperOffset);

% The approach transform to the container.

sauceApproachTr = sauceTr*transl(0,0,-0.1);

% Iniitial pose guess.

qGuess = deg2rad([180, 46.8, -128, -98.4, -90, -90]);

% Get the poses for the approach and pickup.

qApproach = r.model.ikcon(sauceApproachTr, qGuess);
qPickup = r.model.ikcon(sauceTr, qApproach);

% Complete pickup travel moves.

r.Travel(q1, qApproach , 50);

r.Travel(qApproach, qPickup, 30);

% Next stage: Lift and put in box.
% Lift container
itemPickup = 3;
sauceLiftTr = sauceTr*transl(0.1,0,0);

lastPose = RMRC(r, sauceTr, sauceLiftTr, qPickup, 40, 0.2);

% Move to above box.
qGuess = deg2rad([80, 20, 90, -20, -90, 0]);

sauceBoxAboveTr = mealBox_OpenCoords*transl(-0.07,0,0.35)*trotx(pi)*trotz(pi/2);

qAbove = r.model.ikcon(sauceBoxAboveTr, qGuess);
r.Travel(lastPose, qAbove, 100);

% Insert into box and release
lastPose = RMRC(r, sauceBoxAboveTr, sauceBoxAboveTr*transl(0,0,0.5), qAbove, 40, 0.2);
itemPickup = 0;

% Move out of box
lastPose = RMRC(r, sauceBoxAboveTr*transl(0,0,0.5), sauceBoxAboveTr, lastPose, 40, 0.2);

% Return to neutral pose.
r.Travel(lastPose, q1, 50);


%% Navigation for handover of meal box.
delete(mealBox_Open_h);
delete(softDrinkCola_h);
delete(cutlery_h);
delete(sauce_h);
%%
mealBox_ClosedCoords = r.model.base*transl(0,-0.55,0);
global mealBox_Closed_h;
%delete(mealBox_Closed_h);
mealBox_Closed_h = PlaceObject('Items\MealBox_Closed.ply', mealBox_ClosedCoords(1:3,4)'); 

% Rozum

    % Direct robot to above the rozum and pick up the meal box.

    mealBoxPickupTr = mealBox_ClosedCoords*transl(0,0,0.175+0.1+gripperOffset)*trotx(pi)*trotz(pi/2);
    mealBoxPpickupQ = r.model.ikcon(mealBoxPickupTr, lastPose);

    r.Travel(zeros(1,6), mealBoxPpickupQ, 60);
    lastPoseQ = RMRC(r, mealBoxPickupTr, mealBoxPickupTr*transl(0,0,0.1), mealBoxPpickupQ, 30, 0.2);
    mealPickup = 1;
    lastPoseQ = RMRC(r, mealBoxPickupTr*transl(0,0,0.1), mealBoxPickupTr, lastPoseQ, 30, 0.2);

    % Swivel robot 180 degrees.
    r.Travel(lastPoseQ, [(lastPoseQ(1) +pi) ,lastPoseQ(2:end)], 50);
    
    mealBoxHandoverTr = r.model.fkine([(lastPoseQ(1) +pi) ,lastPoseQ(2:end)]);
    
    lastPoseQ = RMRC(r, mealBoxHandoverTr, mealBoxHandoverTr*transl(0.3,0,0), [(lastPoseQ(1) +pi) ,lastPoseQ(2:end)], 30, 0.2);
    mealPickup = 0;
%%

% Dobot
    % Navigate to neutral pose
    dNeutral = deg2rad([-88, 5, 0, 107, 0]);
    DTravel(d, d.model.getpos, dNeutral, 50, 1);

    % Build pickup transform. 
    handOverTr = r.model.fkine(lastPoseQ); 
    pickupTr = handOverTr*trotx(pi);

    pickupQ = deg2rad([-88, 20, 52.6, 107, 0]);
       
    DTravel(d, dNeutral, pickupQ, 50, 1);
    mealPickup = 0;
    r.Travel(lastPoseQ, q1, 40); % Return to neutral pose.

    mealPickup = 1;

% Move to dropoff location. Inspect trajectory for collisions and halt if
% there is one before moving. 

dropoffQ = [(pickupQ(1)+pi), pickupQ(2:end)]; %d.model.ikcon(dropoffTr, pickupQ);
dropoffTr = d.model.fkine(dropoffQ);
DTravel(d, pickupQ, dropoffQ, 50, 1);

% RMRC to lower and leave the meal box.
releaseTr = d.model.fkine(dropoffQ)*transl(0,0,-0.02);
releaseQ = d.model.ikcon(releaseTr, dropoffQ);

DTravel(d, dropoffQ, releaseQ, 20, 1);

%DTravel(d, pickup, dropoff1, 50, 1);
mealPickup = 0;
DTravel(d, releaseQ, dropoffQ, 20, 1);
DTravel(d, dropoffQ, dNeutral, 40, 1);

keyboard;


%% Robot Jogging
itemPickup = 0;
mealPickup = 0;
RobotControl(joy, r);
RobotControl(joy, d);

keyboard;
%% VisualServoing test
itemPickup = 0;
mealPickup = 0;
VisualServoingSafety(r, d);

%% RobotControl
function RobotControl(joy, robot)

    [axes, buttons, povs] = read(joy);
    k = 0.3;
    q = robot.model.getpos;
    dt = 0.15;
    jointSelect = 1;
    fprintf('Selected joint %i\n', jointSelect);
    qEditDot = 0;
    while buttons(3) == 0 % While X is not pressed:
        [axes, buttons, povs] = read(joy);
        x = k*axes(1); % left-right on left joystick (x)
        y = k*axes(2); % up-down on left joystick (y)
        z = k*axes(5); % up-down  on right joystick (z)
        
        if (buttons(5) == 1) && (jointSelect < 6) % Left bumper goes up.
            jointSelect = jointSelect + 1;
            fprintf('Selected joint %i\n', jointSelect);
            pause(1);
        elseif (buttons(6) == 1) && (jointSelect > 1) % Right bumper goes down.
            jointSelect = jointSelect - 1;
            fprintf('Selected joint %i\n', jointSelect);
            pause(1);
        end
        qEditDot = k*axes(3);
        

        vel = [x, y, z, 0, 0, 0];

        epsilon = 0.1;
        J = robot.model.jacob0(q);                 % Get Jacobian at current joint state
        m = sqrt(det(J*J'));
        if m < epsilon  % If manipulability is less than given threshold
            lambda = 0.1;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(robot.model.n))*J';                   % DLS Inverse        
    
        qdot = invJ*vel';
        qdot(jointSelect) = qdot(jointSelect)+qEditDot;
        
        q = q + qdot'*dt;
    
        robot.model.animate(q);  
        pause(0.1);
    end
end

%% VisualServoingSafety
% Start the visual servoing safety demonstration. Note that the camera will
% clear the figure once the function exits.
function VisualServoingSafety(r, d)
    % the Dobot will hold the safety sign that the Rozum has to retreat
    % from. The Rozum will have a camera mounted to its end effector. It is
    % assumed it occupies the same space as the Rozum's gripper.

    % Load in the safety sign and attach it to the Dobot's end effector at
    % a pose where the Rozum can see it.'

    rozumStartPose = deg2rad([-90, 11.6, 101, 68.4, 90, 0]);
    dobotTargetPose = deg2rad([-88, 5, 13.5, 161, -85]);

    r.Travel(r.model.getpos, zeros(1,6), 40); % Move Rozum to safe pose first.

    UniversalTravel(r, zeros(1,6), rozumStartPose, [1, 40], d, d.model.getpos, dobotTargetPose, [1, 40], 1);

    % Now that both robots are in their correct poses, get end effector
    % position of the Dobot.

    Tr = d.model.fkine(dobotTargetPose);
    
    yConstant = Tr(2,4);
    xHighBound = Tr(1,4)-0.11;
    xLowBound = Tr(1,4)+0.11;
    zHighBound = Tr(3,4)-0.1;
    zLowBound = Tr(3,4);
    VSSign_h = surf([xLowBound,xLowBound;xHighBound,xHighBound],[yConstant,yConstant;yConstant,yConstant],[zLowBound,zHighBound;zLowBound,zHighBound],'CData',imread('Items\SafetySign.jpg'),'FaceColor','texturemap');

    % Plot points on the safety sign to follow (use 3).

    P=[xLowBound,   xLowBound,   xHighBound;
       yConstant, yConstant,  yConstant;
       zLowBound,  zHighBound,  zLowBound];

    spherePoints_h = plot_sphere(P, 0.01, 'g');

    % Mount the camera onto the Dobot  

    cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'DobotCamera');
    
    % The desirable transform is the centre of the sign with variable Y. 

    signCentre = [Tr(1,4), Tr(2,4), (zHighBound+zLowBound)/2];
    cameraMount = r.model.fkine(rozumStartPose);
    depth = abs(cameraMount(2,4)-signCentre(2)); % Depth of the IBVS. Distance from camera to points along axis.

    rozumEndPose = r.model.ikcon(cameraMount*transl(0,0,-0.5), rozumStartPose);

    Tc0 = cameraMount*transl(0,0,-0.3);
    trplot(Tc0);

    cam.plot_camera('Tcam', cameraMount, 'scale', 0.15);
    

    % Create the 2D Image view
    cam.clf();
        % What we want the camera to see. P as seen from desired Transform.
        desiredCam = cam.plot(P, 'Tcam', Tc0, 'o');
        cam.hold(true);
        
        % What the camera actually sees. P as seen from current Transform.
        cam.T = cameraMount;
        actualCam = cam.plot(P, '*');
        pause(2);
        cam.hold(true); 

    % Begin visual servoing and RMRC retreat for specified steps.
    
        % Continuously update the target points by updating the Tc0
        % transform by how far back the Dobot has moved.
        
        % Set VS parameters:
        fps = 25;     % Camera Frame Rate.
        lambda = 0.3; % Controller gain.

    qr = rozumStartPose; % Pose of rozum to modify.
    % VS Loop will run the following steps:
    % 1. VS to align in starting pose.
    % 2. RMRC to move to retreat coordinates.
    % 3. VS to realign in end pose to account for any error introduced in
    % RMRC.
    keyboard;
    while true
        % Visual servoing section

            % Calculate the simulated depth of the IBVS.
            depth = abs(cameraMount(2,4)-signCentre(2)); % Depth of the IBVS. Distance from camera to points along axis.

            % Insert current view of camera in cam.T.
            uv = cam.plot(P)
            
            % Calculate the image plane error (as column)
            e = desiredCam - uv;
            e = e(:)

            % If error is sufficiently small, VS is successful. 
            errorCheck = 0;
            for i=1:6 % index through e and count entries with abs(e)<1.
                if abs(e(i)) < 1
                    errorCheck = errorCheck + 1;
                end
            end
            if errorCheck == 6
               break;
            end            

            % Calculate the image Jacobian 
            J = cam.visjac_p(uv, depth);

            % Calculate the velocity of the camera in camera frame for the next move using
            % the error and image Jacobian.
            v = lambda*pinv(J)*e;

            % Compute the Jacobian and inverse Jacobian of the Dobot in its
            % end effector frame.
            Jd = r.model.jacobn(qr);
            invJd = pinv(Jd);

            % Get the joint velocities of the dobot.
            qrDot = invJd*v;

            % Limit maximum angular velocity for robot.
             ind=find(qrDot(:)>2*pi);
             if ~isempty(ind)
                 qrDot(ind)=2*pi;
             end
             ind=find(qrDot(:)<-2*pi);
             if ~isempty(ind)
                 qrDot(ind)=-2*pi;
             end

             % Check that all moves are within joint limits. Set
             % velocity to zero if this is the case.
%              for j = 2:6                                                % Loop through joints 1 to 6
%                  if qd(j) + (1/fps)*qdDot(j) < d.model.qlim(j,1)              % If next joint angle is lower than joint limit...
%                     qdDot(j) = 0; % Stop the motor
%                  elseif qd(j) + (1/fps)*qdDot(j) > d.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
%                     qdDot(j) = 0; % Stop the motor
%                  end
%              end

             % Animate the rozum's joints.
             qr = qr + (1/fps)*qrDot';

             r.model.animate(qr);

             % Get the new camera location as a result of the dobot's
             % movement.

             cameraMount = r.model.fkine(qr);
             cam.T = cameraMount;
             drawnow;

             % Add pause to simulate the impact of the fps.
             pause(1/fps);
           
    end
    disp(" VS SUCCESS");

    keyboard;
    cam.clf();
end

%% UniversalTravel
% Function to move both robots at once at varying time steps.
function UniversalTravel(r1_h ,r1q1, r1q2, r1Steps, r2_h, r2q1, r2q2, r2Steps, mode)

global joy;

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
            % For each step, check controller input for two modes to stop
            % movement:
            % 1. E-Stop
            % 2. Sensing of personnel in hazard zone. 
            
                % Read controller input (Using XBONE configuration)
                [axes, buttons, povs] = read(joy);
                
                % Press A (button 1) to stop movement and enter infinite
                % loop. Press A (button 1) to prime restart. Press B 
                % (button 2) to restart movement.
                if buttons(1) == 1 % STOP
                    disp('STOP: USER INPUT');
                    pause(2); % Let button unpress.
                    eStopStatus = 0; % 0 - stopped, 1 - primed
                    while true
                        pause(0.1);
                        [axes, buttons, povs] = read(joy);
                        if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                            disp('RESTART PRIMED');
                            eStopStatus = 1;
                            pause(2); % Let button unpress.
                        elseif (buttons(1) == 1) && (eStopStatus == 1)
                            disp('RESTART ABORTED');
                            eStopStatus = 0;
                            pause(2);
                        elseif (buttons(2) == 1) && (eStopStatus == 1)
                            disp('RESTARTING');
                            break;
                        end
                    end
                elseif buttons(4) == 1% Press Y to move lego man into hazard zone and stop robots.
                    person_Coords = r.model.base*transl(0.7,0,-0.85);
                    delete(person_h);
                    person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                    disp('STOP: SENSOR INPUT');
                    pause(2); % Let button unpress.
                    eStopStatus = 0; % 0 - stopped, 1 - primed
                    while true
                        pause(0.1);
                        [axes, buttons, povs] = read(joy);
                        if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                            disp('RESTART PRIMED');
                            eStopStatus = 1;
                            pause(2); % Let button unpress.
                        elseif (buttons(1) == 1) && (eStopStatus == 1)
                            disp('RESTART ABORTED');
                            eStopStatus = 0;
                            pause(2);
                        elseif (buttons(2) == 1) && (eStopStatus == 1)
                            disp('RESTARTING');
                            person_Coords = r.model.base*transl(1.7,0,-0.85);
                            delete(person_h);
                            person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                            break;
                        end
                    end
                end


    % Check if the starting step of each point has been reached.
        if (r1Steps(1) <= i) && (r1Index <= r1Steps(2)) % If the step is equal to or exceeds r1 starting step.
            r1_h.model.animate(qr1Matrix(r1Index,:));
            r1Index = r1Index + 1;

            % Move the designated object that has been picked up. Pass the
            % object's handle to the code.
% 
%             verticesAdjustTr = (r1_h.model.fkine(qr1Matrix(r1Index,:))); % Alters last position by this amount.
%             transformedVertices = [self.vertices(:,:,brick),ones(size(self.vertices(:,:,brick),1),1)] * verticesAdjustTr'; 
%             set(self.brick_h(brick,1), 'Vertices', transformedVertices(:,1:3));
            drawnow;

        end
        if (r2Steps(1) <= i) && (r2Index <= r2Steps(2)) % If the step is equal to or exceeds r2 starting step.
            r2_h.model.animate(qr2Matrix(r2Index,:));
            r2Index = r2Index + 1;

            % Move the designated object that has been picked up. Pass the
            % object's handle to the code.

%             verticesAdjustTr = (r2_h.model.fkine(qr2Matrix(r2Index,:))); % Alters last position by this amount.
%             transformedVertices = [self.vertices(:,:,brick),ones(size(self.vertices(:,:,brick),1),1)] * verticesAdjustTr'; 
%             set(self.brick_h(brick,1), 'Vertices', transformedVertices(:,1:3));
%             drawnow;

        end
    pause(0.01);
    end
end

%% RMRC
% Given a handle for a robot, coordinates
function lastPose = RMRC(robot, T1, T2, q0, steps, deltaT)
    global joy;
    global person_h;
    global r;
    global d;
    global itemPickup;
    global softDrinkCola_h;
    global cutlery_h;
    global sauce_h;
    global mealPickup;
    global mealBox_Closed_h;
    global enableJoy;


    % Function requires:
    % - Handle of the robot being used.
    % - Initial transform 
    % - Final transform
    % - Initial guess to help inverse kinematics
    % - Desired steps
    % - deltaT to set velocity
   
    % Set lowest manipulability threshold.
    epsilon = 0.1;

    % Create Cartesian waypoints for trajectory.
    x = zeros(3,steps);

        % Extract Cartesian coordinates from given transforms.
        x1 = T1(1:3,4);
        x2 = T2(1:3,4);
    
        s = lspb(0,1,steps);                                 % Create interpolation scalar
        for i = 1:steps
            x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y-z plane
        end
    
    % Create RPY waypoints for trajectory.
    theta = zeros(3, steps);

        % Extract RPY from given transforms.
        theta1 = tr2rpy(T1)';
        theta2 = tr2rpy(T2)';

        s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
        for i=1:steps
            theta(:,i) = theta1*(1-s(i)) + s(i)*theta2;       
        end


    % Create pose matrix.
    qRMatrix = nan(steps,6);
    qRMatrix(1,:) = q0;% From initial guess which is chosen pose.   %robot.model.ikcon(T1,q0);                 % Solve for joint angles
    
    m = zeros(1,steps-1);

    % Use RMRC to move between set Cartesian points.
    for i = 1:steps-1
        % Get linear velocity
        xdot = (x(:,i+1) - x(:,i))/deltaT;              % Calculate velocity at discrete time step
        
        % Get angular velocity

        T = robot.model.fkine(qRMatrix(i,:));

        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric
        
        thetadot = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix
    
        velocity = [xdot;thetadot];

        J = robot.model.jacob0(qRMatrix(i,:));                    % Get the Jacobian at the current state

        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = 0.1;%(1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse

        qdot = invJ*velocity;                                                 % Solve velocitities via RMRC
        qRMatrix(i+1,:) =  qRMatrix(i,:) + deltaT*qdot';                    % Update next joint state
      
        for j = 1:6                                                         % Loop through joints 1 to 6
            if qRMatrix(i,j) + deltaT*qdot(j)' < robot.model.qlim(j,1)             % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qRMatrix(i,j) + deltaT*qdot(j)' > robot.model.qlim(j,2)         % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end

            % For each step, check controller input for two modes to stop
            % movement:
            % 1. E-Stop
            % 2. Sensing of personnel in hazard zone. 
            
                % Read controller input (Using XBONE configuration)
                if enableJoy
                    [axes, buttons, povs] = read(joy);
                

                    % Press A (button 1) to stop movement and enter infinite
                    % loop. Press A (button 1) to prime restart. Press B 
                    % (button 2) to restart movement.
                    if buttons(1) == 1 % STOP
                        disp('STOP: USER INPUT');
                        pause(2); % Let button unpress.
                        eStopStatus = 0; % 0 - stopped, 1 - primed
                        while true
                            pause(0.1);
                            [axes, buttons, povs] = read(joy);
                            if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                                disp('RESTART PRIMED');
                                eStopStatus = 1;
                                pause(2); % Let button unpress.
                            elseif (buttons(1) == 1) && (eStopStatus == 1)
                                disp('RESTART ABORTED');
                                eStopStatus = 0;
                                pause(2);
                            elseif (buttons(2) == 1) && (eStopStatus == 1)
                                disp('RESTARTING');
                                break;
                            end
                        end
                    elseif buttons(4) == 1% Press Y to move lego man into hazard zone and stop robots.
                        person_Coords = r.model.base*transl(0.7,0,-0.85);
                        delete(person_h);
                        person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                        disp('STOP: SENSOR INPUT');
                        pause(2); % Let button unpress.
                        eStopStatus = 0; % 0 - stopped, 1 - primed
                        while true
                            pause(0.1);
                            [axes, buttons, povs] = read(joy);
                            if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                                disp('RESTART PRIMED');
                                eStopStatus = 1;
                                pause(2); % Let button unpress.
                            elseif (buttons(1) == 1) && (eStopStatus == 1)
                                disp('RESTART ABORTED');
                                eStopStatus = 0;
                                pause(2);
                            elseif (buttons(2) == 1) && (eStopStatus == 1)
                                disp('RESTARTING');
                                person_Coords = r.model.base*transl(1.7,0,-0.85);
                                delete(person_h);
                                person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                                break;
                            end
                        end
                    end
                end
        if itemPickup > 0 %% If there's a drink attached to end effector, move it to end effector.
            if itemPickup == 1 % Cola
                % Delete instance, replot at origin, and transform to end
                % effector.
                delete(softDrinkCola_h);
                softDrinkCola_h = PlaceObject('Items\SoftDrink_Cola.ply'); 

                vertices = get(softDrinkCola_h, 'vertices');
                verticesAdjustTr = robot.model.fkine(qRMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
                transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                set(softDrinkCola_h, 'vertices', transformedVertices(:,1:3));
                drawnow();
            elseif itemPickup == 2 % Cutlery
                % Delete instance, replot at origin, and transform to end
                % effector.
                delete(cutlery_h);
                cutlery_h = PlaceObject('Items\CutleryContainer.ply'); 

                vertices = get(cutlery_h, 'vertices');
                verticesAdjustTr = robot.model.fkine(qRMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
                transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                set(cutlery_h, 'vertices', transformedVertices(:,1:3));
                drawnow();
             elseif itemPickup == 3 % Sauce
                % Delete instance, replot at origin, and transform to end
                % effector.
                delete(sauce_h);
                sauce_h = PlaceObject('Items\SauceContainer.ply'); 

                vertices = get(sauce_h, 'vertices');
                verticesAdjustTr = robot.model.fkine(qRMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
                transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                set(sauce_h, 'vertices', transformedVertices(:,1:3));
                drawnow();           
            end
        end
        if mealPickup == 1 % Pickup the meal with specified robot.
            % Delete instance, replot at origin, and transform to end
            % effector.
            delete(mealBox_Closed_h);
            mealBox_Closed_h = PlaceObject('Items\MealBox_Closed.ply'); 

            vertices = get(mealBox_Closed_h, 'vertices');
            verticesAdjustTr = robot.model.fkine(qRMatrix(i,:))*troty(pi)*transl(0,0,-0.15-0.2)*trotz(pi/2);
            transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
            set(mealBox_Closed_h, 'vertices', transformedVertices(:,1:3));
            drawnow();            
        end
        robot.model.animate(qRMatrix(i,:));
        pause(0.001);
    end 
    lastPose = qRMatrix(end,:);
end

%% Dobot Travel Method Outside of Class 
% Only for independent movement of the robot.
function DTravel(robot, q1, q2, steps, mode)
    global joy;
    global person_h;
    global r;
    global enableJoy;
    global mealPickup;
    global mealBox_Closed_h;
    global vertex;
    global faces;
    global faceNormals;
    global rect_h;


    % mode:
    %   1 - Trapezoidal
    %   2 - Quintic Polynomial

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
            % For each step, check controller input for two modes to stop
            % movement:
            % 1. E-Stop
            % 2. Sensing of personnel in hazard zone. 
            
                % Read controller input (Using XBONE configuration)
                if enableJoy
                    [axes, buttons, povs] = read(joy);
                
                
                    % Press A (button 1) to stop movement and enter infinite
                    % loop. Press A (button 1) to prime restart. Press B 
                    % (button 2) to restart movement.
                    if buttons(1) == 1 % STOP
                        disp('STOP: USER INPUT');
                        pause(2); % Let button unpress.
                        eStopStatus = 0; % 0 - stopped, 1 - primed
                        while true
                            pause(0.1);
                            [axes, buttons, povs] = read(joy);
                            if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                                disp('RESTART PRIMED');
                                eStopStatus = 1;
                                pause(2); % Let button unpress.
                            elseif (buttons(1) == 1) && (eStopStatus == 1)
                                disp('RESTART ABORTED');
                                eStopStatus = 0;
                                pause(2);
                            elseif (buttons(2) == 1) && (eStopStatus == 1)
                                disp('RESTARTING');
                                break;
                            end
                        end
                    elseif buttons(4) == 1% Press Y to move lego man into hazard zone and stop robots.
                        person_Coords = r.model.base*transl(0.7,0,-0.85);
                        delete(person_h);
                        person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                        disp('STOP: SENSOR INPUT');
                        pause(2); % Let button unpress.
                        eStopStatus = 0; % 0 - stopped, 1 - primed
                        while true
                            pause(0.1);
                            [axes, buttons, povs] = read(joy);
                            if (buttons(1) == 1) && (eStopStatus == 0) % Prime restart with A
                                disp('RESTART PRIMED');
                                eStopStatus = 1;
                                pause(2); % Let button unpress.
                            elseif (buttons(1) == 1) && (eStopStatus == 1)
                                disp('RESTART ABORTED');
                                eStopStatus = 0;
                                pause(2);
                            elseif (buttons(2) == 1) && (eStopStatus == 1)
                                disp('RESTARTING');
                                person_Coords = r.model.base*transl(1.7,0,-0.85);
                                delete(person_h);
                                person_h = PlaceObject('Items\lego man.ply', person_Coords(1:3,4)');
                                break;
                            end
                        end
                    end
                end
            if mealPickup == 1 % Pickup the meal with specified robot.
                % Delete instance, replot at origin, and transform to end
                % effector.
                delete(mealBox_Closed_h);
                mealBox_Closed_h = PlaceObject('Items\MealBox_Closed.ply'); 
    
                vertices = get(mealBox_Closed_h, 'vertices');
                verticesAdjustTr = robot.model.fkine(qMatrix(i,:))*transl(0,0,-0.23)*trotz(pi/2);
                transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                set(mealBox_Closed_h, 'vertices', transformedVertices(:,1:3));
                drawnow();            
            end
            % Check for collision with the human cube.

            dobotCollide = collisionCheck(robot, qMatrix, faces, vertex, faceNormals, true);
            while dobotCollide
                % Continually check collisionCheck for a collision.
                dobotCollide = collisionCheck(robot, qMatrix, faces, vertex, faceNormals, true);
                pause(0.1); % Allow for ui use.
                if enableJoy % turn off the obstacle with the controller.
                    [axes, buttons, povs] = read(joy);
                    if buttons(3) == 1 % IF X IS PRESSED, DELETE OBSTACLE
                        delete(rect_h);
                        faces = [];
                        vertex = [];
                        faceNormals = [];
                    end
                end
            end


            % If permitted to proceed, resume animating the robot.
            
           robot.model.animate(qMatrix(i,:));
           pause(0.01);
           drawnow;
       end
end

%% collisionCheck
% Given a robot model (robot) and trajectory (qMatrix), as well as triangle obstacles in the
% environment (faces, vertex, faceNormals)
function result = collisionCheck(robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
    if nargin < 6
        returnOnceFound = true;
    end
    result = false;

    for qIndex = 1:size(qMatrix, 1); % For each row/step in the qMatrix,
        % Get the transform of every joint (starting and ending transform
        % of every link)
        tr = GetLinkPoses(qMatrix(qIndex, :), robot);

        % Go through each link and also each triangle face
        for i=1:size(tr,3)-1
            for faceIndex = 1:size(faces, 1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if (check == 1) && (IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:)))
                    plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                    %display('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end
        end
    end
end

%% GetLinkPoses
% q - robot joint angles
% robot - SerialLink robot model
% transforms - list of transforms
function [transforms] = GetLinkPoses(q, robot)
    links = robot.model.links;
    transforms = zeros(4,4,length(links)+1);
    transforms(:,:,1) = robot.model.base;
    for i = 1:length(links)
        L = links(1,i);

        current_transform = transforms(:,:,i);

        % Apply DH parameters to make transforms along robot links.
        current_transform = current_transform * trotz(q(1,i) + L.offset) * transl(0,0,L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i+1) = current_transform;
    end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)
    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);
    D = uv * uv - uu * vv;
    % Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
        result = 0;
        return;
    end
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
        result = 0;
        return;
    end
    result = 1;                      % intersectP is in Triangle
end

%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end
