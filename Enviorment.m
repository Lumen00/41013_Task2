clf;
clc;
%% Load the food

% set(0, 'DefaultFigureWindowStyle', 'docked')

[f,v,data] = plyread('Donuts.ply','tri');

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

hold on;

    for zOffset = [0.08]
    for yOffset = [1]
        for xOffset = [1]
        trisurf(f,v(:,1) + xOffset,v(:,2) + yOffset, v(:,3) + zOffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
    end
    camlight;
hold on; 
%% Load the drink

% set(0, 'DefaultFigureWindowStyle', 'docked')

[f,v,data] = plyread('SoftDrink_Cola.ply','tri');

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

hold on;

    for zOffset = [0.08]
    for yOffset = [1.5]
        for xOffset = [1.5]
        trisurf(f,v(:,1) + xOffset,v(:,2) + yOffset, v(:,3) + zOffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
    end
hold on; 

%% Load the counter

% set(0, 'DefaultFigureWindowStyle', 'docked')

[f,v,data] = plyread('Counter.ply','tri');

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

hold on;

    for zOffset = [0.08]
    for yOffset = [1]
        for xOffset = [1]
        trisurf(f,v(:,1) + xOffset,v(:,2) + yOffset, v(:,3) + zOffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
    end
    camlight;
hold on; 
%% Load the stop button

% set(0, 'DefaultFigureWindowStyle', 'docked')

[f,v,data] = plyread('Emergency_Stop.ply','tri');

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

hold on;

    for zOffset = [0.08]
    for yOffset = [1]
        for xOffset = [0.5]
        trisurf(f,v(:,1) + xOffset,v(:,2) + yOffset, v(:,3) + zOffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
    end
    camlight;
hold on; 
%% Create UR3
workspace = [0 2 0.08 1.5 0.05 2];
scale = 0.2;
base_ur3 = transl(1,1,1);
q = zeros(1,6);


%value change from UR3 file
 L1 = Link('d',0.152,'a',0,'alpha',pi/2,'qlim',[-3.1404 3.1415], 'offset', 0);
            L2 = Link('d',0.12,'a',-0.244,'alpha',0,'qlim',[-3.1406 -5.9120e-04] , 'offset',-pi/2); 
            L3 = Link('d',-0.093,'a',-0.213,'alpha',0,'qlim',[-1.5704 1.5703] , 'offset', 0);
            L4 = Link('d',0.083,'a',0,'alpha',pi/2,'qlim',[-3.1400 3.1405],'offset', 0); 
            L5 = Link('d',0.083,'a',0,'alpha',-pi/2,'qlim',[-2.3562 2.3554], 'offset',-pi/2);
            L6 = Link('d',0.082,'a',0,'alpha',0,'qlim',[-3.1408 3.1383], 'offset', 0);

UR3 = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3','base',base_ur3);

UR3.plot(q,'workspace',workspace,'scale',scale);



