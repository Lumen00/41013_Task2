classdef RozumPulse75 < handle
    properties
        %> Robot model
        model;

        %> default workspace
        workspace = [-5 5 -5 5 0 5]./2;   

    end

    methods
        function self = RozumPulse75()

            
            self.GetRozumPulse75();
            %self.model.plot(zeros(1,self.model.n), 'workspace', self.workspace, 'scale', 0.3);


            self.PlotRozum();
            %self.Gripper();

            drawnow;
        end
        %%
        function GetRozumPulse75(self)

            name = ['RozumPulse75'];

            % theta - z rotation 
            % d - z translation
            % a - x translation
            % alpha - x rotation

            L1 = Link('d',232.5/1000,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0); 
            L2 = Link('d',120.5/1000,'a',375/1000,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',pi/2); 

            L3 = Link('d',-97.5/1000,'a',295/1000,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 
            
            L4 = Link('d',97.5/1000,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]), 'offset',-pi/2); 
            L5 = Link('d',171/1000,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0); 
            L6 = Link('d',122.6/1000,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 

            self.model = SerialLink([L1,L2,L3,L4,L5,L6],'name',name);

            self.model.base = eye(4);
        end
        %%
        function PlotRozum(self)
            for linkIndex = 0:self.model.n
                %display(['Link',num2str(linkIndex),'.ply']);
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'delay', 0, 'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end  
        %%
        function Travel(self, q1, q2, steps)

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

            % TO DO LIST:
            % - Add choice between trapezoidal, quintic polynomial, RMRC.
            % - Manipulability measure for singularities and DLS
            % - Move simultaneously with the other robot and all other
            % objects.
            % - Attach points for items and grippers.


            s = lspb(0,1,steps);    % Create matrix describing trapezoidal trajectory 
            %                           movement varying smoothly from S0 to SF (time)
            %                           in M steps. V can be specified, but is computed
            %                           automatically. 
            qMatrix = nan(steps,self.model.n);
            for i=1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end

            for i=1:steps
                %delete(eefPos_h)
                %eefPos_h = trplot(self.model.fkine(qMatrix(i,:)));

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
                        verticesAdjustTr = self.model.fkine(qMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
                        transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                        set(softDrinkCola_h, 'vertices', transformedVertices(:,1:3));
                        drawnow();
                     elseif itemPickup == 2 % Cutlery
                        % Delete instance, replot at origin, and transform to end
                        % effector.
                        delete(cutlery_h);
                        cutlery_h = PlaceObject('Items\CutleryContainer.ply'); 
        
                        vertices = get(cutlery_h, 'vertices');
                        verticesAdjustTr = self.model.fkine(qMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
                        transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                        set(cutlery_h, 'vertices', transformedVertices(:,1:3));
                        drawnow();
                    elseif itemPickup == 3 % Sauce
                        % Delete instance, replot at origin, and transform to end
                        % effector.
                        delete(sauce_h);
                        sauce_h = PlaceObject('Items\SauceContainer.ply'); 
        
                        vertices = get(sauce_h, 'vertices');
                        verticesAdjustTr = self.model.fkine(qMatrix(i,:))*troty(pi/2)*transl(-0.15,0,0);
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
                    verticesAdjustTr = self.model.fkine(qMatrix(i,:))*troty(pi)*transl(0,0,-0.15-0.2)*trotz(pi/2)
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * verticesAdjustTr'; 
                    set(mealBox_Closed_h, 'vertices', transformedVertices(:,1:3));
                    drawnow();            
                end
                self.model.animate(qMatrix(i,:));
               pause(0.01);
            end

        end
    end

end