classdef RozumPulse75 < handle
    properties
        %> Robot model
        model;

        %> default workspace
        workspace = [-5 5 -5 5 0 5]./5;   

    end

    methods
        function self = RozumPulse75()

            
            self.GetRozumPulse75();
            self.model.plot(zeros(1,self.model.n), 'workspace', self.workspace, 'scale', 0.3);


            self.PlotRozum();
            %self.PlotAndColourRobot();%robot,workspace);
            %self.Gripper();

            drawnow;
        end
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
    end

end