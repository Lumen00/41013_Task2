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
            self.model.plot(zeros(1,self.model.n), 'workspace', self.workspace, 'scale', 0.5);


            %self.GetLinearUR3Robot();
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
            L2 = Link('d',120.5/1000,'a',370.5/1000,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',pi/2); 

            L3 = Link('d',-97.5/1000,'a',295/1000,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 
            
            L4 = Link('d',97.5/1000,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]), 'offset',-pi/2); 
            L5 = Link('d',171/1000,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0); 
            L6 = Link('d',122.6/1000,'a',0,'alpha',0,'qlim',deg2rad([-180 180]), 'offset',0); 

            self.model = SerialLink([L1,L2,L3,L4,L5,L6],'name',name);

            
        end
    end

end