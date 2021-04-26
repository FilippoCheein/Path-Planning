classdef envClass < rl.env.MATLABEnvironment
    %ENVCLASS: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
%         constants for DQN
        wCol=-10;
        wGoal=10;
        wTime=0.2;
%         alpha=-20;
        alpha=-1;
        
%         constants for APF
        kAtt=1;
        kRep=20;
        rRep=3;
    end
    
    properties
        % Initialize system state [x,dx,theta,dtheta]'
        State=zeros(1,2);
        Goal=zeros(1,2);
        Obst=[];
        uField=zeros(100,100);
        uFieldPad=zeros(108,108);
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone=false;
        newPlot=false;
        Figure
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = envClass()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([9 9]);
            ObservationInfo.Name='Robot Position (y,x)';
            
            % Initialize Action settings   
            actions={[0 0],[-1 0],[-1 1],[0 1],[1 1],[1 0],[1 -1],[0 -1],[-1 -1]};
            ActionInfo=rlFiniteSetSpec(actions);
            ActionInfo.Name='Chosen [y,x] Velocity';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            
            %     previous state and current location's potential value
            prevState=this.State;
            uPrev=this.uField(prevState(1),prevState(2));

        %     apply the given action to get nextState
            nextState=round(prevState+Action);
            this.State=nextState;

        %     check if outside of 100x100 environment
            wallC=min(this.State)<1||max(this.State)>100;
        %     check if inside any obstacle
            obsC=false;
            for n=1:size(this.Obst,1)
                currObsC=norm(this.State-this.Obst(n,:))<=0.5;
                obsC=obsC||currObsC;
            end
        %     both of those two are collision checks
            IsCol=obsC||wallC;
        %     if we collided, then zero out the next observation cause we're gonna
        %     terminate this run, and repeat single pt potential to not influence
        %     reward function
            if IsCol
                uCurr=uPrev;
                Observation=zeros(9);
            else
        %         otherwise do normal single pt potential and observation
        %         calculations
                uCurr=this.uField(this.State(1),this.State(2));
                Observation=subsetBounded(this.uFieldPad,nextState,4);
            end
        %     check if at goal, then done
            IsGoal=norm(this.State-this.Goal)<=0.5;
        %     reward for change of the potential field
            wMove=this.alpha*(uCurr-uPrev);
        %     apply reward
        %     make sure not to give goal reward when "done" via collision
            Reward=this.wTime+wMove+this.wGoal*(IsGoal&&~IsCol)+this.wCol*(IsCol);
        %     mark run as done in the case of success or failure via collision
            IsDone=IsGoal||IsCol;
            this.IsDone=IsDone;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
        %     Choose # of obstacles randomly from 1-10
            nObst=randi(10);
        %     Generate Y and X co-ords of obstacle(s), goal, and start point
        %     randomly, with all Y,X pairs being unique
            yVec=randperm(100,2+nObst);
            xVec=randperm(100,2+nObst);
        %     set the state goal and obstacle's pairs
            init=[yVec(1),xVec(1)];
            this.State=init;
            this.Goal=[yVec(2),xVec(2)];
            this.Obst=[yVec(3:end).',xVec(3:end).'];
        %     use non-padding coords to potential function to make uField
            this.uField=coordsToU(this);
        %     use padding coords to potential fucntion to then use for observation
        %     bounding box
            rPad=4;
            this.uFieldPad=coordsToUPad(this,rPad);
        %     for observation use padded field w/ subsetBounded
            InitialObservation=subsetBounded(this.uFieldPad,init,rPad);

            this.newPlot=true;
                
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        function uField=coordsToU(this)
        %     uField=coordsToU(this,figN)
        %     inputs:
        %       this.State=[y,x] co-ords of robot
        %       this.Goal=[y,x] co-ords of goal
        %       this.Obst=Nx2 matrix where the n-th column pair [y,x]
        %       represents the n-th obstacle, with N = number of obstacles
        %       figN=zero for no figure plotting, otherwise specify figure number
        %     outputs:
        %         uField=potential field array

        %     generate x and y matrix variables
            [x,y]=meshgrid(1:100,1:100);

        %     calculate distance from every point to the goal
            yGoal=y-this.Goal(1);
            xGoal=x-this.Goal(2);

        %     calculate attractive potential based on above distance
            uAtt=0.5*this.kAtt*(yGoal.^2+xGoal.^2);

        %     pre-allocate repulsive field so MATLAB doesn't yell at me
            uRep=zeros(100);
            for n=1:size(this.Obst,1)
        %         calculate distance from every point to n-th obstacle
                currY=y-this.Obst(n,1);
                currX=x-this.Obst(n,2);
                currDist=sqrt(currY.^2+currX.^2);
        %         calculate n-th obstacle's potential repulsive field
                currRep=0.5*this.kRep*(1./(currDist+0.05)-1/this.rRep).^2;
        %         add up all with a step function for domain'ing
                uRep=uRep+currRep.*(currDist<=this.rRep);
            end

        %     combine attractive and repulsive for net field
            uField=uAtt+uRep;
        end

        function uField=coordsToUPad(this,rPad)
        %     uField=coordsToUPad(thism,rPad,figN)
        %     inputs:
        %       this.State=[y,x] co-ords of robot
        %       this.Goal=[y,x] co-ords of goal
        %       this.Obst=Nx2 matrix where the n-th column pair [y,x]
        %       represents the n-th obstacle, with N = number of obstacles
        %       rPad=how many pixels to pad (extra calculate) uField by
        %       figN=zero for no figure plotting, otherwise specify figure number
        %     outputs:
        %         uField=potential field array

        %     run x and y from +/- rPad so that offsets are nice
            [x,y]=meshgrid(1-rPad:100+rPad,1-rPad:100+rPad);

        %     calculate distance from every point to the goal
            yGoal=y-this.Goal(1);
            xGoal=x-this.Goal(2);

        %     calculate attractive potential based on above distance
            uAtt=0.5*this.kAtt*(yGoal.^2+xGoal.^2);

        %     pre-allocate repulsive field so MATLAB doesn't yell at me
            uRep=zeros(100+2*rPad);
            for n=1:size(this.Obst,1)
        %         calculate distance from every point to n-th obstacle
                currY=y-this.Obst(n,1);
                currX=x-this.Obst(n,2);
                currDist=sqrt(currY.^2+currX.^2);

        %         calculate n-th obstacle's potential repulsive field
                currRep=0.5*this.kRep*(1./(currDist+0.05)-1/this.rRep).^2;
        %         add up all with a step function for domain'ing
                uRep=uRep+currRep.*(currDist<=this.rRep);
            end

        %     combine attractive and repulsive for net field
            uField=uAtt+uRep;
        end
        
        function subset=subsetBounded(unbounded,pos,radius)
        %     subset=subsetBounded(unbounded,pos,radius)
        %     inputs:
        %       unbounded=padded potential field
        %       pos=center point of bounds, no co-ord translation needed
        %       radius=radius of bounding box
        %     outputs:
        %       subset=bounded potential field, with padding

        %     calculate un-shifted bounding box
            yLower=pos(1)-radius;
            yUpper=pos(1)+radius;
            xLower=pos(2)-radius;
            xUpper=pos(2)+radius;

        %     shift bounding box due to padding of potential field
            yRange=yLower+4:yUpper+4;
            xRange=xLower+4:xUpper+4;
        %     crop
            subset=unbounded(yRange,xRange);
        end
        
        % (optional) Visualization method
        function plot(this)
            % Initiate the visualization
            this.Figure = figure('Visible','on');
            ha = gca(this.Figure);
            ha.XLimMode = 'manual';
            ha.YLimMode = 'manual';
            ha.XLim = [0 100];
            ha.YLim = [0 100];
            ha.XLabel.String='x axis';
            ha.YLabel.String='y axis';
            hold(ha,'on');
            shading(ha,'interp')
            colormap(ha,'parula')
            view(ha,-20,20)
            
            % Update the visualization
            envUpdatedCallback(this)
            
        end
        
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
            if ~isempty(this.Figure) && isvalid(this.Figure)
                % Set visualization figure as the current figure
                ha = gca(this.Figure);
                
                if this.newPlot
                    prevSurf=findobj(ha,'type','surface');
                    prevLine=findobj(ha,'type','line');
                    prevAnim=findobj(ha,'type','AnimatedLine');
                    
                    delete(prevSurf)
                    delete(prevLine)
                    delete(prevAnim)
                    
                    mesh(ha,this.uField);

                    startZ=this.uField(this.State(1),this.State(2));
        %             mark the start point as a magenta circle
                    plot3(ha,this.State(2),this.State(1),startZ...
                        ,'wo','MarkerSize',15,'MarkerFaceColor','m');


                    this.uField(this.Goal(1),this.Goal(2))
        %             mark the end point as a red diamond
                    endZ=this.uField(this.Goal(1),this.Goal(2));
                    plot3(ha,this.Goal(2),this.Goal(1),endZ...
                        ,'wd','MarkerSize',15,'MarkerFaceColor','r');
                    this.newPlot=false;
                end
                
                an=animatedline(ha,'Marker','o','MarkerEdgeColor','w','MarkerSize',15,'MarkerFaceColor','g');
                
%                 if max(this.State)<100&&min(this.State)>1
                if ~this.IsDone
                    startZ=this.uField(this.State(1),this.State(2));
%                     mark the start point as a green circle
                    addpoints(an,this.State(2),this.State(1),startZ);
                    drawnow
                end
            end
        end
    end
end
