classdef base_model_routing_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        StartSimulationButton        matlab.ui.control.Button
        NumberofNodesEditFieldLabel  matlab.ui.control.Label
        NumberofNodesEditField       matlab.ui.control.NumericEditField
        CreateNodesButton            matlab.ui.control.Button
        ExecFunctionCommandWindowButton  matlab.ui.control.Button
        MoveAllNodesRandomlyButton   matlab.ui.control.Button
        MoveSingleRandomNodeButton   matlab.ui.control.Button
        Select2randomnodesandfindpathButton  matlab.ui.control.Button
        UIAxes                       matlab.ui.control.UIAxes
    end

    
    % Global variables, call them by prepending 'app.' in front of them
    properties (Access = private)
        NodesLocationArray % Description
        AdjacencyMatrix
        SimulationSpeed = 0.05 % Speed at which the signal moves in terms of seconds
        plothandles
        N %numebr of nodes
        
    end
    
    methods (Access = private)
        
        % This is a custom function that you can use
        function customFunction(app, par1)
            disp(['The simulation speed is set at: ', num2str(app.SimulationSpeed), ' seconds per frame'])
            
            % Variables are not stored in the workspace by default (can use debug mode to show it)
            % Or use the function assignin (should declare it locally first before assignin works)            
            if (par1 == true)
                simspeed = app.SimulationSpeed;            
                assignin('base', 'savedVar_simspeed', simspeed);
            end
        end
        
        function redraw(app)
             cla(app.UIAxes)
             plot(app.UIAxes, app.NodesLocationArray(:,1), app.NodesLocationArray(:,2), 'ok', 'MarkerfaceColor', 'k');
             for i = 1:size(app.NodesLocationArray, 1)    
                for j = (i+1):size(app.NodesLocationArray, 1)
                    if(app.AdjacencyMatrix(i,j) > 0)
                        line(app.UIAxes, [app.NodesLocationArray(i,1) app.NodesLocationArray(j,1)], [app.NodesLocationArray(i,2) app.NodesLocationArray(j,2)], 'Color', 'k');
                    end                    
                end
             end
        end
        function redrawRefactor(app, txNodeIndex, rxNodeIndex,highlight)
             cla(app.UIAxes)
             for i=1:size(app.NodesLocationArray, 1)
                    if(i~=txNodeIndex & i~=rxNodeIndex)
                        plot(app.UIAxes, app.NodesLocationArray(i,1), app.NodesLocationArray(i,2), 'ok', 'MarkerfaceColor', 'k');
                    end
             end
             if(highlight==true)
                plot(app.UIAxes, app.NodesLocationArray(txNodeIndex,1), app.NodesLocationArray(txNodeIndex,2), 'ok', 'MarkerfaceColor', 'red');
                plot(app.UIAxes, app.NodesLocationArray(rxNodeIndex,1), app.NodesLocationArray(rxNodeIndex,2), 'ok', 'MarkerfaceColor', 'red');
             else
                plot(app.UIAxes, app.NodesLocationArray(txNodeIndex,1), app.NodesLocationArray(txNodeIndex,2), 'ok', 'MarkerfaceColor', 'k');
                plot(app.UIAxes, app.NodesLocationArray(rxNodeIndex,1), app.NodesLocationArray(rxNodeIndex,2), 'ok', 'MarkerfaceColor', 'k');
             end
             
             for i = 1:size(app.NodesLocationArray, 1)    
                for j = (i+1):size(app.NodesLocationArray, 1)
                    if(app.AdjacencyMatrix(i,j) > 0)
                        line(app.UIAxes, [app.NodesLocationArray(i,1) app.NodesLocationArray(j,1)], [app.NodesLocationArray(i,2) app.NodesLocationArray(j,2)], 'Color', 'k');
                    end                    
                end
             end
        end
        
        
        function updateWeightsAfterMoving(app,nodeToMove,delta)
            for index=1:app.N
                if(app.AdjacencyMatrix(nodeToMove,index)~=0)
                    app.AdjacencyMatrix(nodeToMove,index)= app.AdjacencyMatrix(nodeToMove,index)+delta;
                    app.AdjacencyMatrix(index,nodeToMove)=app.AdjacencyMatrix(nodeToMove,index);
                end
            end
            
            
        end
        
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Input stuff that you want to execute at startup
            disp('App has started up!') % Will display on the main MATLAB command window
            addpath('./dijkstra/');
            xlim(app.UIAxes, [0 1]);
            ylim(app.UIAxes, [0 1]);
        end

        % Button pushed function: CreateNodesButton
        function CreateNodesButtonPushed(app, event)
            cla(app.UIAxes) % Clears the whole axes when new nodes are created
            app.N = app.NumberofNodesEditField.Value; % Number of nodes, input from user
            
            app.NodesLocationArray = rand(app.N,2); % Creates random N x 2 array [2: (X,Y) values]
            app.AdjacencyMatrix = zeros(app.N,app.N);
            % Plot nodes as small circles
            plot(app.UIAxes, app.NodesLocationArray(:,1), app.NodesLocationArray(:,2), 'ok', 'MarkerfaceColor', 'k');
        end

        % Button pushed function: StartSimulationButton
        function StartSimulationButtonPushed(app, event)
            hold(app.UIAxes, 'on') % Ensures all previous plots (nodes) will remain on the axes
            
            % Create a complete network
            % [x] This should be changed as the network should contain some 5/6 hops
            for i = 1:size(app.NodesLocationArray, 1)    
                for j = 1:size(app.NodesLocationArray, 1)
                    if(i ~= j)
                        randomNumber = rand*size(app.NodesLocationArray, 1);
                        if(j<randomNumber/2)
                        line(app.UIAxes, [app.NodesLocationArray(i,1) app.NodesLocationArray(j,1)], [app.NodesLocationArray(i,2) app.NodesLocationArray(j,2)], 'Color', 'k');
                        edgeWeight = round(rand * 9) + 1;   % Make sure edge weight is from 1 to 10(this way edge wait won't get rounded to 0)
                        app.AdjacencyMatrix(i,j) = edgeWeight;
                        app.AdjacencyMatrix(j,i) = edgeWeight;
                        end
                    end    
%                     pause(app.SimulationSpeed) % Remove this if you want it instantly shown
                end                
            end
            disp(app.AdjacencyMatrix);
        end

        % Button pushed function: ExecFunctionCommandWindowButton
        function ExecFunctionCommandWindowButtonPushed(app, event)
            customFunction(app, true); % Executes our custom function
        end

        % Button pushed function: MoveAllNodesRandomlyButton
        function moveAllNodesRandomly(app, event)

            for k=1:5 
                app.N = app.NumberofNodesEditField.Value;
                app.NodesLocationArray = rand(app.N,2); 
                redraw(app)
                pause(1) % delay of 1s
%               TODO: Make links here using routing protocol
            end
        end

        % Button pushed function: MoveSingleRandomNodeButton
        function moveSingleRandomNode(app, event)
            % this function assumes that a graph with links is already
            % there and we move one randomly selected node by one step
            % modify one random point in graph(move)
            
            nodeToMove=randi(size(app.NodesLocationArray, 1));%select random node
            
            
            stepType=rand>0.5; %randmoly select +ve or -ve step
            direction=rand>0.5; %randomly select X or Y direction
            disp("adjanceny matrix before moving")
            disp(app.AdjacencyMatrix)
            %process step with edge case handling
            if(stepType & direction)
                if(app.NodesLocationArray(nodeToMove,1)+0.1<1)
                    app.NodesLocationArray(nodeToMove,1)=app.NodesLocationArray(nodeToMove,1)+0.1;
                    updateWeightsAfterMoving(app,nodeToMove,1)
                end
            elseif(stepType & not(direction))
                if(app.NodesLocationArray(nodeToMove,2)+0.1<1)
                    app.NodesLocationArray(nodeToMove,2)=app.NodesLocationArray(nodeToMove,2)+0.1;
                    updateWeightsAfterMoving(app,nodeToMove,1)
                end
            elseif(not(stepType) & direction)
                if(app.NodesLocationArray(nodeToMove,1)-0.1>0)
                    app.NodesLocationArray(nodeToMove,1)=app.NodesLocationArray(nodeToMove,1)-0.1;
                    updateWeightsAfterMoving(app,nodeToMove,-1)
                end
            elseif(not(stepType) & not(direction))
                if(app.NodesLocationArray(nodeToMove,2)-0.1>0)
                    app.NodesLocationArray(nodeToMove,2)=app.NodesLocationArray(nodeToMove,2)-0.1;
                    updateWeightsAfterMoving(app,nodeToMove,-1)
                end
            end
            
           
            disp("adjanceny matrix after moving")
            disp(app.AdjacencyMatrix)
            
            redraw(app) %plot it again
           
            
            
        end

        % Button pushed function: 
        % Select2randomnodesandfindpathButton
        function testing(app, event)
           
              txNodeIndex=randi(size(app.NodesLocationArray, 1))
              
              rxNodeIndex=randi(size(app.NodesLocationArray, 1))
              
              redrawRefactor(app,txNodeIndex,rxNodeIndex,true)
              
              pause(1) % delay of 1s
              %find path using DSDV or DJIKSTRA or bellman here
              %redraw graph to highlight that path
              
              disp(app.AdjacencyMatrix)
              
              [cost,path]=dijkstra(app.AdjacencyMatrix,txNodeIndex,rxNodeIndex)
             
              
               
            
            
           
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 939 590];
            app.UIFigure.Name = 'UI Figure';

            % Create StartSimulationButton
            app.StartSimulationButton = uibutton(app.UIFigure, 'push');
            app.StartSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @StartSimulationButtonPushed, true);
            app.StartSimulationButton.Position = [26 446 100 22];
            app.StartSimulationButton.Text = 'Start Simulation';

            % Create NumberofNodesEditFieldLabel
            app.NumberofNodesEditFieldLabel = uilabel(app.UIFigure);
            app.NumberofNodesEditFieldLabel.HorizontalAlignment = 'right';
            app.NumberofNodesEditFieldLabel.Position = [26 528 100 22];
            app.NumberofNodesEditFieldLabel.Text = 'Number of Nodes';

            % Create NumberofNodesEditField
            app.NumberofNodesEditField = uieditfield(app.UIFigure, 'numeric');
            app.NumberofNodesEditField.Position = [141 528 100 22];
            app.NumberofNodesEditField.Value = 10;

            % Create CreateNodesButton
            app.CreateNodesButton = uibutton(app.UIFigure, 'push');
            app.CreateNodesButton.ButtonPushedFcn = createCallbackFcn(app, @CreateNodesButtonPushed, true);
            app.CreateNodesButton.Position = [26 483 100 22];
            app.CreateNodesButton.Text = 'Create Nodes';

            % Create ExecFunctionCommandWindowButton
            app.ExecFunctionCommandWindowButton = uibutton(app.UIFigure, 'push');
            app.ExecFunctionCommandWindowButton.ButtonPushedFcn = createCallbackFcn(app, @ExecFunctionCommandWindowButtonPushed, true);
            app.ExecFunctionCommandWindowButton.Position = [27 383 100 50];
            app.ExecFunctionCommandWindowButton.Text = {'Exec Function'; 'Command'; 'Window'};

            % Create MoveAllNodesRandomlyButton
            app.MoveAllNodesRandomlyButton = uibutton(app.UIFigure, 'push');
            app.MoveAllNodesRandomlyButton.ButtonPushedFcn = createCallbackFcn(app, @moveAllNodesRandomly, true);
            app.MoveAllNodesRandomlyButton.VerticalAlignment = 'top';
            app.MoveAllNodesRandomlyButton.BackgroundColor = [0 1 0];
            app.MoveAllNodesRandomlyButton.Position = [16 296 151 22];
            app.MoveAllNodesRandomlyButton.Text = 'Move All Nodes Randomly';

            % Create MoveSingleRandomNodeButton
            app.MoveSingleRandomNodeButton = uibutton(app.UIFigure, 'push');
            app.MoveSingleRandomNodeButton.ButtonPushedFcn = createCallbackFcn(app, @moveSingleRandomNode, true);
            app.MoveSingleRandomNodeButton.BackgroundColor = [0 0 1];
            app.MoveSingleRandomNodeButton.FontColor = [1 1 1];
            app.MoveSingleRandomNodeButton.Position = [16 333 163 22];
            app.MoveSingleRandomNodeButton.Text = 'Move Single Random Node';

            % Create Select2randomnodesandfindpathButton
            app.Select2randomnodesandfindpathButton = uibutton(app.UIFigure, 'push');
            app.Select2randomnodesandfindpathButton.ButtonPushedFcn = createCallbackFcn(app, @testing, true);
            app.Select2randomnodesandfindpathButton.Position = [16 264 225 22];
            app.Select2randomnodesandfindpathButton.Text = 'Select 2 random nodes and find path';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, {'Routing protocol '; 'simulation'})
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.Position = [366 33 549 517];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = base_model_routing_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end