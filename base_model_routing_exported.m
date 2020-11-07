classdef base_model_routing_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        CreateWeightedGraphButton    matlab.ui.control.Button
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
        DVR
        SimulationSpeed = 0.05 % Speed at which the signal moves in terms of seconds
        plothandles
        N %numebr of nodes
        txNodeIndex
        rxNodeIndex
        highlight=false
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
                    if(app.AdjacencyMatrix(i,j) > 0)    %TODO: change edge weights when redrawing
                        line(app.UIAxes, [app.NodesLocationArray(i,1) app.NodesLocationArray(j,1)], [app.NodesLocationArray(i,2) app.NodesLocationArray(j,2)], 'Color', 'k');
                    end                    
                end
             end
        end
        function redrawRefactor(app)
             cla(app.UIAxes)
             txNodeIndex=app.txNodeIndex;
             rxNodeIndex=app.rxNodeIndex;
             highlight=app.highlight;

             for i=1:size(app.NodesLocationArray, 1)
%                     if(i~=txNodeIndex & i~=rxNodeIndex)
                        plot(app.UIAxes, app.NodesLocationArray(i,1), app.NodesLocationArray(i,2), 'ok', 'MarkerfaceColor', 'k');
%                     end
             end
             if(highlight==true)
                plot(app.UIAxes, app.NodesLocationArray(txNodeIndex,1), app.NodesLocationArray(txNodeIndex,2), 'ok', 'MarkerfaceColor', 'red');
                plot(app.UIAxes, app.NodesLocationArray(rxNodeIndex,1), app.NodesLocationArray(rxNodeIndex,2), 'ok', 'MarkerfaceColor', 'red');
%              else
%                 plot(app.UIAxes, app.NodesLocationArray(txNodeIndex,1), app.NodesLocationArray(txNodeIndex,2), 'ok', 'MarkerfaceColor', 'k');
%                 plot(app.UIAxes, app.NodesLocationArray(rxNodeIndex,1), app.NodesLocationArray(rxNodeIndex,2), 'ok', 'MarkerfaceColor', 'k');
             end

             for i = 1:size(app.NodesLocationArray, 1)
                for j = (i+1):size(app.NodesLocationArray, 1)
                    if(app.AdjacencyMatrix(i,j) > 0)
                        line(app.UIAxes, [app.NodesLocationArray(i,1) app.NodesLocationArray(j,1)], [app.NodesLocationArray(i,2) app.NodesLocationArray(j,2)], 'Color', 'k');
                    end
                end
             end

             if(app.highlight)
                %find path using DVR here
                %[cost,path]=dijkstra(app.AdjacencyMatrix,app.txNodeIndex,app.rxNodeIndex)
                currentNode = app.txNodeIndex;
                route(1) = currentNode;
                j=2;
                while(currentNode~=app.rxNodeIndex)
                    [row,col]=find(app.DVR(app.rxNodeIndex,:,currentNode)==min(app.DVR(app.rxNodeIndex,:,currentNode)));
                    route(j)=col;
                    currentNode=col;
                    j=j+1;
                end
                k=1:j-1;
                disp("Route: ")
                disp(route(k))
                highlightPath(app,route)
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
        
        function highlightPath(app, path)
            for nodeIndex=1:length(path)-1
            line(app.UIAxes, [app.NodesLocationArray(path(nodeIndex),1) app.NodesLocationArray(path(nodeIndex+1),1)], [app.NodesLocationArray(path(nodeIndex),2) app.NodesLocationArray(path(nodeIndex+1),2)], 'Color', 'blue','LineWidth',4);
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
            app.highlight=false
            app.N = app.NumberofNodesEditField.Value; % Number of nodes, input from user

            app.NodesLocationArray = rand(app.N,2); % Creates random N x 2 array [2: (X,Y) values]
            app.AdjacencyMatrix = zeros(app.N,app.N);
            % Plot nodes as small circles
            plot(app.UIAxes, app.NodesLocationArray(:,1), app.NodesLocationArray(:,2), 'ok', 'MarkerfaceColor', 'k');
        end

        % Button pushed function: CreateWeightedGraphButton
        function CreateWeightedGraphButtonPushed(app, event)
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
            
            for source = 1:size(app.NodesLocationArray, 1)  %Initialise the DVR matrix
                for node = 1:size(app.NodesLocationArray, 1)
                    for destination = 1:size(app.NodesLocationArray, 1)
                        if(source ~= node && source ~= destination)
                            if(node == destination && app.AdjacencyMatrix(source,destination) ~= 0)
                                app.DVR(destination, node, source) = app.AdjacencyMatrix(source, destination);  %If neighbour is destination, add weight from adjacencyMatrix to DVR matrix
                            else
                                app.DVR(destination, node, source) = 10 * app.N;  %If destination is not neighbour, set distance to 10 * #nodes
                            end
                        else
                            app.DVR(destination, node, source) = inf;   %If source == destination or next node, assign infinity
                        end
                    end
                end
            end
            %disp("DVR Matrix: ")
            %disp(app.DVR)
            previousDVR = zeros(size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1)); %Initialise previous DVR matrix, in order to keep updating the DVR matrix, until it doesn't change anymore
            while(previousDVR ~= app.DVR)
                previousDVR = app.DVR;
                minDist = zeros(size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1));  %Variable used to safe the distance of the path we are checking
                for source = 1:size(app.NodesLocationArray, 1)
                    for destination = 1:size(app.NodesLocationArray, 1)
                        if(source ~= destination && app.AdjacencyMatrix(source, destination) ~= 0)
                            for a = 1:size(app.NodesLocationArray, 1)
                                for b = 1:size(app.NodesLocationArray, 1)
                                   minDist(a,b) = app.AdjacencyMatrix(source, destination) + min(app.DVR(b, :, destination));
                                   if(minDist(a,b) < app.DVR(b, destination, source) && app.DVR(b, destination, source) < inf)  %If the calculated distance is lower than previous recorded distance for that path, and not infinity, update the distance for path in DVR
                                       app.DVR(b, destination, source) = minDist(a,b);
                                   end
                                end
                            end
                        end
                    end
                end
            end
            disp("DVR Matrix: ")
            disp(app.DVR)
        end

        % Button pushed function: ExecFunctionCommandWindowButton
        function ExecFunctionCommandWindowButtonPushed(app, event)
            %UNUSED: REMOVE
            customFunction(app, true); % Executes our custom function
        end

        % Button pushed function: MoveAllNodesRandomlyButton
        function moveAllNodesRandomly(app, event)

            for k=1:5 
                N = app.NumberofNodesEditField.Value;
                app.NodesLocationArray = rand(N,2); 
                redraw(app)
                pause(1) % delay of 1s
%               TODO: Make links here using routing protocol
                for source = 1:size(app.NodesLocationArray, 1)
                    for node = 1:size(app.NodesLocationArray, 1)
                        for destination = 1:size(app.NodesLocationArray, 1)
                            if(source ~= node && source ~= destination)
                                if(node == destination && app.AdjacencyMatrix(source,destination) ~= 0)
                                    
                                end
                            end
                        end
                    end
                end
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
            
            for source = 1:size(app.NodesLocationArray, 1)  %Initialise the DVR matrix
                for node = 1:size(app.NodesLocationArray, 1)
                    for destination = 1:size(app.NodesLocationArray, 1)
                        if(source ~= node && source ~= destination)
                            if(node == destination && app.AdjacencyMatrix(source,destination) ~= 0)
                                app.DVR(destination, node, source) = app.AdjacencyMatrix(source, destination);  %If neighbour is destination, add weight from adjacencyMatrix to DVR matrix
                            else
                                app.DVR(destination, node, source) = 10 * app.N;  %If destination is not neighbour, set distance to 10 * #nodes
                            end
                        else
                            app.DVR(destination, node, source) = inf;   %If source == destination or next node, assign infinity
                        end
                    end
                end
            end
            previousDVR = zeros(size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1)); %Initialise previous DVR matrix, in order to keep updating the DVR matrix, until it doesn't change anymore
            while(previousDVR ~= app.DVR)
                previousDVR = app.DVR;
                minDist = zeros(size(app.NodesLocationArray, 1), size(app.NodesLocationArray, 1));  %Variable used to safe the distance of the path we are checking
                for source = 1:size(app.NodesLocationArray, 1)
                    for destination = 1:size(app.NodesLocationArray, 1)
                        if(source ~= destination && app.AdjacencyMatrix(source, destination) ~= 0)
                            for a = 1:size(app.NodesLocationArray, 1)
                                for b = 1:size(app.NodesLocationArray, 1)
                                   minDist(a,b) = app.AdjacencyMatrix(source, destination) + min(app.DVR(b, :, destination));
                                   if(minDist(a,b) < app.DVR(b, destination, source) && app.DVR(b, destination, source) < inf)  %If the calculated distance is lower than previous recorded distance for that path, and not infinity, update the distance for path in DVR
                                       app.DVR(b, destination, source) = minDist(a,b);
                                   end
                                end
                            end
                        end
                    end
                end
            end
            disp("DVR Matrix: ")
            disp(app.DVR)
            
            redrawRefactor(app) %plot it again
           
            
            
        end

        % Button pushed function: 
        % Select2randomnodesandfindpathButton
        function testing(app, event)
            
              app.txNodeIndex=randi(size(app.NodesLocationArray, 1))
              app.rxNodeIndex=randi(size(app.NodesLocationArray, 1))
              app.highlight=true;
              redrawRefactor(app)

              %disp(app.AdjacencyMatrix)
              %[cost,path]=dijkstra(app.AdjacencyMatrix,app.txNodeIndex,app.rxNodeIndex)
              %disp("Dijkstra: ")
              %disp(path)
                                  
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

            % Create CreateWeightedGraphButton
            app.CreateWeightedGraphButton = uibutton(app.UIFigure, 'push');
            app.CreateWeightedGraphButton.ButtonPushedFcn = createCallbackFcn(app, @CreateWeightedGraphButtonPushed, true);
            app.CreateWeightedGraphButton.Position = [66 443 155 22];
            app.CreateWeightedGraphButton.Text = '2. Create Weighted Graph';

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
            app.CreateNodesButton.Position = [82 483 103 22];
            app.CreateNodesButton.Text = '1. Create Nodes';

            % Create ExecFunctionCommandWindowButton
            app.ExecFunctionCommandWindowButton = uibutton(app.UIFigure, 'push');
            app.ExecFunctionCommandWindowButton.ButtonPushedFcn = createCallbackFcn(app, @ExecFunctionCommandWindowButtonPushed, true);
            app.ExecFunctionCommandWindowButton.Position = [16 104 100 50];
            app.ExecFunctionCommandWindowButton.Text = {'Exec Function'; 'Command'; 'Window'};

            % Create MoveAllNodesRandomlyButton
            app.MoveAllNodesRandomlyButton = uibutton(app.UIFigure, 'push');
            app.MoveAllNodesRandomlyButton.ButtonPushedFcn = createCallbackFcn(app, @moveAllNodesRandomly, true);
            app.MoveAllNodesRandomlyButton.VerticalAlignment = 'top';
            app.MoveAllNodesRandomlyButton.BackgroundColor = [1 1 1];
            app.MoveAllNodesRandomlyButton.Position = [16 168 151 22];
            app.MoveAllNodesRandomlyButton.Text = 'Move All Nodes Randomly';

            % Create MoveSingleRandomNodeButton
            app.MoveSingleRandomNodeButton = uibutton(app.UIFigure, 'push');
            app.MoveSingleRandomNodeButton.ButtonPushedFcn = createCallbackFcn(app, @moveSingleRandomNode, true);
            app.MoveSingleRandomNodeButton.BackgroundColor = [0 0 1];
            app.MoveSingleRandomNodeButton.FontColor = [1 1 1];
            app.MoveSingleRandomNodeButton.Position = [58 375 163 22];
            app.MoveSingleRandomNodeButton.Text = 'Move Single Random Node';

            % Create Select2randomnodesandfindpathButton
            app.Select2randomnodesandfindpathButton = uibutton(app.UIFigure, 'push');
            app.Select2randomnodesandfindpathButton.ButtonPushedFcn = createCallbackFcn(app, @testing, true);
            app.Select2randomnodesandfindpathButton.BackgroundColor = [0 1 0];
            app.Select2randomnodesandfindpathButton.Position = [29 408 230 22];
            app.Select2randomnodesandfindpathButton.Text = '3. Select 2 random nodes and find path';

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