% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function [h] = visualize3d(data)
    % Create a figure with three parts. One part is for a 3D visualization,
    % and the other two are for running graphs of angular velocity and displacement.
    figure; plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
    subplot(plots(1));
    pause;

    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t, thrusts] = quadcopter;

    % Set axis scale and labels.
    axis([-10 30 -20 20 5 15]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');

    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, plots)
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
    for t = 1:10:length(data.t)
        % The first, main part, is for the 3D visualization.
        subplot(plots(1));

        % Compute translation to correct linear position coordinates.
        dx = data.x(:, t);
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
        angles = data.sig(:, t);
        rotate = (mrpTOdcm(angles));
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);

        % Compute scaling for the thrust cylinders. The lengths should represent relative
        % strength of the thrust at each propeller, and this is just a heuristic that seems
        % to give a good visual indication of thrusts.
        %scales = exp(data.input(:, t) / min(abs(data.input(:, t))) + 5) - exp(6) +  1.5;
        scales=data.input(:,t);
        for i = 1:4
            % Scale each cylinder. For negative scales, we need to flip the cylinder
            % using a rotation, because makehgtform does not understand negative scaling.
            s = scales(i);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(thrusts(i), 'Matrix', move * rotate * scalez);
        end

        % Update the drawing.      
        xmin = data.x(1,t)-20;%-5;%
        xmax = data.x(1,t)+20;%12;%
        ymin =data.x(2,t)-20;%-10;%
        ymax =data.x(2,t)+20;%10;%
        zmin = data.x(3,t)-5;%0;%
        zmax =data.x(3,t)+5;%14;%
        axis([xmin xmax ymin ymax zmin zmax]);
       
        dd=data.x(:, 1:t);
         plot3(dd(1,:),dd(2,:),dd(3,:),'r')
        drawnow;

        % Use the bottom two parts for angular velocity and displacement.
        subplot(plots(2));
        multiplot(data, data.x, t);
        xlabel('Time (ms)');
        ylabel('Positin (meters)');
        title('Position');
        legend('x', 'y','z');

        subplot(plots(3));
        multiplot(data, data.sig, t);
        xlabel('Time (s)');
        ylabel('Mrp');
        title('Modfied Rodrigues Parameter');
        legend('${\sigma}_1$', '${\sigma}_2$','${\sigma}_3$','Interpreter','latex');
    end
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.t(:, 1:ind);
    values = values(:, 1:ind);

    % Plot in RGB, with different markers for different components.
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b-.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    axis([xmin xmax ymin ymax]);
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-.5, -0.025, -0.025, 1, 0.05, 0.05);
    h(2) = prism(-0.025, -.5, -0.025, 0.05, 1, 0.05);

    % Draw bulbs representing propellers at the end of each arm.
    [x ,y ,z] = sphere;
    x = 0.05 * x;
    y = 0.05 * y;
    z = 0.05 * z;
    h(3) = surf(x - .5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + .5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - .5, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + .5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x ,y, z] = cylinder(0.01, 7);
    thrusts(1) = surf(x, y + .5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(2) = surf(x + .5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');
    thrusts(3) = surf(x, y - .5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(4) = surf(x - .5, y, z, 'EdgeColor', 'none', 'FaceColor', 'y');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X ,Y, Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X ,Y ,Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
  %  d=[X;Y;Z];
   % r=[cos(45) sin(45) 0;-sin(45) cos(45) 0;0 0 1];
   % dd=r*d;
   % X=dd(1,:);
  %  Y=dd(2,:);
  %  Z=dd(3,:);
end
