
function World = SLAM(handles, AxisDim, Lmks, Wpts, Obstacles)

    %%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cla; % Clear axes
    rob = Robot; % Create new robot object
    sen = Sensor;
    sen_rf = Sensor;
    sen_rf.noise = [0.1; 3 * pi/180];
    sen_rf.range = 10;
    sen.range = 30;
    sen.fov = 270 * pi/180;
    rob.q = [0.1;2*pi/180];
    
    [World, rob] = configuration(rob, sen_rf, Lmks, Wpts, AxisDim);
    
    Graphics = initGraphics(World, Obstacles, handles);
    
    % Determine which obstacles are moving
    numObstacles = length(Obstacles);
    numSegments = 0;
    movingObstacles = zeros(1, numObstacles);
    for i = 1:numObstacles
        if ~isequal([0; 0], Obstacles(i).velocity)
            movingObstacles(i) = 1;
        end
        numSegments = numSegments + size(Obstacles(i).vertices, 2) - 1;
    end
    movingObstacles = find(movingObstacles);

    
    pose_this_scan = [];
    this_scan_good = 0;
    big_turn = 0;
    r = World.r;   

    for t = 1:World.tend

        World.t = t;
        
        R_old = rob.R;
        if ~isempty(World.Wpts)
            rob.steer(World.Wpts);
        end
        for i = movingObstacles
            Obstacles(i).move(AxisDim);
        end
        rob.R = rob.move(zeros(2,1), 'true');        % Move with noise
        for lid = 1:size(World.W,2)
            v = sen_rf.noise .* randn(2,1);
            World.y(:,lid) = scanPoint(rob.R, World.W(:,lid)) + v;
        end

        lmks_all = World.y;
        [World.y, lmks_visible] = sen_rf.constrainMeasurement(World.y);
        

        enough_correlations = 0;
        
        last_scan_good = this_scan_good;
        pose_last_scan = pose_this_scan;
        this_scan_good = 0; % Initialise this_scan_good for this iteration

        scan_ref = World.scan_data;
        World.scan_data = -ones(3, numSegments * (round(sen.fov/sen.angular_res) + 1));
        scan_ndx = 1;
        for i = 1:length(Obstacles)
            scan_data = Obstacles(i).getMeasured(sen, rob);
            if ~isempty(scan_data)
                World.scan_data(:, scan_ndx:scan_ndx+size(scan_data, 2)-1) = scan_data;
                scan_ndx = scan_ndx + size(scan_data, 2);
            end
        end
        World.scan_data = World.scan_data(:, World.scan_data(2, :) ~= -1);
        
        World.scan_data = removeDuplicateLasers(World.scan_data);
        World.scan_data = World.scan_data(2:3, :); 

        v = repmat(sen.noise, 1, size(World.scan_data, 2)) .* randn(2,size(World.scan_data, 2));
        World.scan_data = World.scan_data + v;

        if ~isempty(World.scan_data)
            obstaclesDetected = 1;

            World.scan_data = getInvMeasurement(World.scan_data);


            if size(World.scan_data, 2) > World.scan_corr_tolerance
                this_scan_good = 1;
            end
            

        else
            obstaclesDetected = 0;
            World.scan_global = [];
        end

        if last_scan_good && this_scan_good
            n = rob.q .* randn(2,1);
            [R, T, correl, icp_var] = doICP(scan_ref, World.scan_data, 1, rob.u + n);

            if length(correl) > World.scan_corr_tolerance
                enough_correlations = 1;
                r_scan = zeros(3, 1);
                da = getPiToPi(asin(R(2,1)));
                r_scan(1:2) = transToGlobal(pose_last_scan, T);
                r_scan(3) = pose_last_scan(3) + da;
                
                scan_data_corr = World.scan_data;
                scan_data_corr = transToGlobal(r_scan, scan_data_corr);
                scan_data_corr = scan_data_corr(:, correl(:, 2)');

                [r_odo, R_r, R_n] = rob.move(n);

                C = getICPCovariance(icp_var, scan_data_corr);
                J_u = [R zeros(2, 1); 0 0 1];
                cov_scan = J_u * C * J_u';
                cov_scan_norm = trace(cov_scan);
            end
        end
        
if ~enough_correlations
            n = rob.q .* randn(2,1);
            [r_odo, R_r, R_n] = rob.move(n);
            cov_scan = zeros(3, 3);
            dr_scan = zeros(3, 1);
        
    else
            dr_scan = r_scan - rob.r;
            dr_scan(3) = getPiToPi(dr_scan(3));
        end            

        
        cov_odo = R_n * World.Q * R_n';
        cov_odo_norm = trace(cov_odo);
        
        dr_odo = r_odo - rob.r;
        dr_odo(3) = getPiToPi(dr_odo(3));
        
        dr_true = rob.R - R_old;


        if abs(dr_scan(2)) > pi/6 || abs(dr_odo(2)) > pi/6
            big_turn = 1;
        elseif abs(dr_odo(2)) < pi/18
            big_turn = 0;
        end
        

        if ~enough_correlations % || big_turn
            weight_odo = 1;
            weight_scan = 0;
        else
            weight_odo = cov_scan_norm / (cov_scan_norm + cov_odo_norm);
            weight_scan = 1 - weight_odo;
            weight_scan = 1;
            weight_odo = 0;
        end
                

        rob.r = weight_odo * dr_odo + weight_scan * dr_scan + rob.r;
        World.x(r) = rob.r;


        P_rr = World.P(r,r);
        World.P(r,:) = R_r * World.P(r,:);
        World.P(:,r) = World.P(r,:)';
        World.P(r,r) = R_r * P_rr * R_r' + ...
            weight_scan * cov_scan + ...
            weight_odo * cov_odo;
        

        r_old = rob.r;
        
        lmks_visible_known = find(World.l(1,lmks_visible));
        lids = lmks_visible(lmks_visible_known);
        
        for lid = lids    

            [e, E_r, E_l] = scanPoint(rob.r, World.x(World.l(:,lid)));

            E_rl = [E_r E_l];
            rl   = [r World.l(:,lid)'];
            E    = E_rl * World.P(rl,rl) * E_rl';

            yi = World.y(:,lid);

            z = yi - e;
            z(2) = getPiToPi(z(2));
            Z = World.M + E;


            K = World.P(:, rl) * E_rl' * Z^-1;


            World.x = World.x + K * z;
            World.P = World.P - K * Z * K'; % The complexity of this line is 
                                            % very high (subtraction of two
                                            % large matrices)
            rob.r = World.x(r);
        end


        lmks_visible_unknown = find(World.l(1,lmks_visible)==0);
        lids = lmks_visible(lmks_visible_unknown);
        
        if ~isempty(lids)
            for lid = lids
                s = find(World.mapspace, 2);
                if ~isempty(s)
                    World.mapspace(s) = 0;
                    World.l(:,lid) = s';
                    % Measurement
                    yi = World.y(:,lid);

                    [World.x(World.l(:,lid)), L_r, L_y] = invScanPoint(rob.r, yi);
                    World.P(s,:) = L_r * World.P(r,:);
                    World.P(:,s) = World.P(s,:)'; % Cross variance of lmk with all other lmks
                    World.P(s,s) = L_r * World.P(r,r) * L_r' + L_y * World.M * L_y';
                end
            end
        end
        pose_this_scan = rob.r; % The corrected pose at which the scan was taken
            

        update = rob.r - r_old;

        if enough_correlations && ~isequal(rob.r, r_old)

            World.scan_data = transformPoints(World.scan_data, update);
                        
            World = doMapping(World, sen, rob.r, World.scan_data, handles);
        elseif ~obstaclesDetected && isequal(mod(World.t, 5), 0)

            World = doMapping(World, sen, rob.r, World.scan_data, handles);
        end
        
        if ~isempty(World.scan_data)
            World.scan_global = transToGlobal(rob.r, World.scan_data);
        end

        World.R_hist(:, t) = rob.R(1:2);	% True position history
        World.r_hist(:, t) = rob.r(1:2);   % Estimated position history
        pose_error2 = (rob.R(1:2) - rob.r(1:2)).^2;
        World.error_hist(t) = sqrt(pose_error2(1) + pose_error2(2));
        World.Pr_hist(:, t) = [World.P(r(1),r(1)); World.P(r(2),r(2))];
        
        if enough_correlations
            scan_error = abs(dr_scan-dr_true); scan_error(3) = abs(getPiToPi(scan_error(3)));
            scan_error(1) = pdist([0 0; scan_error(1) scan_error(2)]); scan_error(2) = [];
        else
            scan_error = [0; 0];
        end
        
        odo_error = abs(dr_odo-dr_true); odo_error(3) = abs(getPiToPi(odo_error(3)));
        odo_error(1) = pdist([0 0; odo_error(1) odo_error(2)]); odo_error(2) = [];
        
        World.scan_error_hist(:, t) = scan_error;
        World.odo_error_hist(:, t) = odo_error;
        World.turning_hist(t) = rob.u(2);
        World.weight_scan_hist(t) = weight_scan;
        World.weight_odo_hist(t) = weight_odo;


        Graphics.lmks_all = lmks_all; Graphics.lmks_visible = lmks_visible;
        doGraphics(rob, World, Graphics, Obstacles(movingObstacles), AxisDim);

    end
    

    % Plot trajectories;
    
    set(Graphics.R_hist, 'xdata', World.R_hist(1,:), 'ydata', World.R_hist(2, :))
    set(Graphics.r_hist, 'xdata', World.r_hist(1,:), 'ydata', World.r_hist(2, :))
    
    % Plot position uncertainties
    figure('color', 'white')
    subplot(2, 1, 1)
    plot(1:World.tend, World.error_hist)
    title('SLAM: position error over time')
    xlabel('Time (s)'), ylabel('Position Error (m)')
    subplot(2, 1, 2)
    plot(1:World.tend, sqrt(World.Pr_hist(1, :)), 'r', ...
        1:World.tend, sqrt(World.Pr_hist(2, :)), 'b')
    title('SLAM: position uncertainty over time')
    xlabel('Time (s)'), ylabel('Standard Deviation (m)')
    legend('St. Dev. in x', 'St. Dev. in y')
    
    figure('color', 'white')
    subplot(4, 1, 1)
    AX = plotyy(1:World.tend, World.scan_error_hist(1,:), ...
        1:World.tend, World.scan_error_hist(2,:), 'plot');
    set(get(AX(1),'Ylabel'),'String',['Position' char(10) 'Error (m)'])
    set(get(AX(2),'Ylabel'),'String',['Angular' char(10) 'Error (rad)'])
    title('Scan errors over time')
    xlabel('Time (s)')
    
    subplot(4, 1, 2)
    AX2 = plotyy(1:World.tend, World.odo_error_hist(1,:), ...
        1:World.tend, World.odo_error_hist(2,:), 'plot');
    set(get(AX2(1),'Ylabel'),'String',['Position' char(10) 'Error (m)']) 
    set(get(AX2(2),'Ylabel'),'String',['Angular' char(10) 'Error (rad)'])
    title('Odometry error over time')
    
    subplot(4, 1, 3)
    plot(1:World.tend, World.weight_scan_hist, 'r', ...
        1:World.tend, World.weight_odo_hist, 'b')
    title('Scan and odometry weights over time')
    xlabel('Time (s)'), ylabel('Weight')
    legend('Scan', 'Odometry')
    axis tight
    
    subplot(4, 1, 4)
    plot(1:World.tend, World.turning_hist)
    title('Turning control over time')
    xlabel('Time (s)'), ylabel('Angle (rad)')
    axis tight
end