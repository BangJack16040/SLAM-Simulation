function [World, rob] = configuration(rob, sen_rf, Lmks, Wpts, AxisDim)
global RunTime;


    rob.R(1:2) = Wpts(:,1);
    rob.R(3) = bearingToPoint(rob.R, Wpts(:, 2)) + rob.R(3);
    rob.r = rob.R;
    
    World.Q = diag(rob.q .^ 2);     % System uncertainty
    World.M = diag(sen_rf.noise .^ 2); % Measurement uncertainty
    
    World.tend = RunTime;

    World.W = Lmks;
    World.Wpts = Wpts;
    
    World.y = zeros(2,size(World.W,2));

    World.x = zeros(numel(rob.r)+numel(World.W), 1);    % State estimate
    World.P = zeros(numel(World.x),numel(World.x));     % Covariance matrix
    World.mapspace = 1:numel(World.x);                  % State map availability
    World.l = zeros(2, size(World.W,2));                % Landmark positions

    World.r = find(World.mapspace,numel(rob.r));
    World.mapspace(World.r) = 0;
    World.x(World.r) = rob.R;     % Set initial pose estimate equal to truth
    World.P(World.r, World.r) = 0;  % Set initial pose covariance equal to 0

    World.R_hist = zeros(2, World.tend);
    World.r_hist = zeros(2, World.tend);
    World.Pr_hist = zeros(2, World.tend);
    World.error_hist = zeros(1, World.tend);
    World.scan_error_hist = zeros(2, World.tend);
    World.odo_error_hist = zeros(2, World.tend);
    World.turning_hist = zeros(1, World.tend);
    World.weight_scan_hist = zeros(1, World.tend);
    World.weight_odo_hist = zeros(1, World.tend);


    World.map_res = 0.5;

    World.scan_corr_tolerance = 20;
    % Descretised grid map values
    World.map_vals = -AxisDim+World.map_res:World.map_res:AxisDim-World.map_res;
    World.gridmap = zeros(AxisDim * 2 / World.map_res - 1);
    World.gridmap_counter = ones(size(World.gridmap)) * round(255/2);
    World.scan_data = [];
    World.scan_global = [];
    World.scan_true = [];
end