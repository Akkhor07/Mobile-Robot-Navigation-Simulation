%% ROBOT_NAVIGATION_SIM
% Autonomous mobile-robot navigation demo combining:
%   - Theta* global path planning (any-angle grid search)
%   - Path smoothing / resampling
%   - Simulated 2D LIDAR (ray marching against a ground-truth occupancy grid)
%   - Pure-pursuit path tracking blended with Artificial Potential Field (APF)
%     local obstacle avoidance
%   - Live occupancy-grid mapping ("SLAM" here means occupancy mapping with a
%     KNOWN robot pose -- see the honesty note at the bottom of this header)
%   - Simple stall detection + Theta* re-planning
%   - Multi-panel live animation, optionally recorded to MP4

clear; clc; close all; rng(7);

%% ---------------------- USER-TUNABLE PARAMETERS ------------------------
worldW      = 8.0;      % world width  [m]
worldH      = 8.0;      % world height [m]
res         = 0.10;     % occupancy grid resolution [m/cell]

robotRadius = 0.18;     % [m]
safetyMargin= 0.08;     % extra inflation for planning [m]

startPose   = [0.5, 0.5, pi/4];   % [x y theta]
goalPos     = [7.3, 7.3];         % [x y]
goalTol     = 0.20;                % [m]

vNominal    = 0.45;     % nominal forward speed [m/s]
lookahead   = 0.55;     % pure-pursuit lookahead distance [m]
KpHeading   = 3.0;      % heading P-gain

lidarMaxRange = 2.6;    % [m]
lidarNumBeams = 72;     % beams over 360 deg
lidarFOV      = 2*pi;

apfInfluence = 0.75;    % [m] obstacles closer than this create repulsion
apfGain      = 0.35;
apfSafeDist  = 0.55;    % [m] distance at which repulsion fully dominates

dt          = 0.1;      % simulation step [s]
maxSimTime  = 90;       % [s] safety cutoff
maxSteps    = round(maxSimTime/dt);

stallWindowSteps = 25;      % ~2.5 s
stallDistThresh  = 0.04;    % [m] considered "stuck" if less than this progress
stallCooldown    = 40;      % steps to wait after a replan before checking again

recordVideo = true;
videoFile   = fullfile(fileparts(mfilename('fullpath')), 'robot_navigation_demo.mp4');
videoFPS    = 20;
videoEveryN = 2;   % write every Nth simulated frame (keeps file size sane)

%% ---------------------------- ENVIRONMENT --------------------------------
nx = round(worldW/res);
ny = round(worldH/res);

obstacles = define_obstacles();
occGroundTruth = build_environment(nx, ny, res, obstacles);

inflateRadius = robotRadius + safetyMargin;
occInflated   = inflate_occ(occGroundTruth, res, inflateRadius);

startCell = world_to_grid(startPose(1:2), res, nx, ny);
goalCell  = world_to_grid(goalPos,        res, nx, ny);

if occInflated(startCell(2), startCell(1)) || occInflated(goalCell(2), goalCell(1))
    error('Start or goal lies inside an inflated obstacle -- adjust startPose/goalPos.');
end

%% ------------------------- INITIAL GLOBAL PLAN ---------------------------
pathCells = theta_star(occInflated, startCell, goalCell);
if isempty(pathCells)
    error('Theta*: no path found between start and goal.');
end
pathWorld  = grid_to_world(pathCells, res);
globalPath = resample_path(pathWorld, 0.05);   % 5 cm spacing

%% ------------------------------ STATE ------------------------------------
pose = startPose;              % [x y theta]
pathIdxHint = 1;

belief   = zeros(ny, nx);      % simplified log-odds occupancy belief (0 = unknown)
lOcc     = 1.6;
lFree    = 0.35;
lMax     = 6;
lMin     = -6;

trail = pose(1:2);
tHist = 0; vHist = 0; dHist = norm(pose(1:2)-goalPos);

posHistBuffer = repmat(pose(1:2), stallWindowSteps, 1);
stallCooldownCounter = 0;
numReplans = 0;
numNearCollisions = 0;
numCollisions = 0;

reachedGoal = false;

%% ------------------------------ FIGURE SETUP ------------------------------
fig = figure('Color','w','Position',[80 80 1150 820]);

% --- Panel 1: global view ---
axG = subplot(2,2,1); hold(axG,'on'); axis(axG,'equal');
xlim(axG,[0 worldW]); ylim(axG,[0 worldH]);
title(axG,'Global View: Theta* Path, LIDAR, Robot');
draw_obstacles(axG, obstacles);
hPlanPath  = plot(axG, globalPath(:,1), globalPath(:,2), 'm--', 'LineWidth',1.3);
hTrail     = plot(axG, trail(:,1), trail(:,2), 'b-', 'LineWidth', 1.6);
hLidarLines= plot(axG, nan, nan, 'g-', 'LineWidth', 0.5);
hGoalStar  = plot(axG, goalPos(1), goalPos(2), 'rp', 'MarkerFaceColor','r','MarkerSize',14);
robotBodyG = patch(axG, nan, nan, [0.2 0.4 0.9], 'EdgeColor','k');
for w = 1:4
    wheelHG(w) = patch(axG, nan, nan, [0.1 0.1 0.1], 'EdgeColor','k'); %#ok<SAGROW>
end

% --- Panel 2: zoomed local view ---
axZ = subplot(2,2,2); hold(axZ,'on'); axis(axZ,'equal');
title(axZ,'Zoomed Robot View + LIDAR Rays');
draw_obstacles(axZ, obstacles);
hLidarLinesZ = plot(axZ, nan, nan, 'g-', 'LineWidth', 0.7);
hPlanPathZ   = plot(axZ, globalPath(:,1), globalPath(:,2), 'm--', 'LineWidth',1.3);
robotBodyZ = patch(axZ, nan, nan, [0.2 0.4 0.9], 'EdgeColor','k');
for w = 1:4
    wheelHZ(w) = patch(axZ, nan, nan, [0.1 0.1 0.1], 'EdgeColor','k'); %#ok<SAGROW>
end

% --- Panel 3: live occupancy grid ---
axM = subplot(2,2,3); hold(axM,'on'); axis(axM,'equal','tight');
title(axM,'Live Occupancy Grid Map');
[gxImg,gyImg] = meshgrid(((1:nx)-0.5)*res, ((1:ny)-0.5)*res);
hMapImg = imagesc(axM, [0 worldW],[0 worldH], prob_from_logodds(belief));
set(axM,'YDir','normal'); colormap(axM, flipud(gray)); caxis(axM,[0 1]);
hRobotMapMarker = plot(axM, pose(1), pose(2), 'bo', 'MarkerFaceColor','b','MarkerSize',6);

% --- Panel 4: stats over time ---
axS = subplot(2,2,4); hold(axS,'on');
title(axS,'Speed [m/s] and Distance-to-Goal [m] vs Time');
% NOTE: deliberately not using yyaxis (dual y-axis) so this script also
% runs unmodified on GNU Octave, which does not implement yyaxis. Both
% quantities are plotted on one shared axis with a legend instead.
hVLine = plot(axS, tHist, vHist, 'b-', 'LineWidth',1.3);
hDLine = plot(axS, tHist, dHist, 'r-', 'LineWidth',1.3);
legend(axS, {'Speed [m/s]','Dist to goal [m]'}, 'Location','best');
xlabel(axS,'Time [s]'); grid(axS,'on');

drawnow;

if recordVideo
    vidObj = VideoWriter(videoFile, 'MPEG-4');
    vidObj.FrameRate = videoFPS;
    open(vidObj);
end

%% ------------------------------ MAIN LOOP ---------------------------------
for step = 1:maxSteps
    t = step*dt;

    % ---- Simulated LIDAR scan against the TRUE environment ----
    [ranges, angles, hitPts] = lidar_scan(pose, occGroundTruth, res, nx, ny, ...
                                           lidarMaxRange, lidarNumBeams, lidarFOV);
    minRange = min(ranges);
    if minRange < robotRadius + 0.05
        numNearCollisions = numNearCollisions + 1;
    end
    gcell = world_to_grid(pose(1:2), res, nx, ny);
    if occGroundTruth(gcell(2), gcell(1))
        numCollisions = numCollisions + 1;
    end

    % ---- Update occupancy-grid belief from this scan ----
    belief = update_belief(belief, pose, ranges, angles, lidarMaxRange, res, nx, ny, ...
                            lOcc, lFree, lMax, lMin);

    % ---- Control: pure-pursuit lookahead blended with APF repulsion ----
    [lookPt, pathIdxHint] = find_lookahead_point(globalPath, pose, lookahead, pathIdxHint);
    Frep = apf_repulsion(pose, hitPts, ranges, apfInfluence, apfGain);

    pathDir = lookPt - pose(1:2);
    pathDirN = pathDir / max(norm(pathDir), 1e-6);
    if norm(Frep) > 1e-6
        repDirN = Frep / norm(Frep);
    else
        repDirN = [0 0];
    end
    apfWeight = min(1, max(0, (apfSafeDist - minRange) / apfSafeDist));
    desiredDir = (1 - apfWeight) * pathDirN + apfWeight * repDirN;
    if norm(desiredDir) < 1e-6
        desiredDir = pathDirN;
    end
    desiredHeading = atan2(desiredDir(2), desiredDir(1));
    headingErr = wrap_to_pi(desiredHeading - pose(3));

    omega = KpHeading * headingErr;
    omega = max(-2.5, min(2.5, omega));
    v = vNominal * (1 - 0.6*min(1,abs(headingErr)/pi));
    v = v * (0.35 + 0.65*min(1, minRange/apfSafeDist));
    v = max(0, v);

    % ---- Integrate kinematics (unicycle model) ----
    pose(1) = pose(1) + v*cos(pose(3))*dt;
    pose(2) = pose(2) + v*sin(pose(3))*dt;
    pose(3) = wrap_to_pi(pose(3) + omega*dt);
    pose(1) = min(max(pose(1), 0.02), worldW-0.02);
    pose(2) = min(max(pose(2), 0.02), worldH-0.02);

    trail(end+1,:) = pose(1:2); %#ok<SAGROW>
    tHist(end+1) = t; vHist(end+1) = v; dHist(end+1) = norm(pose(1:2)-goalPos); %#ok<SAGROW>

    % ---- Stall detection & Theta* re-planning ----
    posHistBuffer = [posHistBuffer(2:end,:); pose(1:2)];
    if stallCooldownCounter > 0
        stallCooldownCounter = stallCooldownCounter - 1;
    end
    progressSpan = max(vecnorm(posHistBuffer - posHistBuffer(1,:), 2, 2));
    isStalled = (progressSpan < stallDistThresh) && (dHist(end) > goalTol) && (stallCooldownCounter==0);

    if isStalled
        numReplans = numReplans + 1;
        % brief recovery: back up a little
        pose(1) = pose(1) - 0.15*cos(pose(3));
        pose(2) = pose(2) - 0.15*sin(pose(3));
        pose(3) = wrap_to_pi(pose(3) + pi/3);

        curCell = world_to_grid(pose(1:2), res, nx, ny);
        newPathCells = theta_star(occInflated, curCell, goalCell);
        if ~isempty(newPathCells)
            newPathWorld = grid_to_world(newPathCells, res);
            globalPath = resample_path(newPathWorld, 0.05);
            pathIdxHint = 1;
        end
        stallCooldownCounter = stallCooldown;
    end

    % ---- Check goal ----
    if dHist(end) < goalTol
        reachedGoal = true;
    end

    % ---- Visualization update (every step, video written every Nth) ----
    set(hTrail, 'XData', trail(:,1), 'YData', trail(:,2));
    set(hPlanPath, 'XData', globalPath(:,1), 'YData', globalPath(:,2));
    set(hPlanPathZ, 'XData', globalPath(:,1), 'YData', globalPath(:,2));

    % LIDAR ray segments (NaN-separated so they draw as one line object)
    nb = numel(angles);
    lx = nan(3*nb,1); ly = nan(3*nb,1);
    for i = 1:nb
        ex = pose(1) + ranges(i)*cos(angles(i));
        ey = pose(2) + ranges(i)*sin(angles(i));
        lx(3*i-2) = pose(1); ly(3*i-2) = pose(2);
        lx(3*i-1) = ex;      ly(3*i-1) = ey;
        lx(3*i)   = NaN;     ly(3*i)   = NaN;
    end
    set(hLidarLines, 'XData', lx, 'YData', ly);
    set(hLidarLinesZ, 'XData', lx, 'YData', ly);

    [bx, by] = robot_shape_points(pose, 0.28, 0.20);
    set(robotBodyG, 'XData', bx, 'YData', by);
    set(robotBodyZ, 'XData', bx, 'YData', by);
    [wx, wy] = wheel_shape_points(pose, 0.28, 0.20);
    for w = 1:4
        set(wheelHG(w), 'XData', wx{w}, 'YData', wy{w});
        set(wheelHZ(w), 'XData', wx{w}, 'YData', wy{w});
    end

    zoomR = 1.4;
    xlim(axZ, [pose(1)-zoomR, pose(1)+zoomR]);
    ylim(axZ, [pose(2)-zoomR, pose(2)+zoomR]);

    set(hMapImg, 'CData', prob_from_logodds(belief));
    set(hRobotMapMarker, 'XData', pose(1), 'YData', pose(2));

    set(hVLine, 'XData', tHist, 'YData', vHist);
    set(hDLine, 'XData', tHist, 'YData', dHist);
    xlim(axS, [0, max(1,t)]);

    drawnow;

    if recordVideo && mod(step, videoEveryN)==0
        writeVideo(vidObj, getframe(fig));
    end

    if reachedGoal
        break;
    end
end

if recordVideo
    close(vidObj);
end

%% ------------------------------ FINAL REPORT -------------------------------
fprintf('\n--- Simulation summary (honest numbers, no rounding up) ---\n');
if reachedGoal
    fprintf('Result: GOAL REACHED at t = %.2f s\n', t);
else
    fprintf('Result: goal NOT reached within %.1f s time budget (dist remaining %.2f m)\n', ...
        maxSimTime, dHist(end));
end
fprintf('Path length traveled: %.2f m\n', sum(vecnorm(diff(trail),2,2)));
fprintf('Re-plans triggered:   %d\n', numReplans);
fprintf('Steps with LIDAR reading < robot radius + 5cm (near-collision): %d / %d\n', ...
    numNearCollisions, step);
fprintf('Steps where robot center was inside a true obstacle cell (collision): %d / %d\n', ...
    numCollisions, step);
if recordVideo
    fprintf('Video written to: %s\n', videoFile);
end
fprintf('-------------------------------------------------------------\n');


