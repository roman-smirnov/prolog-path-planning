%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% constraints.pl
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% temporal resolution in seconds (i.e time increment per transition)
time_step(0.5).

% how many time steps forward to plan
max_search_depth(3).

% search time allotment in seconds
max_search_time(2.0).

% how many successors to generate when expanding a search node
max_successors(32).

% velocity constraints in m/s (meters per second)
max_velocity(22.0).
min_velocity(-22.0).

% acceleration in m/s^2
max_acceleration(10.0).
min_acceleration(-10.0).

% motion jerk in m/s^3 (it's a measure of acceleration 'smoothness')
max_jerk(50.0).
min_jerk(-50.0).

% highway median divider x coordinate, vehicle must be to its right
min_road_x(0).
% road verge x coordinate, vehicle must be to its left
max_road_x(12).

% y coordinate of defined highway segment start 
min_road_y(0).
% y coordinate of defined highway segment
max_road_y(6945).

% x coordinates of lane centers on track at direction of motion
center_of_left_lane_x(2).
center_of_middle_lane_x(6).
center_of_right_lane_x(10).

evaluation_penalty_value(9999).

% calculate xy coordinates of the next position to visit 
position(CUR_POS_X, CUR_POS_Y, DISP_X, DISP_Y, NXT_POS_X, NXT_POS_Y) :-
    NXT_POS_X is CUR_POS_X+DISP_X,
    NXT_POS_Y is CUR_POS_Y+DISP_Y.

% calculate xy displacemnet from current position
displacement(CUR_VEL_X, CUR_VEL_Y, NXT_VEL_X, NXT_VEL_Y, DISP_X, DISP_Y) :-
    time_step(TIME_STEP),
    AVG_VEL_X is (CUR_VEL_X+NXT_VEL_X)/2,
    AVG_VEL_Y is (CUR_VEL_Y+NXT_VEL_Y)/2,
    DISP_X is AVG_VEL_X*TIME_STEP,
    DISP_Y is AVG_VEL_Y*TIME_STEP. 

% calculate x,y velocity components
velocity(CUR_ACCL_X, CUR_ACCL_Y, CUR_VEL_X, CUR_VEL_Y, NXT_ACCL_X, NXT_ACCL_Y, NXT_VEL_X, NXT_VEL_Y) :-
    time_step(TIME_STEP),
    AVG_ACCL_X is (CUR_ACCL_X + NXT_ACCL_X)/2,
    AVG_ACCL_Y is (CUR_ACCL_Y + NXT_ACCL_Y)/2,
    NXT_VEL_X is CUR_VEL_X+AVG_ACCL_X*TIME_STEP,
    NXT_VEL_Y is CUR_VEL_Y+AVG_ACCL_Y*TIME_STEP.

% calculate x,y acceleration vector components
acceleration(CUR_ACCL_X, CUR_ACCL_Y, JERK_X, JERK_Y, NXT_ACCL_X, NXT_ACCL_Y) :-
    time_step(TIME_STEP),
    NXT_ACCL_X is CUR_ACCL_X + JERK_X*TIME_STEP,
    NXT_ACCL_Y is CUR_ACCL_Y + JERK_Y*TIME_STEP.

% generate random jerk x,y vector components upholding defined constraints
jerk(JERK_X, JERK_Y) :-
    max_jerk(MAX_JERK),
    RAND_JERK_MAGNITUDE = random_float,
    RAND_JERK_RATIO = pi*2*random_float,
    JERK_X is RAND_JERK_MAGNITUDE*MAX_JERK*cos(RAND_JERK_RATIO),
    JERK_Y is RAND_JERK_MAGNITUDE*MAX_JERK*sin(RAND_JERK_RATIO).