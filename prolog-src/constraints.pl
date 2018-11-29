%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% constraints.pl
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% temporal resolution in seconds (i.e time increment per graph transition)
time_step(1.0).

% how many time steps forward to plan
max_search_depth(10).

% velocity constraints in m/s (meters per second)
max_velocity(22.0).
min_velocity(-22.0).

% acceleration in m/s^2
max_acceleration(10).
min_acceleration(-10).

% motion jerk in m/s^3 (it's a measure of acceleration 'smoothness')
max_jerk(50.0).
min_jerk(-50.0).

% highway median divider is at x=0, vehicle must be to its right
min_road_x(0).

% road verge is at x=12, vehicle must be to its left
max_road_x(12).

% highway segment start at y=0
min_road_y(0).

% highway segment end at y=6946
max_road_y(6945).

position(CUR_POS_X, CUR_POS_Y, DISP_X, DISP_Y, NXT_POS_X, NXT_POS_Y) :-
    min_road_x(MIN_X),
    max_road_x(MAX_X),
    min_road_y(MIN_Y),
    max_road_y(MAX_Y),
    NXT_POS_X is round(CUR_POS_X+DISP_X),
    NXT_POS_Y is round(CUR_POS_Y+DISP_Y),
    NXT_POS_X>=MIN_X,
    NXT_POS_X=<MAX_X,
    NXT_POS_Y>=MIN_Y,
    NXT_POS_Y=<MAX_Y.

displacement(CUR_VEL_X, CUR_VEL_Y, NXT_VEL_X, NXT_VEL_Y, DISP_X, DISP_Y) :-
    time_step(TIME_STEP),
    AVG_VEL_X is (CUR_VEL_X+NXT_VEL_X)/2,
    AVG_VEL_Y is (CUR_VEL_Y+NXT_VEL_Y)/2,
    DISP_X is AVG_VEL_X*TIME_STEP,
    DISP_Y is AVG_VEL_Y*TIME_STEP. 

velocity(CUR_ACCL_X, CUR_ACCL_Y, CUR_VEL_X, CUR_VEL_Y, NXT_ACCL_X, NXT_ACCL_Y, NXT_VEL_X, NXT_VEL_Y) :-
    time_step(TIME_STEP),
    max_velocity(MAX_VEL),
    AVG_ACCL_X is (CUR_ACCL_X + NXT_ACCL_X)/2,
    AVG_ACCL_Y is (CUR_ACCL_Y + NXT_ACCL_Y)/2,
    NXT_VEL_X is round(CUR_VEL_X+AVG_ACCL_X*TIME_STEP),
    NXT_VEL_Y is round(CUR_VEL_Y+AVG_ACCL_Y*TIME_STEP),
    NXT_VEL_MAGNITUDE is sqrt(NXT_VEL_X**2+NXT_VEL_Y**2),
    NXT_VEL_MAGNITUDE=<MAX_VEL. 

acceleration(NXT_ACCL_X, NXT_ACCL_Y) :-
    min_acceleration(MIN_ACCL),
    max_acceleration(MAX_ACCL),
    between(MIN_ACCL, MAX_ACCL, NXT_ACCL_X),
    between(MIN_ACCL, MAX_ACCL, NXT_ACCL_Y),
    NXT_ACCL_MAGNITUDE is sqrt(NXT_ACCL_X**2+NXT_ACCL_Y**2),
    NXT_ACCL_MAGNITUDE=<MAX_ACCL.

jerk(CUR_ACCL_X, CUR_ACCL_Y, NXT_ACCL_X, NXT_ACCL_Y) :-
    max_jerk(MAX_JERK),
    time_step(TIME_STEP),
    JERK_X is (NXT_ACCL_X-CUR_ACCL_X)/TIME_STEP,
    JERK_Y is (NXT_ACCL_Y-CUR_ACCL_Y)/TIME_STEP,
    JERK_MAGNITUDE is sqrt(JERK_X**2+JERK_Y**2),
    JERK_MAGNITUDE=<MAX_JERK.