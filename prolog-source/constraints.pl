%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% constraints.pl
%
% Vehicle path planner module constraint definitions
%
% Stav Rockah, Roman Smirnov
%
% Note: coordinates assumed to be in Frenet reference frame.
% Note: coordinates assumed to be a discrete (integers).
% Hence, we compute on what's essentially a sparse grid with a 1m resolution (i.e accuracy of +-0.5m).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% temporal resolution is 0.1 seconds per search graph transition
% NOTE: we've defined it to be 10 s/m (seconds per meter - a unit of pace) to avoid division (we multiply instead).
time_step(10).

% speed limit in m/s (meter per second)
max_speed(22).

% maximum acceleration in m/s^2
max_acceleration(10).

% motion jerk in m/s^3 (it's a measure of acceleration 'smoothness')
max_jerk(50).

% highway median divider is at x=0, vehicle must be to its right
road_min_x(1).

% road edge is at x=12, vehicle must be to its left
road_max_x(11).

% the given highway segment start coordinate
road_min_y(1).

% the given highway segment end coordinate
road_max_y(6945).

% don't look at possible positions laterally farther (relative to ego vehicle) than this value
clipping_distance_x(1).

% don't look at possible positions further down the highway (relative to ego vehicle) than this value
clipping_distance_y(10).

% database of vehicle coordinates
:- dynamic vehicle_at_position/2.

% generate integer values in given range
range(Low, Low, _).
range(Out,Low,High) :- NewLow is Low+1, NewLow =< High, range(Out, NewLow, High).

% vehicle position should be between the median divider and road verge, and no farther than the clipping distance
position_bounds_x(CUR_POS_X, MIN_X, MAX_X) :-
    clipping_distance_x(CLIP_DIST_X), road_min_x(ROAD_MIN_X),road_max_x(ROAD_MAX_X),
    ((CUR_POS_X-CLIP_DIST_X > ROAD_MIN_X, MIN_X is CUR_POS_X-CLIP_DIST_X); MIN_X is ROAD_MIN_X),
    ((CUR_POS_X+CLIP_DIST_X < ROAD_MAX_X, MAX_X is CUR_POS_X+CLIP_DIST_X); MAX_X is ROAD_MAX_X).

% vehicle position should be between highway segment start and end, and no farther than the clipping distance
position_bounds_y(CUR_POS_Y, MIN_Y, MAX_Y) :-
    clipping_distance_y(CLIP_DIST_Y), road_min_y(ROAD_MIN_Y), road_max_y(ROAD_MAX_Y),
    ((CUR_POS_Y-CLIP_DIST_Y > ROAD_MIN_Y, MIN_Y is CUR_POS_Y-CLIP_DIST_Y); MIN_Y is ROAD_MIN_Y),
    ((CUR_POS_Y+CLIP_DIST_Y < ROAD_MAX_Y, MAX_Y is CUR_POS_Y+CLIP_DIST_Y); MAX_Y is ROAD_MAX_Y).

% generate valid positions upholding constraints
valid_position(CUR_POS_X, CUR_POS_Y, NEXT_POS_X, NEXT_POS_Y) :-
    position_bounds_x(CUR_POS_X, MIN_X, MAX_X),
    position_bounds_y(CUR_POS_Y, MIN_Y, MAX_Y),
    range(NEXT_POS_X,MIN_X,MAX_X),
    range(NEXT_POS_Y,MIN_Y,MAX_Y).

% euclidean distance between given coordinates
distance(CUR_POS_X, CUR_POS_Y, NEXT_POS_X, NEXT_POS_Y, DISTANCE) :-
    DISTANCE is round(sqrt((CUR_POS_X-NEXT_POS_X)**2 + (CUR_POS_Y-NEXT_POS_Y)**2)).

% average speed to go the distance in a single time step
speed(DISTANCE, SPEED) :-
    time_step(TIME_STEP),
    SPEED is DISTANCE * TIME_STEP.

% speed constraints
valid_speed(SPEED) :-
    max_speed(MAX_SPEED),
    SPEED >= 0, SPEED =< MAX_SPEED.

% average change in speed during transition
acceleration(CUR_SPEED, NEXT_SPEED, ACCELERATION) :-
    time_step(TIME_STEP),
    ACCELERATION is abs(NEXT_SPEED - CUR_SPEED)*TIME_STEP.

% acceleration constraints
valid_acceleration(ACCELERATION) :-
    max_acceleration(MAX_ACCL),
    ACCELERATION >= 0, ACCELERATION =< MAX_ACCL.

% overall transition constraints
valid_transition(CUR_POS_X, CUR_POS_Y, CUR_SPEED, NEXT_POS_X, NEXT_POS_Y, NEXT_SPEED) :-
    valid_position(CUR_POS_X, CUR_POS_Y, NEXT_POS_X, NEXT_POS_Y),
    distance(CUR_POS_X, CUR_POS_Y, NEXT_POS_X, NEXT_POS_Y, DISTANCE),
    speed(DISTANCE, NEXT_SPEED),
    valid_speed(NEXT_SPEED),
    acceleration(CUR_SPEED, NEXT_SPEED, ACCELERATION),
    valid_acceleration(ACCELERATION),
    not(vehicle_at_position(NEXT_POS_X, NEXT_POS_Y)).


% TODO: collect solutions into a list, sort by speed, return best solution
% TODO: add vehicle position to database
% TODO: extrapolate vehicle positions, assert into databsem, retract previous positions



% assertz(vehicle_at_position(1,1)).
% retractall(vehicle_at_position(X,Y)).
% vehicle located at coordinates
% vehicle_position(DISP_X,DISP_Y,[[X_CAR,Y_CAR]|CARS_LIST]) :-
%     (DISP_X is X_CAR, DISP_Y is Y_CAR);
%     is_car_position(DISP_X,DISP_Y,CARS_LIST).

% is_car_position(DISP_X,DISP_Y,[]).