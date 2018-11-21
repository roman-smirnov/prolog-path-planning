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

% highway segment start
road_min_y(1).

% highway segment end
road_max_y(6945).

% don't look at possible positions farther than this value (relative to ego vehicle) 
clipping_distance_x(1).
clipping_distance_y(10).

% proximity to vehicle considered unsafe
unsafe_proximity_x(1).
unsafe_proximity_y(2).

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
    ACCELERATION is abs(NEXT_SPEED - CUR_SPEED). %*TIME_STEP.

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

% add vehicle positions to fact database
set_vehicle_positions([]).
set_vehicle_positions([[POS_X, POS_Y, _VEL_X, _VEL_Y]|VEHICLES]) :-
    assertz(vehicle_at_position(POS_X,POS_Y)),    
    set_vehicle_positions(VEHICLES).

% extrapolate vehicle positions, assert into databsem, retract previous positions
update_vehicle_positions([],[]).
update_vehicle_positions([[X, Y, VEL_X, VEL_Y]| CUR], NEW) :-
    NEW_X is X+VEL_X, NEW_Y is Y+VEL_Y,
    append([[NEW_X, NEW_Y, VEL_X, VEL_Y]], NXT, NEW),
    % remove old position from database, add new position
    retract(vehicle_at_position(X,Y)),
    assertz(vehicle_at_position(NEW_X,NEW_Y)),
    update_vehicle_positions(CUR, NXT).

% find all valid successor positions
% ?- successors(1,1,10, S), length(S,LEN).
successors(CUR_POS_X, CUR_POS_Y, CUR_SPEED, SUCCESSORS) :-
    findall([NEXT_POS_X, NEXT_POS_Y, NEXT_SPEED], valid_transition(CUR_POS_X, CUR_POS_Y, CUR_SPEED, NEXT_POS_X, NEXT_POS_Y, NEXT_SPEED), SUCCESSORS).

% find best successor speed
% ?- successors(1,1,10, S), length(S,LEN), best(S,B).
best([],0).
best([[_,_,CUR_S]|SUCCESORS], [_,_,BEST_S]) :-
    best(SUCCESORS, [_,_,NXT_S]),!,
    ((CUR_S>NXT_S, BEST_S is CUR_S); BEST_S is NXT_S).

% [[0,0,1,1], [1,1,1,1], [2,2,1,1], [3,3,1,1]]

% TODO: check velocity change actually works
% TODO: find best successor state!
% TODO: set maximum path length/steps
% TODO: find path recursively
% TODO: collect solutions into a list, sort by speed, return best solution
% TODO: proximity around vehicle
% TODO: use retractall instead of updating one by one
% TODO: separate update and extrapolate rules
% TODO: create the search pipeline
% TODO: separate into modules


