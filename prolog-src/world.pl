%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% world.pl
%
% Path planner world model and related functionality
%
% Stav Rockah, Roman Smirnov
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- dynamic vehicle_at_position/3.

search_depth_limit(10).

% add vehicle positions at current step to fact database
set_vehicle_positions([[_,_,_,_,_,POS_Y,POS_X]|TAIL], VEHICLES, STEP) :-
    write(POS_X), write(" "), write(POS_Y), write(" "), writeln(STEP),
    assertz(vehicle_at_position(POS_X, POS_Y, STEP)),
    set_vehicle_positions(TAIL, VEHICLES, STEP).

% advanced a time step, extrapolate positions, add to fact database 
set_vehicle_positions([], OLD_VEHICLES, STEP) :-
    NEXT_STEP is STEP+1, 
    not(search_depth_limit(NEXT_STEP)),
    update_vehicle_positions(OLD_VEHICLES, VEHICLES),
    set_vehicle_positions(VEHICLES, VEHICLES, NEXT_STEP).

% terminal condition for fact database construction
set_vehicle_positions([], _, STEP) :-
    NEXT_STEP is STEP+1,
    search_depth_limit(NEXT_STEP),!.

% convinience rule
set_vehicle_positions(VEHICLES) :-
    set_vehicle_positions(VEHICLES, VEHICLES, 0).

% extrapolate vehicle positions at next time step
update_vehicle_positions([],[]).
update_vehicle_positions([[_,_,_,VEL_Y,VEL_X,POS_Y,POS_X]|TAIL], VEHICLES) :-
    NEW_X is POS_X+VEL_X,
    NEW_Y is POS_Y+VEL_Y,
    append([[_,_,_, VEL_Y, VEL_X, NEW_Y, NEW_X]], NXT, VEHICLES),
    update_vehicle_positions(TAIL, NXT).