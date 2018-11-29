%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% websocket.pl
%
% Path planner logic http websocket server
%
% Stav Rockah, Roman Smirnov
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



:- use_module(library(http/websocket)).
:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- http_handler(root(ws), http_upgrade_to_websocket(echo, []), [spawn([])]).
:- include("./search.pl").

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SERVER SETUP AND CALLBACKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

default_port(8083).

start_server :-
    default_port(Port),
    http_server(http_dispatch, [port(Port)]).

stop_server :-
    default_port(Port),
    http_stop_server(Port, []).

echo(WebSocket) :-
    ws_receive(WebSocket, Message, [format(json)]),
    (   Message.opcode==close
    ->  true
    ;   writeln(Message.data),
        handle_message(Message, Response),
        writeln(" -> "),
        writeln(json(Response)),
        writeln(" "),
        ws_send(WebSocket, json(Response)),
        echo(WebSocket)
    ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MESSAGE HANDLING & RESPONSE COMPUTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_state_from_message(START, Message) :-
    POS_X is Message.data.pos_d,
    POS_Y is Message.data.pos_s,
    VEL_X is Message.data.vel_d, 
    VEL_Y is Message.data.vel_s,
    ACCL_X is 0, ACCL_Y is 0,
    state(POS_X, POS_Y, VEL_X, VEL_Y, ACCL_X, ACCL_Y, START).

% parse incoming json message, generate response message
handle_message(Message, Response) :-
    % retractall(vehicle_at_position(_,_,_)),
    % set_vehicle_positions(Message.data.vehicles),  % [0,1,2,3,4,5,6])
    max_search_depth(PATH_LENGTH),
    time_step(TIME_INCREMENT),
    start_state_from_message(START, Message),
    search(START, GOAL),
    path(GOAL, PATH_X, PATH_Y),
    Response=_{next_d:PATH_X, next_s:PATH_Y, path_len:PATH_LENGTH, time_inc:TIME_INCREMENT}.
    


