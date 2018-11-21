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

:- include("./world.pl").
:- include("./constraints.pl").

default_port(8083).

start_server :-
    default_port(Port),
    http_server(http_dispatch, [port(Port)]).

stop_server :-
    default_port(Port),
    http_stop_server(Port, []).

handle_message(Message, Response) :-
    retractall(vehicle_at_position(_,_,_)),
    set_vehicle_positions(Message.data.vehicles),  % [0,1,2,3,4,5,6])
    NEXT_S1 is Message.data.car_s+1,
    NEXT_S2 is Message.data.car_s+2,
    NEXT_D is Message.data.car_d,
    Response=_{next_d:[NEXT_D, NEXT_D], next_s:[NEXT_S1, NEXT_S2]}.
    
echo(WebSocket) :-
    ws_receive(WebSocket, Message, [format(json)]),
    (   Message.opcode==close
    ->  true
    ;   handle_message(Message, Response),
        % writeln(Message), % write(" -> "), % writeln(json(Response)),
        ws_send(WebSocket, json(Response)),
        echo(WebSocket)
    ).

