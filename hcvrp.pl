:- lib(ic).
:- lib(ic_global).
:- lib(branch_and_bound).
:- [hcvrp_data].

hcvrp(NCl, NVe, Timeout, Solution, Cost, Time) :-
    cputime(Time1),

    vehicles(VehiclesInput1),
    get_first_n(NVe, VehiclesInput1, VehiclesInput),
    clients(ClientsInput),
    get_first_n(NCl, ClientsInput, ClientsInput1), 
    Clients = [c(0, 0, 0)|ClientsInput1],

    % Clients is the list of the first NCl clients
    % with a "dummy" client in the beggining.

    % VehiclesInput is the list of the first NVe vehicles.

    distances(Clients, Clients, Distances1),
    flatten(Distances1, Distances),

    coordinates(Clients, Latitudes, Longitudes),

    init_vehicles(NCl, NVe, Vehicles),

    flatten(Vehicles, Flattened),

    constrain_occ(Flattened, NCl),

    demands(Clients, Demands),

    constrain_load(VehiclesInput, Demands, NCl, Flattened),

    constrain_zeros(Vehicles),
    
    optimize_route(Vehicles, Latitudes, Longitudes),

    get_cost(Vehicles, NCl, Distances, Cost),

    bb_min(search(Vehicles, 0, first_fail, indomain, complete, []), 
        Cost, bb_options{timeout:Timeout}), 

    purge_zeros(Vehicles, Solution),

    cputime(Time2),
    
    Time is Time2 - Time1.
    

distances([], _, []).
distances([C|Cs], Clients, [D|Ds]) :-
    distances_single(C, Clients, D),
    distances(Cs, Clients, Ds).

distances_single(_, [], []).
distances_single(Curr, [C|Cs], [D|Ds]) :-
    distance(Curr, C, D),
    distances_single(Curr, Cs, Ds).

distance(c(_, X1, Y1), c(_, X2, Y2), D) :-
    D is integer(round(sqrt((X2 - X1)^2 + (Y2 - Y1)^2) * 1000)).

coordinates([], [], []).
coordinates([c(_, X, Y)|Cs], [X|Xs], [Y|Ys]) :-
    coordinates(Cs, Xs, Ys).

demands([], []).
demands([c(Demand, _, _)|Cs], [Demand|Ds]) :-
    demands(Cs, Ds).

get_first_n(0, _, []).
get_first_n(N, [I|Is], [I|Os]) :-
    NNext is N - 1,
    get_first_n(NNext, Is, Os).

init_vehicles(NCl, NVe, Vehicles) :-
    length(Vehicles, NVe),
    init_vehicles(NCl, Vehicles).
init_vehicles(_, []).
init_vehicles(NCl, [V|Vs]) :-
    length(V, NCl),
    V #:: 0..NCl,
    init_vehicles(NCl, Vs).

constrain_occ(_, 0).
constrain_occ(Vehicles, NCl) :-
    occurrences(NCl, Vehicles, 1),
    NextCl is NCl - 1,
    constrain_occ(Vehicles, NextCl).

constrain_load([], _, _, _).
constrain_load([VI|VIs], Demands, NCl, Flattened) :-
    first_n_demands(Flattened, Demands, NCl, NDemands, Rest),
    sum(NDemands) #=< VI,
    constrain_load(VIs, Demands, NCl, Rest).

first_n_demands(Rest, _, 0, [], Rest).
first_n_demands([I|Is], Demands, N, [Demand|RDemands], Rest) :-
    Index #= I + 1,
    element(Index, Demands, Demand),
    NNext is N - 1,
    first_n_demands(Is, Demands, NNext, RDemands, Rest).

constrain_zeros([]).
constrain_zeros([L|Ls]) :-
    constrain_zeros_single(L),
    constrain_zeros(Ls).
constrain_zeros_single([_]).
constrain_zeros_single([L1, L2|Ls]) :-
    L1 #= 0 => L2 #= 0,
    constrain_zeros_single([L2|Ls]).

purge_zeros([], []).
purge_zeros([L|Ls], [R|Rs]) :-
    purge_zeros_single(L, R),
    purge_zeros(Ls, Rs).

purge_zeros_single(Ls, Rs) :- 
    purge_zeros_single(Ls, Rs, []).
purge_zeros_single([], Rs, Rs).
purge_zeros_single([0|_], Rs, Acc) :-
    purge_zeros_single([], Rs, Acc), !.
purge_zeros_single([L|Ls], Rs, Acc) :-
    AccN = [L|Acc],
    purge_zeros_single(Ls, Rs, AccN).

get_cost(Packed, NCl, Distances, Cost) :-
    get_cost(Packed, NCl, Distances, Cost, 0).
get_cost([], _, _, Cost, Cost).
get_cost([P|Ps], NCl, Distances, Cost, Acc) :-
    get_cost_single(P, NCl, Distances, C),
    P = [FirstCl|_],
    get_distance(NCl, Distances, 0, FirstCl, Distance),
    AccN #= Acc + C + Distance,
    get_cost(Ps, NCl, Distances, Cost, AccN).

get_cost_single(Clients, NCl, Distances, Cost) :-
    get_cost_single(Clients, NCl, Distances, Cost, 0).
get_cost_single([C], NCl, Distances, Cost, Acc) :-
    get_distance(NCl, Distances, 0, C, Distance),
    Cost #= Acc + Distance.
get_cost_single([C1, C2|Cs], NCl, Distances, Cost, Acc) :-
    get_distance(NCl, Distances, C1, C2, Distance),
    AccN #= Distance + Acc,
    get_cost_single([C2|Cs], NCl, Distances, Cost, AccN).

get_distance(NCl, Distances, C1, C2, Distance) :-
    Index #= (C1 * (NCl + 1)) + C2 + 1,
    element(Index, Distances, Distance).

get_latitude(Latitudes, Cl, X) :-
    Index #= Cl + 1,
    element(Index, Latitudes, X).

get_longitude(Longitudes, Cl, Y) :-
    Index #= Cl + 1,
    element(Index, Longitudes, Y).

optimize_route([], _, _).
optimize_route([P|Ps], Latitudes, Longitudes) :-
    P = [C1|_],
    get_latitude(Latitudes, C1, X),
    get_longitude(Longitudes, C1, Y),
    optimize_route_single(P, Latitudes, Longitudes, X, Y),
    optimize_route(Ps, Latitudes, Longitudes).

optimize_route_single([_], _, _, _, _).
optimize_route_single([C1, C2|Cs], Latitudes, Longitudes, XFirst, YFirst) :-
    get_latitude(Latitudes, C1, X1),
    get_longitude(Longitudes, C1, Y1),
    get_latitude(Latitudes, C2, X2),
    (X1 #\= 0 and X2 #= 0) => X1 #=< XFirst,
    (X1 #\= 0 and X2 #= 0 and X1 #= XFirst) => Y1 #=< YFirst,
    optimize_route_single([C2|Cs], Latitudes, Longitudes, XFirst, YFirst).