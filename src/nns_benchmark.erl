-module(nns_benchmark).
-export([validate/0, run/0, validate/1, run/1]).
-include_lib("rstar/include/rstar.hrl").

suites() ->
    [euclid_2d, euclid_3d, sphere].

common_impl() ->
    [{mvptree2, impl(mvptree,2)},
     {mvptree3, impl(mvptree,3)},
     {mvptree4, impl(mvptree,4)},
     {mvptree5, impl(mvptree,5)},
     {mvptree6, impl(mvptree,6)},
     {mvptree7, impl(mvptree,7)},
     {mvptree8, impl(mvptree,8)}].

suite(euclid_2d) ->
    {fun({X1,Y1},{X2,Y2}) ->
             X = X2 - X1,
             Y = Y2 - Y1,
             math:sqrt(X*X+Y*Y)
     end,
     fun () ->
             {rand:uniform(),
              rand:uniform()}
     end,
     2.0,
     [{rstar, impl(rstar)},
      {kd_tree, impl(kd_tree)}
      |common_impl()]
    };
suite(euclid_3d) ->
    {fun({X1,Y1,Z1},{X2,Y2,Z2}) ->
             X = X2 - X1,
             Y = Y2 - Y1,
             Z = Z2 - Z1,
             math:sqrt(X*X+Y*Y+Z*Z)
     end,
     fun () ->
             {rand:uniform(),
              rand:uniform(),
              rand:uniform()}
     end,
     2.0,
     [{e3d_kd3, impl(e3d_kd3)}
      |common_impl()]
    };
suite(sphere) ->
    R = math:pi() / 180.0,
    {fun ({Lat1, Lon1}, {Lat2, Lon2}) ->
             great_circle_distance(Lat1*R, Lat2*R, (Lon2 - Lon1) * R)
     end,
     fun() ->
             {rand:uniform() * 180 - 90,
              rand:uniform() * 360 - 180}
     end,
     math:pi()*4,
     [
      {e3d_kd3, impl(sphere3d, e3d_kd3)},
      {mvptree2, impl(sphere3d, mvptree,2)},
      {mvptree3, impl(sphere3d, mvptree,3)},
      {mvptree4, impl(sphere3d, mvptree,4)},
      {mvptree5, impl(sphere3d, mvptree,5)},
      {mvptree6, impl(sphere3d, mvptree,6)},
      {mvptree7, impl(sphere3d, mvptree,7)},
      {mvptree8, impl(sphere3d, mvptree,8)},
      {kdtree, impl(kdtree)}
      |common_impl()]
    }.

impl(X, Y, Z) ->
    impl({X, {Y, Z}}).

impl(X, Y) ->
    impl({X, Y}).

impl({sphere3d, Impl}) ->
    R = math:pi() / 180.0,
    Convert =
        fun({Lat, Lon}) ->
                LatR = Lat * R,
                LonR = Lon * R,
                C = math:cos(LatR),
                X = math:sin(LonR) * C,
                Y = math:cos(LonR) * C,
                Z = math:sin(LatR),
                {X, Y, Z}
        end,
    Distance =
        fun({X1,Y1,Z1},{X2,Y2,Z2}) ->
                X = X2 - X1,
                Y = Y2 - Y1,
                Z = Z2 - Z1,
                math:sqrt(X*X+Y*Y+Z*Z)
        end,

    {FromList, Search} = impl(Impl),
    {fun(_, List) ->
             FromList(Distance, [{Convert(P), V} || {P, V} <- List])
     end,
     fun(_, Point, Min, Value, Index) ->
             Search(Distance, Convert(Point), Min, Value, Index)
     end};
impl(naive) ->
    {fun (_,List) ->
             List
     end,
     fun naive_search/5};
impl(vptree) ->
    {fun vptree:from_list/2,
     fun vptree:search/5};
impl(vptree2) ->
    {fun vptree2:from_list/2,
     fun vptree2:search/5};
impl({mvptree, M}) ->
    {fun (Distance, List) -> mvptree:from_list(M, Distance, List) end,
     fun mvptree:search/5};
impl(rstar) ->
    {fun(_, List) ->
             lists:foldl(
               fun (X, Tree) ->
                       rstar:insert(Tree, X)
               end,
               rstar:new(2),
               [rstar_geometry:point2d(X, Y, V)
                || {{X,Y},V} <- List ])
     end,
     fun(_, {X, Y}, _, Value, Tree) ->
             [#geometry{value=V}] = rstar:search_nearest(Tree, rstar_geometry:point2d(X, Y, Value), 1),
             V
     end};
impl(kd_tree) ->
    {fun(_, List) ->
             kd_tree:kdtree([erlang:insert_element(1, P, V) || {P,V} <- List], 0, 2)
     end,
     fun(_, Point, _, Value, Tree) ->
             [{_, Tuple}] = kd_tree:nearest_neighbour(Tree, erlang:insert_element(1, Point, Value), 1, 2),
             element(1, Tuple)
     end};
impl(e3d_kd3) ->
    {fun(_, List) ->
             e3d_kd3:from_list([{V, P} || {P,V} <- List])
     end,
     fun (_, Point, _, _, Tree) ->
             {V, _} = e3d_kd3:nearest(Point, Tree),
             V
     end};
impl(kdtree) ->
    {fun(_, List) ->
             kdtree:from_list(List)
     end,
     fun(_, Point, _, _, Tree) ->
             {{_, V}, _} = kdtree:nearest(Tree, Point),
             V
     end}.

naive_search(_, _, _, V, []) ->
    V;
naive_search(Distance, Point, Min, Value, [{L, V}|T]) ->
    case Distance(L, Point) of
        D when D < Min ->
            naive_search(Distance, Point, D, V, T);
        _ ->
            naive_search(Distance, Point, Min, Value, T)
    end.

validate() ->
    [validate(S) || S <- suites()],
    ok.

run() ->
    [run(S) || S <- suites()],
    ok.

validate(Suite) ->
    validate(suite(Suite), 5000, 10000).

run(Suite) ->
    run(suite(Suite), 100000, 5000).

run(Suite, Size, Time) ->
    rand:seed(exrop),
    run(Suite, rand:export_seed(), Size, Time).

validate(Suite, Size, Times) ->
    rand:seed(exrop),
    validate(Suite, rand:export_seed(), Size, Times).

validate({Distance, Point, Limit, Impls}, Seed, Size, Times) ->
    Impls1 = build_index(Distance, Point, Size, Seed, [{naive, impl(naive)}|Impls]),
    Parent = self(),
    Validators =
        [ begin
              Ref = make_ref(),
              Pid =
                  spawn_link(
                    fun Loop() ->
                            receive
                                {point, P} ->
                                    Parent ! {nearest, Ref, P, Search(Distance, P, Limit, not_found, Index)},
                                    Loop();
                                quit ->
                                    ok
                            end
                    end),
              {Name, Ref, Pid}
          end
          || {Name, Search, Index} <- Impls1],
    validate_loop(Point, Validators, Times),
    [ begin Pid ! quit end || {_, _, Pid} <- Validators],
    io:format("~n", []),
    ok.

validate_loop(_, _, 0) ->
    ok;
validate_loop(Point, Validators, N) ->
    P = Point(),
    [ Pid ! {point, P} || {_, _, Pid} <- Validators],
    validate_result(P, Validators),
    validate_loop(Point, Validators, N-1).

validate_result(P, [{naive, Ref, _}|Rest]) ->
    receive
        {nearest, Ref, P, V} ->
            validate_result(P, V, Rest)
    end,
    io:format(".", []).

validate_result(_, _, []) ->
    ok;
validate_result(P, V, [{Name, Ref, _}|Rest]) ->
    receive
        {nearest, Ref, P, V} ->
            ok;
        {nearest, Ref, P, V1} ->
            io:format("BAD result from ~s for ~p~nEXPECTED:~n ~p~nGOT:~n ~p~n", [Name, P, V, V1])
    end,
    validate_result(P, V, Rest).

generate_points(_, 0) ->
    [];
generate_points(Point, N) ->
    [{Point(), N}|generate_points(Point, N-1)].


run({Distance, Point, Limit, Impls}, Seed, Size, Time) ->
    Impls1 = build_index(Distance, Point, Size, Seed, Impls),
    Seed1 = rand:export_seed(),

    mybench:run(
      [ {Name,
         {
          fun() -> Search(Distance, Point(), Limit, not_found, Index) end,
          fun() -> rand:seed(Seed1) end
         }
        }
        || {Name, Search, Index} <- Impls1 ],
      Time,
      #{
        base =>
            {
             fun() -> Point() end,
             fun() -> rand:seed(Seed1) end
            }
       }).

build_index(Distance, Point, Size, Seed, Impls) ->
    rand:seed(Seed),
    io:format("Started with seed ~p~n", [Seed]),
    io:format("Generating points~n", []),
    {Time, Points} =timer:tc(fun () -> [ {Point(), I} || I <- lists:seq(1, Size) ] end),
    io:format("Finished in ~p us~n", [Time]),

    io:format("Building index~n", []),
    Parent = self(),
    Builders =
        [begin
             Ref = make_ref(),
             spawn_link(
               fun() ->
                       {T, Index} = timer:tc(fun () -> FromList(Distance, Points) end),
                       Parent ! {Name, Ref, Index, T}
               end),
             {Name, Ref}
         end
         || {Name, {FromList, _}} <- Impls ],

    Indices =
        [ receive
              {Name, Ref, Index, T} ->
                  io:format("~s finished in ~p us~n", [Name, T]),
                  Index
          end
          || {Name, Ref} <- Builders ],
    [{Name, Search, Index}
     || {{Name, {_, Search}}, Index} <- lists:zip(Impls, Indices)].


great_circle_distance(Lat1, Lat2, Lon0) ->
    Lat = math:sin((Lat2 - Lat1)/2.0),
    Lon = math:sin(Lon0/2.0),
    A = math:sqrt(Lat*Lat + math:cos(Lat1)*math:cos(Lat2)*Lon*Lon),
    math:asin(min(1.0, A)).
