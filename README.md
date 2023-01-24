# Heterogenous Capacitated Vehicle Routing Problem solver

This is a solver for the Heterogenous Capacitated Vehicle Routing Problem (HCVRP) using the [ic constraint solver](https://www.cs.nmsu.edu/~ipivkina/ECLIPSE/doc/bips/lib/ic/index.html). The solver is written in [ECLiPSe Prolog](https://eclipseclp.org/).

## Usage

The file `hcvrp_data.pl` contains the problem data. The `vehicles` predicate contains the vehicle capacities, and the `clients` predicate contains the clients, where each client is represented by a tuple of the form `(demand, x, y)`. A convention is made that the starting point is at `(0, 0)`.

Once you have edited this file with your data, you can run the solver by executing the following command:

```bash
$ eclipse-clp
[eclipse 1]: [hcvrp].
...
Yes
[eclipse 2]: hcvrp(10, 5, 100, S, C, T).
...
S = [[5, 1], [8, 4, 3], [7, 6, 9], [], [10, 2]]
C = 1032261
T = 99.819624948
Yes
```

The first argument is the number of clients to use from the clients list, the second argument is the number of vehicles to use from the vehicles list, the third is a timeout for the solver in seconds, and the last three are the output variables. The output variables are the solution, the cost of the solution, and the time it took to find the solution. The solver will find multiple solutions, and the best solution it can find in `timeout` seconds will be returned.
