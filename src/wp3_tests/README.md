# wp3_tests

## Idea

The wp3 testing framework runs a traffic scenario of connected
vehicles and evaluates a safety criteria againts connectivity/
network properties. The test case scenario is the following:

    In the test case `T`, `N` connected vehicles approach
    an intersection. Each vehicle is required to compute
    the safety-critical function `S` as it is nearing
    the intersection. `S` takes `compute_time` seconds
    to complete and can only run if it has "recent" state
    updates from all its peers. If the vehicle does not
    compute `S` in a time that satisfies the safety condition

    `C = headway > max(latencies) + compute_time`,

    then the vehicle is considered *unsafe*. If all vehicles
    remain *safe* during `T` then `T` is considered successful
    under the set network properties `P`.

    Note: Each vehicle start at the same distance from the
    intersection.

In this package we provide the implementation to run these tests
in both a centralized and decentralized communication architecture.
The tests can be automated over some configuration parameters that
set `P`. Each single test case `T` is logged and saved separately.

To accomplish this we have a master node that coordinates all
agents (vehicles). Read more in Usage.

## Definitions

Master:
    Coordinates the agents to run a test case with specific
    parameters. This is mainly to support automation of the
    test suite.

Vehicle:
    This is the actual "Device Under Test".

Agent:
    Synonym to Vehicle, used when in relation to the master.

Peer:
    Synonym to Vehicle, used when describing inter-vehicle relation.

Test case:
    As described above. This is one single test.

Test suite:
    A collection of test cases. The master starts a test suite.
    When the master exits, the test suite is finished. As of now,
    when a test suite is run, all of the test cases log to a test
    suite directory named after the (ros) time it started.

## Usage

If you are running the 2-agent test under a P2P architecture...

```
# Agent 1
roslaunch wp3_tests VehicleP2P.launch n:=2

# Agent 2
roslaunch wp3_tests VehicleP2P.launch n:=2

# Master
roslaunch wp3_tests MasterP2P.launch n:=2
```

The master coordinates the agents to run all test cases. Each
actual test case is between the agents.

There is only support for `n:=2`, `n:=4` and `n:=8`.

## Details

We need to run too many `T` with different `P` so it infeasible
to manually go in and change each vehicle node (that may be on
different networks) => Use master coordinator and a state machine
for switching between different `T`. See state machine diagram
figure in package.

This scenario requires concurrency by nature (since we have a
distributed system). ROS uses threads by default so it is easier
that we work with threads, instead of `asyncio`.

### Problem 1

One of the first problems I ran into: transitioning can be
dangerous since it takes some time for master to realizes a
transition. This leads to master sending the same transition
request and "confusing" the agent (or you need to implement
proper checks, bleh!). Solving this is quite straight forward,
have a neutral transition request that the master falls back to
(`NONE_T`) after all transitions. A transition should look
something like this for each master/agent:

    Master:     req(XY_T)   wait(ACK_S) req(NONE_T)     wait(Y_S)
                -------------------------------------------------|
                v           ^           v       ^
                -------------------------------------
    Agent(X_S): set(ACK_S)      wait(NONE_T) set(Y_S)

Agent is in `X_S`, the master wants to transition to `Y_S`.
Master first request the transition `XY_T` and waits for a
transition acknowledgement state `ACK_S` from the agent. While
the agent is in `ACK_S` it will perform everything required to
do the transition (also called state switch). When the agent is
done with that, it will not release itself to the new state until
the master confirms with going back to the neutral transition
request `NONE_T`. On the master side, it will switch to `NONE_T`
as soon as it sees that the agent is in `ACK_S`. The master will
also make sure to not try transitioning again until it has
confirmed the agent actually is in the new state `Y_S`.

This robustifies the transition, making sure that both sides are
synchronized during the transition and prevents, e.g., the master
from requesting the same transition twice simply because it doesn't
know when/if the agent has transitioned.

### Problem 2

`ros_abconnect` does not work. I don't really know why. You must
supply a typical abconnect `index.js` file that sets up the
connections. There is only three cases, `n:=2`, `n:=4` and `n:=8`.

The following connections should be made:

master:/transition -> {agents...}:/transition
{agents...}:/outgoing -> {master, agents...}:/incoming

### Problem 3

Thread stuff requires proper resources handling
switch state locks, incoming queues, dicts.

### Problem 4

WRTC has packet size limit.

### Problem 5

Store log data and send to master

### Problem 6

