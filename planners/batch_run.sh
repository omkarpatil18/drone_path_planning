#!/bin/bash

# export PLAN_FREQ="4"
# export HORIZON_LEN="10"
# export SCALE="2"
# python astar.py;
# python mikami.py;

# export PLAN_FREQ="4"
# export HORIZON_LEN="10"
# export SCALE="3"
# python astar.py;
# python mikami.py;

# export PLAN_FREQ="4"
# export HORIZON_LEN="10"
# export SCALE="5"
# python astar.py;
# python mikami.py;



export PLAN_FREQ="49"
export HORIZON_LEN="100"
export SCALE="2"
python astar.py;
python mikami.py;

export PLAN_FREQ="49"
export HORIZON_LEN="100"
export SCALE="3"
python astar.py;B
python mikami.py;

export PLAN_FREQ="49"
export HORIZON_LEN="100"
export SCALE="5"
python astar.py;
python mikami.py;



