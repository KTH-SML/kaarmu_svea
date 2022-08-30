# Will reorder PYTHONPATH to use Python 3 with higher priority
#
# You need to source config.sh first

WORKSPACE='/city_lmpc_ws'
export PYTHONPATH=$WORKSPACE/devel/lib/python3/dist-packages:/opt/ros/melodic/lib/python3/dist-packages:$WORKSPACE/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:
