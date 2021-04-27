export DISPLAY=:0       # [int] display to show gui
export DELETE_BUILD=0   # [int] delete ros2 build stuff

export TURN_ACELERATION_FC=0.5      # [float] deceleration/acceleration factor in turn profile
export TURN_CRTL_POINTS=50          # [int] control points to discrite the trajectory in turn profile
export TURN_PRINT_WAYPOINT=0        # [bool] 1: Enable/ 0: Disable control points to discrite the trajectory in turn profile
export TURN_TIME=2.0               # [int][sec] time to perform a turn 

export FORWARE_ACELERATION_FC=0.3   # [float] deceleration/acceleration factor in speed profile
export FORWARE_CRTL_POINTS=100      # [int] control points to discrite the trajectory in speed profile
export FORWARE_PRINT_WAYPOINT=0     # [bool] 1: Enable/ 0: Disable control points to discrite the trajectory in speed profile

export BOT_INITIAL_YAW=90            # [float][degress] robots yaw angle
export BOT_INITIAL_X=917            # [int][pixels] x axis initial coordinate
export BOT_INITIAL_Y=1047           # [int][pixels] y axis initial coordinate
