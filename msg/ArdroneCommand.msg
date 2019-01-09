Header header

# New order to send to drone
int8 command
int8 OFF=0      # land (emergency stop)
int8 STANDBY=1  # takeoff and wait
int8 REACH=2    # fly to specified 'pose' (in frame specified in header), then standby
int8 FIND=3     # fly and find target located at specified 'pose' within 'precision' radius, then standby
int8 TRACK=4    # fly, find and follow target located at specified 'pose' within 'precision' radius
int8 LAND=5     # land on currently tracked target

# Target pose [m]
# Optionnal, used by commands REACH, FIND and TRACK
geometry_msgs/Point position

# Precision of the target location [m]
# Optionnal, used by commands FIND and TRACK
float64 precision
