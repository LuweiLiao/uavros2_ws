std_msgs/Header header

float64 latitude     # latitude in degree
float64 longitude    # longitude in degree
float64 altitude     # above start-up point
float64 heading      # GPS heading
# ROS 2 interface definitions require lower_snake_case field identifiers.
# These are the ROS 2 spellings of the ROS 1 maxSpeed and maxAcc fields; the
# field order and wire meaning are unchanged.
float64 max_speed    # maximum approach speed
float64 max_acc      # maximum approach accelerations
