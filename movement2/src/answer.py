#!/usr/bin/env python
#It is assumed that the main package will be here.
from i_like_to_move_it import Move_It
import rospy

x = Move_It()

x.setLinear(-2,-2,4, 1)
x.activate()

rospy.spin()
