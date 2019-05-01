#!/usr/bin/env python
# license removed for brevity

import rospy

from geometry_msgs.msg import PoseStamped


def movebase_client():
	goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
	rospy.sleep(1)
	print("subscribed!!!")
	goal = PoseStamped()

	goal.header.seq = 1
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"

	goal.pose.position.x = 0.39499989152
	goal.pose.position.y = 0.0
	goal.pose.position.z = 0.0

	goal.pose.orientation.x = 0.0
	goal.pose.orientation.y = 0.0
	goal.pose.orientation.z = 0.0024251765401
	goal.pose.orientation.w = 0.999991023365

	goal_publisher.publish(goal)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
