#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from vehicle_msgs.msg import TaskSet, Task

def create_and_publish_taskset(num_tasks):
    # Initialize the ROS node
    rospy.init_node('taskset_publisher', anonymous=True)
    
    # Create a publisher for the TaskSet message
    pub = rospy.Publisher('task_info', TaskSet, queue_size=10)
    
    # Create the TaskSet message
    taskset_msg = TaskSet()
    
    # Populate the header
    taskset_msg.header = Header()
    taskset_msg.header.stamp = rospy.Time.now()
    taskset_msg.header.frame_id = "base_link"
    
    # Populate the tasks
    for i in range(num_tasks):
        task = Task()
        task.vehicle_id = i
        if  i == 0:
            task.user_preferred_behavior = -1  # Example value
            task.user_desired_behavior = -1    # Example value
            task.user_desired_speed = 20.0    # Example value
            task.target_lane = 2      
        elif  i == 2:
            task.user_preferred_behavior = -1  # Example value
            task.user_desired_behavior = -1    # Example value
            task.user_desired_speed = 20.0    # Example value
            task.target_lane = 4
        else: 
            task.user_preferred_behavior = -1  # Example value
            task.user_desired_behavior = -1    # Example value
            task.user_desired_speed = 20.0    # Example value
            task.target_lane = 3              # Example value
        taskset_msg.tasks.append(task)
    
    # Publish the message
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pub.publish(taskset_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        num_tasks = 9 #int(input("Enter the number of tasks: "))
        create_and_publish_taskset(num_tasks)
    except rospy.ROSInterruptException:
        pass

