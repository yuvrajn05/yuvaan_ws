#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

class DirectionFilter:
    def __init__(self):
        rospy.init_node('direction_filter_node')
        
        # Store the latest distance reading (in centimeters)
        self.current_distance = float('inf')
        
        # Create publisher for filtered direction
        self.filtered_pub = rospy.Publisher(
            'filtered_direction',
            String,
            queue_size=10
        )
        
        # Subscribe to distance topic to have the latest reading available
        rospy.Subscriber(
            'ultrasonic_distance',
            Float32,
            self.distance_callback
        )
        
        # Subscribe to arrow direction topic
        rospy.Subscriber(
            'arrow_direction',
            String,
            self.direction_callback
        )
        
        rospy.loginfo("Direction filter node initialized")
    
    def distance_callback(self, msg):
        """Update the stored distance value (incoming in cm)"""
        self.current_distance = msg.data
    
    def direction_callback(self, msg):
        """Process incoming direction and publish if conditions are met"""
        direction = msg.data
        
        # Convert distance to meters for comparison (200 cm = 2 m)
        distance_in_meters = self.current_distance / 100.0
        
        # Check distance condition
        if distance_in_meters <= 2.0:
            filtered_msg = String()
            filtered_msg.data = direction
            self.filtered_pub.publish(filtered_msg)
            rospy.loginfo(f"Direction: {direction}, Distance: {distance_in_meters:.2f}m - Published")
        else:
            rospy.loginfo(f"Direction: {direction}, Distance: {distance_in_meters:.2f}m - Too far, not published")
    
    def run(self):
        """Main run loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        filter_node = DirectionFilter()
        filter_node.run()
    except rospy.ROSInterruptException:
        pass