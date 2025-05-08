#!/usr/bin/env python3

import rospy
from cola2_msgs.msg import Setpoints
from cola2_msgs.srv import Intensitylightcamfrequency, IntensitylightcamfrequencyRequest

class CombinedPublisher:
    def __init__(self):
        # Create a NodeHandle (implicitly via rospy)
        self.nh = rospy

        # Create publisher for Setpoints
        self.setpoints_publisher = self.nh.Publisher("/stm32/Setpoints", Setpoints, queue_size=10)

        # Create service client for IntensityLightCamFrequency
        self.intensity_service_client = rospy.ServiceProxy("cam_trigger_service", Intensitylightcamfrequency)

        # Wait for the service to be available
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("cam_trigger_service", timeout=1.0)
                break
            except rospy.ROSException:
                rospy.logwarn("Waiting for cam_trigger_service...")

        # Create timers to send data periodically
        self.setpoints_timer = rospy.Timer(
            rospy.Duration(0.5), self.publish_setpoints
        )

        self.intensity_timer = rospy.Timer(
            rospy.Duration(1.0), self.send_intensity_request
        )

        rospy.loginfo("CombinedPublisher Initialized.")

    def publish_setpoints(self, event):
        # Create Setpoints message
        msg = Setpoints()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frame"
        msg.setpoints = [-0.0196, -0.0196, -0.0196, -0.0196, -0.0196, -0.0196]

        # Publish message
        self.setpoints_publisher.publish(msg)
        rospy.loginfo("Published Setpoints: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                      msg.setpoints[0], msg.setpoints[1], msg.setpoints[2],
                      msg.setpoints[3], msg.setpoints[4], msg.setpoints[5])

    def send_intensity_request(self, event):
        # Create service request for IntensityLightCamFrequency
        req = IntensitylightcamfrequencyRequest()
        req.camfrec = 20        # Integer value 10
        req.intensity = 0.5       # Normalized between 0 and 1

        # Call the service
        try:
            self.intensity_service_client.call(req)
            rospy.loginfo("Sending Request: camfrec=%d, intensity=%.2f",
                          req.camfrec, req.intensity)
        except rospy.ServiceException:
            rospy.logerr("Service call failure: cam_trigger_service")

if __name__ == '__main__':
    rospy.init_node('combined_publisher')
    node = CombinedPublisher()
    rospy.spin()
