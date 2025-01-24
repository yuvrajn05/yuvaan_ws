from flask import Flask, jsonify
import rospy
from yuvaan_controller.msg import yuvaan
import multiprocessing

# Global variable to store motor speed data
motor_data = {}

# ROS Callback to update motor data
def motor_command_callback(msg):
    global motor_data
    motor_data = {
        'mode': msg.mode,
        'vel_linear_x': msg.vel_linear_x,
        'vel_angular_z': msg.vel_angular_z,
        'ra_1': msg.ra_1,
        'ra_2': msg.ra_2,
        'ra_3': msg.ra_3,
        'ra_4': msg.ra_4,
        'ra_5': msg.ra_5,
        'ra_6': msg.ra_6,
        'ba_1': msg.ba_1,
        'ba_2': msg.ba_2,
        'ba_3': msg.ba_3,
        'ba_4': msg.ba_4,
        'ba_5': msg.ba_5,
        'ba_6': msg.ba_6
    }
    print("Updated Motor Data:", motor_data)


# Initialize ROS node
def init_ros_node():
    rospy.init_node('flask_ros_node', anonymous=True)
    rospy.Subscriber('motor_command', yuvaan, motor_command_callback)
    rospy.spin()

# Define Flask route to fetch motor data
app = Flask(__name__)

@app.route('/motor_data', methods=['GET'])
def get_motor_data():
    print("Current Motor Data:", motor_data)
    return jsonify(motor_data)


# Start Flask and ROS nodes in separate processes
if __name__ == '__main__':
    # Create a new process for the ROS node
    ros_process = multiprocessing.Process(target=init_ros_node)
    ros_process.start()

    # Start Flask in the main process
    app.run(debug=True, host='0.0.0.0', port=5000)

    # Wait for ROS process to finish (it shouldn't unless the node shuts down)
    ros_process.join()
