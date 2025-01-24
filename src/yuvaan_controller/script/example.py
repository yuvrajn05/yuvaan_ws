from flask import Flask, jsonify, request
import rospy
from yuvaan_controller.msg import yuvaan

app = Flask(__name__)

# Global variable for motor_data
motor_data = {}

@app.route('/set_motor_values', methods=['POST'])
def set_motor_values():
    global motor_data
    data = request.get_json()  # Get the JSON data sent from the frontend
    if data:
        motor_data.update(data)  # Update motor_data with new values from the form
        rospy.loginfo(f"Updated motor data: {motor_data}")
        # Now, publish to a ROS topic with updated values
        pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)
        msg = yuvaan()
        msg.vel_linear_x = motor_data.get('vel_linear_x', 0)
        msg.vel_angular_z = motor_data.get('vel_angular_z', 0)
        # Fill the rest of the message fields
        pub.publish(msg)
        return jsonify({"status": "success", "motor_data": motor_data}), 200
    else:
        return jsonify({"status": "error", "message": "No data provided"}), 400

@app.route('/get_motor_values', methods=['GET'])
def get_motor_values():
    return jsonify(motor_data)

if __name__ == '__main__':
    rospy.init_node('flask_ros_node', anonymous=True)
    app.run(debug=True, use_reloader=False)
