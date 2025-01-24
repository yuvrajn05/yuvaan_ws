from flask import Flask, render_template, jsonify, Response, stream_with_context
import subprocess
import time
import json

app = Flask(__name__)

@app.route("/")
def index():
    # Automatically open terminal with ROS and other commands when the page loads
    try:
        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "roscore; exec bash"],
            cwd="/home/jetson"
        )
        time.sleep(15)  # Allow some time for roscore to start

        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "rosrun joy joy_node; exec bash"],
            cwd="/home/jetson"
        )
        time.sleep(1)

        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "rosrun yuvaan_controller controller.py; exec bash"],
            cwd="/home/jetson"
        )
        time.sleep(1)

        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "python3 ~/yuvaan_ws/src/yuvaan_controller/script/zed.py; exec bash"],
            cwd="/home/jetson"
        )
        time.sleep(1)

        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "rosrun yuvaan_controller zed_control.py; exec bash"],
            cwd="/home/jetson"
        )
        time.sleep(1)
        
    except Exception as e:
        print(f"Error opening terminal: {e}")

    # Render the HTML page with buttons
    return render_template("index.html")

@app.route("/stream_command/<command>")
def stream_command(command):
    commands = {
    "ping_localhost": 'rostopic pub /joy sensor_msgs/Joy "{axes: [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]}" && stdbuf -oL rostopic echo /motor_command',
    "trace_route": "stdbuf -oL rostopic echo /motor_command",  
}
    if command in commands:
        def generate():
            process = subprocess.Popen(commands[command], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
            values = {
                'ra_1': None,
                'ra_2': None,
                'ra_3': None,
                'ra_4': None,
                'ra_5': None,
                'ra_6': None,
                'ba_1': None,
                'ba_2': None,
                'ba_3': None,
                'ba_4': None,
                'ba_5': None,
                'ba_6': None,
                'vel_linear_x': None,
                'vel_angular_z': None,
                'mode': None
            }

            while True:
                output = process.stdout.readline()
                if output == "" and process.poll() is not None:
                    break
                if output:
                    # Extract ra_ and ba_ values
                    for key in range(1, 7):  # Check ra_1 to ra_6
                        ra_key = f'ra_{key}'
                        if ra_key in output:
                            values[ra_key] = output.split(ra_key + ':')[-1].strip().split()[0]

                    for key in range(1, 7):  # Check ba_1 to ba_6
                        ba_key = f'ba_{key}'
                        if ba_key in output:
                            values[ba_key] = output.split(ba_key + ':')[-1].strip().split()[0]

                    # Extract vel_linear_x, vel_angular_z, and mode
                    if 'vel_linear_x:' in output:
                        values['vel_linear_x'] = output.split('vel_linear_x:')[-1].strip().split()[0]
                    if 'vel_angular_z:' in output:
                        values['vel_angular_z'] = output.split('vel_angular_z:')[-1].strip().split()[0]
                    if 'mode:' in output:
                        values['mode'] = output.split('mode:')[-1].strip().split()[0]

                    # Prepare the output data
                    data = {
                        'output': output.strip(),
                        **values  # Unpack all values
                    }
                    yield f"data: {json.dumps(data)}\n\n"

            # Handle any errors from the process
            stderr_output = process.stderr.read()
            if stderr_output:
                yield f"data: {json.dumps({'output': stderr_output.strip(), **values})}\n\n"

        return Response(stream_with_context(generate()), mimetype="text/event-stream")
    else:
        return jsonify({"output": "Invalid command."})
    
if __name__ == "__main__":
    app.run(debug=True)
