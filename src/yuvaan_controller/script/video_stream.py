import eventlet
eventlet.monkey_patch()  # Ensure this is the first line
from flask import Flask, Response, jsonify, request, render_template_string
import cv2
from flask_socketio import SocketIO, emit
import websocket
import json
import threading

# 2. Initialize Flask app and SocketIO
app = Flask(__name__)
# socketio = SocketIO(app, async_mode='eventlet')
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")


# 3. Global dictionary to store motor data
motor_data = {
    'vel_linear_x': 0,
    'vel_angular_z': 0,
    'mode': 0,
    'ra_1': 0,
    'ra_2': 0,
    'ra_3': 0,
    'ra_4': 0,
    'ra_5': 0,
    'ra_6': 0,
    'ba_1': 0,
    'ba_2': 0,
    'ba_3': 0,
    'ba_4': 0,
    'ba_5': 0,
    'ba_6': 0,
}

# 4. Function to handle WebSocket communication
def update_velocities_via_websocket():
    try:
        print("Attempting to connect to rosbridge WebSocket...")
        ws = websocket.create_connection("ws://localhost:9090", timeout=5)  # Update URL and add timeout
        print("Connected to rosbridge WebSocket.")

        subscribe_msg = {
            "op": "subscribe",
            "topic": "/motor_command",  # Make sure this topic is correct
            "type": "yuvaan_controller/yuvaan",
            "throttle_rate": 0,
            "queue_length": 1
        }

        ws.send(json.dumps(subscribe_msg))
        print("Subscription message sent to /motor_command topic.")

        while True:
            try:
                response = ws.recv()  # Non-blocking WebSocket receive
                if response:
                    data = json.loads(response)
                    print(f"Received data: {data}")
                    if "msg" in data:
                        motor_data.update({
                            "vel_linear_x": data['msg']['vel_linear_x'],
                            "vel_angular_z": data['msg']['vel_angular_z'],
                            "mode": data['msg']['mode'],
                            "ra_1": data['msg']['ra_1'],
                            "ra_2": data['msg']['ra_2'],
                            "ra_3": data['msg']['ra_3'],
                            "ra_4": data['msg']['ra_4'],
                            "ra_5": data['msg']['ra_5'],
                            "ra_6": data['msg']['ra_6'],
                            "ba_1": data['msg']['ba_1'],
                            "ba_2": data['msg']['ba_2'],
                            "ba_3": data['msg']['ba_3'],
                            "ba_4": data['msg']['ba_4'],
                            "ba_5": data['msg']['ba_5'],
                            "ba_6": data['msg']['ba_6'],
                        })
                        print(f"Motor data updated: {motor_data}")

                        # 5. Use Flask app context when emitting data
                        with app.app_context():  # Push the app context
                            socketio.emit('motor_data_update', motor_data)

            except websocket.WebSocketTimeoutException:
                # Handle timeout if no message is received
                pass
            except Exception as e:
                print(f"Error while receiving WebSocket data: {e}")
                break

    except Exception as e:
        print(f"Failed to connect to rosbridge WebSocket: {e}")

# Start WebSocket data thread
websocket_thread = threading.Thread(target=update_velocities_via_websocket, daemon=True)
websocket_thread.start()


def generate_video(video_device_path):
    """Stream video from the specified device."""
    cap = cv2.VideoCapture(video_device_path)
    
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video device {video_device_path}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        _, buffer = cv2.imencode('.jpg', frame)
        frame_data = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
    
    cap.release()

@app.route('/video/<int:camera_id>')
def stream_video(camera_id):
    """
    Stream video from the specified video device.
    :param camera_id: The camera device ID (e.g., 0, 1, 2)
    """
    video_device_path = f"/dev/video{camera_id}"
    return Response(
        generate_video(video_device_path),
        content_type='multipart/x-mixed-replace; boundary=frame'
    )

# @app.route('/')
# def index():
#     """Serve the HTML page displaying multiple video feeds."""
#     camera_ids = [2, 2, 2,2,2,2,2,2,2]  # Adjust this list for available camera IDs
#     return render_template_string("""
#     <!DOCTYPE html>
#     <html lang="en">
#     <head>
#         <meta charset="UTF-8">
#         <meta name="viewport" content="width=device-width, initial-scale=1.0">
#         <title>Multi-Camera Feeds</title>
#         <style>
#             body {
#                 margin: 0;
#                 font-family: Arial, sans-serif;
#                 background-color: #f5f5f5;
#                 display: grid;
#                 grid-template-columns: 1fr 1fr; /* Two halves: left and right */
#                 height: 100vh; /* Full viewport height */
#             }
#             .left-half {
#                 display: grid;
#                 grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); /* Responsive grid */
#                 gap: 15px;
#                 align-content: start; /* Align items at the top */
#                 padding: 20px;
#                 background-color: #ffffff;
#                 overflow-y: auto; /* Scroll if content overflows */
#                 border-right: 2px solid #ddd; /* Optional visual separation */
#             }
#             .camera-feed {
#                 width: 100%;
#                 aspect-ratio: 16 / 9;
#                 border: 2px solid #333;
#                 border-radius: 8px;
#                 background-color: #000;
#                 overflow: hidden;
#                 box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
#             }
#             .camera-feed img {
#                 width: 100%;
#                 height: 100%;
#                 object-fit: cover;
#             }
#             .camera-title {
#                 text-align: center;
#                 margin: 5px 0;
#                 font-size: 16px;
#                 font-weight: bold;
#                 color: #333;
#             }
#             .right-half {
#                 padding: 20px;
#                 background-color: #f0f0f0;
#                 text-align: center;
#                 color: #555;
#                 overflow-y: auto;
#             }
#             h1 {
#                 text-align: center;
#                 margin: 20px 0;
#                 color: #333;
#             }
#         </style>
#     </head>
#     <body>
#         <!-- Left Half: Multi-Camera Feeds -->
#         <div class="left-half">
#             {% for cam_id in camera_ids %}
#             <div>
#                 <div class="camera-title">Camera {{ cam_id }}</div>
#                 <div class="camera-feed">
#                     <img src="/video/{{ cam_id }}" alt="Camera {{ cam_id }} Feed">
#                 </div>
#             </div>
#             {% endfor %}
#         </div>
        
#         <!-- Right Half: Placeholder for additional content -->
#         <div class="right-half">
#             <h1>Additional Content</h1>
#             <p>Use this space for anything you'd like, such as:</p>
#             <ul>
#                 <li>Live data updates</li>
#                 <li>Control buttons</li>
#                 <li>Statistics or logs</li>
#                 <li>Empty space for customization</li>
#             </ul>
#         </div>
#     </body>
#     </html>
#     """, camera_ids=camera_ids)


@app.route('/')
def index():
    """Serve the HTML page for displaying live motor data."""
    camera_ids = [2, 2, 2,2,2,2,2,2,2]
    return render_template_string("""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Multi-Camera Feeds with Motor Data</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.6.0/socket.io.min.js"></script>
        <style>
            /* Body layout with two halves */
            body {
                margin: 0;
                font-family: Arial, sans-serif;
                display: grid;
                grid-template-columns: 1fr 1fr; /* Left and right halves */
                height: 100vh; /* Full height of the viewport */
            }

            /* Left-half for camera feeds */
            .left-half {
                padding: 20px;
                background-color: #ffffff;
                overflow-y: auto; /* Allow scrolling if camera feeds overflow */
            }

            /* Grid layout for camera feeds */
            .camera-grid {
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); /* Responsive grid */
                gap: 15px; /* Space between grid items */
            }

            /* Camera feed styling */
            .camera-feed {
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
            }

            .camera-frame {
                width: 100%;
                aspect-ratio: 16 / 9; /* Maintain 16:9 ratio */
                border: 2px solid #333;
                border-radius: 8px;
                background-color: #000;
                overflow: hidden;
                box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            }

            .camera-frame img {
                width: 100%;
                height: 100%;
                object-fit: cover; /* Prevent distortion */
            }

            .camera-title {
                margin: 8px 0;
                font-size: 16px;
                font-weight: bold;
                text-align: center;
                color: #555;
            }

            /* Right-half for motor data */
            .right-half {
                padding: 20px;
                background-color: #f0f0f0;
                display: flex;
                flex-direction: column;
                justify-content: flex-start;
                align-items: center;
                color: #333;
                text-align: center;
                overflow-y: auto;
            }

            .motor-data {
                width: 100%;
                max-width: 600px;
                padding: 20px;
                border: 2px solid #333;
                border-radius: 10px;
                background-color: #ffffff;
                box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
            }

            .motor-data h2 {
                margin-bottom: 20px;
                font-size: 20px;
                color: #333;
            }

            .motor-data ul {
                list-style-type: none;
                padding: 0;
                margin: 0;
            }

            .motor-data li {
                font-size: 16px;
                margin: 5px 0;
                display: flex;
                justify-content: space-between;
            }

            /* Add responsiveness for smaller screens */
            @media (max-width: 768px) {
                body {
                    grid-template-columns: 1fr; /* Stack left and right halves */
                }

                .right-half {
                    border-top: 2px solid #ddd; /* Optional visual separation */
                }
            }
        </style>
    </head>
    <body>
        <!-- Left Half: Multi-Camera Feeds -->
        <div class="left-half">
            <h1>Camera Feeds</h1>
            <div class="camera-grid">
                {% for cam_id in camera_ids %}
                <div class="camera-feed">
                    <div class="camera-title">Camera {{ cam_id }}</div>
                    <div class="camera-frame">
                        <img src="/video/{{ cam_id }}" alt="Camera {{ cam_id }} Feed">
                    </div>
                </div>
                {% endfor %}
            </div>
        </div>
        
        <!-- Right Half: Motor Data -->
        <div class="right-half">
            <div class="motor-data">
                <h2>Motor Data</h2>
                <ul id="motor-data-list">
                    <li><span>Linear Velocity (X):</span> <span id="vel_linear_x">0</span></li>
                    <li><span>Angular Velocity (Z):</span> <span id="vel_angular_z">0</span></li>
                    <li><span>Mode:</span> <span id="mode">0</span></li>
                    <li><span>RA 1:</span> <span id="ra_1">0</span></li>
                    <li><span>RA 2:</span> <span id="ra_2">0</span></li>
                    <li><span>RA 3:</span> <span id="ra_3">0</span></li>
                    <li><span>RA 4:</span> <span id="ra_4">0</span></li>
                    <li><span>RA 5:</span> <span id="ra_5">0</span></li>
                    <li><span>RA 6:</span> <span id="ra_6">0</span></li>
                    <li><span>BA 1:</span> <span id="ba_1">0</span></li>
                    <li><span>BA 2:</span> <span id="ba_2">0</span></li>
                    <li><span>BA 3:</span> <span id="ba_3">0</span></li>
                    <li><span>BA 4:</span> <span id="ba_4">0</span></li>
                    <li><span>BA 5:</span> <span id="ba_5">0</span></li>
                    <li><span>BA 6:</span> <span id="ba_6">0</span></li>
                </ul>
            </div>
        </div>

    <script>
        // Connect to the Socket.IO server
        const socket = io.connect('http://localhost:5000');  // Use your server's URL if different

        // Handle connection error
        socket.on("connect_error", (err) => {
            console.error("Connection failed:", err);
        });

        // Listen for motor data updates
        socket.on('motor_data_update', function(data) {
            console.log('Received motor data:', data);

            // Update the displayed values in the HTML
            document.getElementById('vel_linear_x').textContent = data.vel_linear_x;
            document.getElementById('vel_angular_z').textContent = data.vel_angular_z;
            document.getElementById('mode').textContent = data.mode;

            document.getElementById('ra_1').textContent = data.ra_1;
            document.getElementById('ra_2').textContent = data.ra_2;
            document.getElementById('ra_3').textContent = data.ra_3;
            document.getElementById('ra_4').textContent = data.ra_4;
            document.getElementById('ra_5').textContent = data.ra_5;
            document.getElementById('ra_6').textContent = data.ra_6;

            document.getElementById('ba_1').textContent = data.ba_1;
            document.getElementById('ba_2').textContent = data.ba_2;
            document.getElementById('ba_3').textContent = data.ba_3;
            document.getElementById('ba_4').textContent = data.ba_4;
            document.getElementById('ba_5').textContent = data.ba_5;
            document.getElementById('ba_6').textContent = data.ba_6;
        });
    </script>
    </body>
    </html>
    """, camera_ids=camera_ids)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)