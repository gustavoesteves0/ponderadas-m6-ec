from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import threading
import cv2
import time
import io

app = FastAPI()

class RobotControlNode(Node):

    def __init__(self):
        super().__init__('robot_control_node')
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.running = True

        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_srv = self.create_service(Empty, 'stop_robot', self.stop_robot_service)
        self.shutdown_srv = self.create_service(Empty, 'shutdown_robot', self.shutdown_robot_service)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def stop_robot_service(self, request, response):
        self.stop_robot()
        return response

    def shutdown_robot_service(self, request, response):
        self.shutdown_robot()
        return response

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().info("Parada de emergência ativada!")

    def shutdown_robot(self):
        self.running = False
        self.get_logger().info("Desligando o robô...")
        rclpy.shutdown()

    def set_velocity(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular

    def timer_callback(self):
        if not self.running:
            self.destroy_node()
            return

        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.velocity_pub.publish(msg)

rclpy.init(args=None)
node = RobotControlNode()
thread = threading.Thread(target=rclpy.spin, args=(node,))
thread.start()

app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/", response_class=HTMLResponse)
def read_root():
    try:
        with open("./templates/index.html", "r") as file:
            html_content = file.read()
        return HTMLResponse(content=html_content, status_code=200)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/forward")
def forward():
    node.set_velocity(2.0, 0.0)
    return {"status": "moving forward"}

@app.get("/backward")
def backward():
    node.set_velocity(-2.0, 0.0)
    return {"status": "moving backward"}

@app.get("/left")
def left():
    node.set_velocity(0.0, 2.0)
    return {"status": "turning left"}

@app.get("/right")
def right():
    node.set_velocity(0.0, -2.0)
    return {"status": "turning right"}

@app.get("/stop")
def stop():
    node.stop_robot()
    return {"status": "stopped"}

@app.get("/shutdown")
def shutdown():
    node.shutdown_robot()
    return {"status": "shutting down"}

@app.get("/stream")
def stream():
    return StreamingResponse(generate_video(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/latency")
def get_latency():
    latency = estimate_latency()
    return JSONResponse(content={"latency": latency})

def generate_video():
    cap = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        end_time = time.time()
        latency = end_time - start_time

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n'
               b'X-Latency: ' + str(latency).encode() + b'\r\n\r\n')

def estimate_latency():
    cap = cv2.VideoCapture(0)
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        return None
    end_time = time.time()
    latency = end_time - start_time
    cap.release()
    return latency

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
