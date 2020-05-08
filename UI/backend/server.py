"""
Purpose: Bridge between React app and ROS

        #### CONNECTION SUMMARY ####

        Navigation Stack: TCP + ROS
         Manual Traverse: WebRTC + ROS
Robotic Arm Manipulation: WebRTC + ROS

Useful Links:
Python WebRTC Example: https://github.com/aiortc/aiortc/tree/master/examples/server
"""
from flask import Flask

app = Flask(__name__, static_folder="../rover-ui/build", static_url_path="/")


@app.route("/")
def index():
    return app.send_static_file("index.html")


app.run(debug=True)
