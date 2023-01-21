from flask import Flask
from flask_cors import cross_origin
from flask_cors import CORS
import threading

import plutoComms

c = plutoComms.COMMS()
if c is None:
    print("c is none")
else:
    print("c is not none")

readThread = threading.Thread(target=c.read)
writeThread = threading.Thread(target=c.write)
writeThread.start()
readThread.start()

# Function to execute commands based on event triggered
def calcOp(text):

    global c

    try:
        eval("c." + text + "()")
        return "True"
    except Exception as e:
        print(e)
        return "NULL"


def droneParameters():
    global c
    params = c.paramsReceived
    print(c.paramsReceived)
    data = {}
    data["rcThrottle"] = params["rcThrottle"]
    data["Roll"] = params["Roll"]
    data["Pitch"] = params["Pitch"]
    data["Yaw"] = params["Yaw"]

    # printing data on terminal
    print("droneParameter " + str(data))
    return data


app = Flask(__name__)
CORS(app)


@app.route("/controller/<input>")
@cross_origin()
def calc(input):
    return calcOp(input)


@app.route("/drone_param")
@cross_origin()
def droneParamView():
    return droneParameters()


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5001)
