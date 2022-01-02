from flask import Flask
from flask import render_template
from flask import jsonify
import os.path

app = Flask(__name__, static_url_path='', static_folder='/walking_robot/v2Good/')


@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"


@app.route("/robot")
def robot_images():
    with open("/walking_robot/v2Good/images/latestTime.txt", "r") as file:
        timestr = file.read().split('\n')

    time1 = timestr[0]
    time2 = timestr[1] if len(timestr) > 1 else time1
    time3 = timestr[2] if len(timestr) > 1 else time1
    time4 = timestr[3] if len(timestr) > 1 else time1
    timePath = time1 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time1) + ".png") else (
        time2 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time2) + ".png") else (
            time3 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time3) + ".png") else (
                time4
            )
        )
    )

    return render_template('robot.html', title='Welcome', time1=time1,
                           time2=time2, time3=time3, timePath=timePath)


@app.route("/time", methods=["GET"])
def getTime():
    with open("/walking_robot/v2Good/images/latestTime.txt", "r") as file:
        timestr = file.read().split()

    time1 = timestr[0]
    time2 = timestr[1] if len(timestr) > 1 else time1
    time3 = timestr[2] if len(timestr) > 1 else time1
    time4 = timestr[3] if len(timestr) > 1 else time1
    timePath = time1 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time1) + ".png") else (
        time2 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time2) + ".png") else (
            time3 if os.path.isfile("/walking_robot/v2Good/images/onpath-" + str(time3) + ".png") else (
                time4
            )
        )
    )

    return jsonify({"time1": time1, "time2": time2, "time3": time3, "timePath": timePath})


@app.route("/log", methods=["GET"])
def getLog():
    with open("/walking_robot/v2Good/log.txt", "r") as file:
        logstr = file.read().replace("\n", "\r\n")

    return logstr


if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")
