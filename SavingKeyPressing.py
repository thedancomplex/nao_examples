import csv
import pyglet
from psychopy import core


import NaoGetKey as nao


window = pyglet.window.Window(fullscreen=True)

#GoalResult = ['R','L','R','R','L','R','R','L','L','R']
GoalResult = ['R','L','R','R','L']

RcvResult = []
RT =[]

indx = 0
nao.NaoDirection(GoalResult[indx])
desclock = core.Clock()  # get current time
t1 = desclock.getTime()

@window.event
def on_key_press(key, modifiers):
    global t1
    rt = desclock.getTime() - t1

    if (key == pyglet.window.key.LEFT):
        RcvResult.append('R')
        RT.append(rt)
        t1 = desclock.getTime()
    elif (key == pyglet.window.key.RIGHT):
        RcvResult.append('L')
        RT.append(rt)
        t1 = desclock.getTime()

    global indx
    indx = indx + 1
    if indx < 5:
        nao.NaoDirection(GoalResult[indx])


pyglet.app.run()
#nao.NaoDirection("Quit")

while len(RcvResult) < 5:
    RcvResult.append(999)
    RT.append(999)


def compare_result(ObsRslt,GoalRslt):
    result = []
    for x in range(0,5):
        if (ObsRslt[x]) == (GoalRslt[x]):
            result.append(1)
        else:
            result.append(0)

    return result


def saving_data(data1,data2,data3,data4):
    w, h = 4, 5
    matrix_data = [[0 for x in range(w)] for y in range(h)]
    for x in range(0,5):
        for y in range(0,4):
            if y == 0:
                matrix_data[x][y] = data1[x]
            elif y == 1:
                matrix_data[x][y] = data2[x]
            elif y == 2:
                matrix_data[x][y] = data3[x]
            elif y == 3:
                matrix_data[x][y] = data4[x]

    return matrix_data


def saving_excel(data):

    with open('names.csv', 'w') as csvfile:
        fieldnames = ['Goal_result', 'Observed_result', 'Accuracy', 'RT']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for x in range(0,5):
            writer.writerow({'Goal_result': data[x][0], 'Observed_result': data[x][1], 'Accuracy': data[x][2], 'RT': data[x][3]})


accuracy = compare_result(RcvResult, GoalResult)
result = saving_data(GoalResult,RcvResult,accuracy,RT)
#print(RcvResult)
saving_excel(result)
