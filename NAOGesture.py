robotIP = "169.254.195.149"
PORT = 9559

import sys
from naoqi import ALProxy
import pyglet
window = pyglet.window.Window(width=1920, height=1080)

motionProxy = ALProxy("ALMotion", robotIP , PORT)

def obsresult_aray():
    @window.event
    def on_key_press(key, mpdifiers):
        if (key == pyglet.window.key.LEFT):
			motionProxy.openHand('RHand')
			motionProxy.closeHand('RHand')
        elif (key == pyglet.window.key.RIGHT):
			motionProxy.openHand('LHand')
			motionProxy.closeHand('LHand')
    pyglet.app.run()
    
obsresult_aray()