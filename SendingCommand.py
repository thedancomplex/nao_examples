import sys
import pyglet
import NaoGetKey as nao


window = pyglet.window.Window(fullscreen=True) 

@window.event
def on_key_press(key, mpdifiers):
	if key == pyglet.window.key.LEFT:
            nao.NaoDirection("L")
        elif key == pyglet.window.key.RIGHT:
            nao.NaoDirection("R")
	
pyglet.app.run()




