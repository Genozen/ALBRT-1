"""
        Auth: Made Autonomous 02/12/2024
"""


from gpiozero import Motor
from time import sleep
import curses


frmotor = Motor(forward = 6, backward = 5)
flmotor = Motor(forward = 16, backward = 12)
blmotor = Motor(forward = 21, backward = 20)
brmotor = Motor(forward = 26, backward = 19)

def slide_left():
	print('slide left combo:')
	forward_left()
	reverse_left()

def slide_right():
	print('slide right combo:')
	forward_right()
	reverse_right()

def forward_right():
	print('forward right')
	flmotor.forward()
	brmotor.forward()

def reverse_left():
	print('reverse left')
	flmotor.backward()
	brmotor.backward()

def forward_left():
	print('forward left')
	frmotor.forward()
	blmotor.forward()

def reverse_right():
	print('reverse right')
	frmotor.backward()
	blmotor.backward()


def rotate_left():
	print('rotate left')
	frmotor.forward()
	brmotor.forward()
	flmotor.backward()
	blmotor.backward()

def rotate_right():
	print('rotate right')
	frmotor.backward()
	brmotor.backward()
	flmotor.forward()
	blmotor.forward()

def forward():
	print('forward')
	frmotor.forward()
	brmotor.forward()
	flmotor.forward()
	blmotor.forward()

def reverse():
	print('reverse')
	frmotor.backward()
	brmotor.backward()
	flmotor.backward()
	blmotor.backward()

def stop():
	print('stop')
	frmotor.stop()
	brmotor.stop()
	flmotor.stop()
	blmotor.stop()


actions = {
	curses.KEY_UP: forward,
	curses.KEY_DOWN: reverse,
	curses.KEY_LEFT: rotate_left,
	curses.KEY_RIGHT: rotate_right,
	curses.KEY_IC: forward_left,
	curses.KEY_DC: reverse_left,
	curses.KEY_PPAGE: forward_right,
	curses.KEY_NPAGE: reverse_right,
	10: slide_right,   #key 10 is enter
	curses.KEY_BACKSPACE: slide_left
	# curses.KEY_DOWN: 
}

def main(window):
	next_key = None
	# auto_run = False
	while(1):
		curses.halfdelay(1)
		if next_key is None:
			key = window.getch()
		else:
			key = next_key
			next_key = None
		if key != -1:
			#key pressed
			curses.halfdelay(3)
			print(f'curr_key: {key}')
			action = actions.get(key)
			if action is not None:
				action()
			next_key = key
			while next_key == key:
				next_key = window.getch()
			#key released
			stop()

if __name__ == "__main__":
#	print('hello world')
#	try:
#		while(1):
#			frmotor.forward()
#			flmotor.forward()
#			brmotor.forward()
#			blmotor.forward()
#			sleep(2)
#			frmotor.backward()
#			flmotor.backward()
#			brmotor.backward()
#			blmotor.backward()
#			sleep(2)
#	except KeyboardInterrupt:
#		pass

	curses.wrapper(main)
