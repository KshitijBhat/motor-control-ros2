#import board
#import digitalio
import machine

#A_pin = digitalio.DigitalInOut(board.GP16)
#B_pin = digitalio.DigitalInOut(board.GP17)

#A_pin.direction = digitalio.Direction.INPUT
#B_pin.direction = digitalio.Direction.INPUT

#A_pin.pull = digitalio.Pull.UP
#B_pin.pull = digitalio.Pull.UP

A_pin = machine.Pin(16, machine.Pin.IN)
B_pin = machine.Pin(17, machine.Pin.IN)

outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
last_AB = 0b00
counter = 0


while True:
    A = A_pin.value()
    B = B_pin.value()
    #print(A,B)
    current_AB = (A << 1) | B
    position  = (last_AB << 2) | current_AB
    counter += outcome[position]
    last_AB = current_AB
    print(counter)