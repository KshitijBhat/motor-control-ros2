from machine import Pin, PWM
import time

SPEED_FACTOR = 1
ENCODER_TICKS_ONE_ROT = 660

IN1 = Pin(14, Pin.OUT)
IN2 = Pin(13, Pin.OUT)

ENA = PWM(Pin(15))
ENA.freq(1000)

def Enc_Handler(Source):
    global Enc_Counter
    global Qtr_Cntr
    global Enc_A_State
    global Enc_A_State_old
    global Enc_B_State
    global Enc_B_State_old
    global error
    #s = str(Source)  #useful for debugging and setup to see which pin triggered interupt
    #print(s[4:6])
        
    Enc_A_State = Enc_Pin_A.value()  #Capture the current state of both A and B
    Enc_B_State = Enc_Pin_B.value()
    if Enc_A_State == Enc_A_State_old and Enc_B_State == Enc_B_State_old:  #Probably 'bounce" as there was a trigger but no change
        error += 1  #add the error event to a variable - may by useful in debugging
    elif (Enc_A_State == 1 and Enc_B_State_old == 0) or (Enc_A_State == 0 and Enc_B_State_old == 1):
        # this will be clockwise rotation
        # A   B-old
        # 1 & 0 = CW rotation
        # 0 & 1 = CW rotation
        Enc_Counter += 1  #Increment counter by 1 - counts ALL transitions
        Qtr_Cntr = round(Enc_Counter/4)  #Calculate a new 1/4 counter value
    elif (Enc_A_State == 1 and Enc_B_State_old == 1) or (Enc_A_State == 0 and Enc_B_State_old == 0):
        # this will be counter-clockwise rotation
        # A   B-old
        # 1 & 1 = CCW rotation
        # 0 & 0 = CCW rotation
        Enc_Counter -= 1 # Decrement counter by 1 - counts ALL transitions
        Qtr_Cntr = round(Enc_Counter/4)  #Calculate a new 1/4 counter value
    else:  #if here, there is a combination we don't care about, ignore it, but track it for debugging
        error += 1
    Enc_A_State_old = Enc_A_State     # store the current encoder values as old values to be used as comparison in the next loop
    Enc_B_State_old = Enc_B_State  

#Configure the A channel and B channel pins and their associated interrupt handing
Enc_Pin_A = Pin(16,Pin.IN,Pin.PULL_DOWN)
Enc_Pin_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Enc_Handler)
Enc_Pin_B = Pin(17,Pin.IN,Pin.PULL_DOWN)
Enc_Pin_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Enc_Handler)

#Preset some variables to useful and known values
Enc_A_State_old = Enc_Pin_A.value()
Enc_B_State_old = Enc_Pin_B.value()
last_Enc_Counter = 0
Enc_Counter = 0
Last_Qtr_Cntr = 0
Qtr_Cntr = 0
error = 0

def motor_command(speed):

    if speed>0:
        IN1.low()  #spin forward
        IN2.high()
    elif speed<0:
        IN1.high()  #spin backward
        IN2.low()
    else:
        IN1.low()  #stop
        IN2.low()

    ENA.duty_u16(SPEED_FACTOR*abs(speed))



def read_encoder():
    global Qtr_Cntr
    global Last_Qtr_Cntr
    if Qtr_Cntr != Last_Qtr_Cntr:  # if the Qtr_Cntr changed since last time, print the counter value
        Last_Qtr_Cntr = Qtr_Cntr   # Update the variable to the current state
        print(Qtr_Cntr*360/ENCODER_TICKS_ONE_ROT)
    return Qtr_Cntr*360/ENCODER_TICKS_ONE_ROT

if __name__ == "__main__":
    motor_command(0)
    time.sleep(5)
    
    # while True:
    #     angle = read_encoder()
    #     if angle > 360 or angle < -360:
    #         motor_command(0)
    #         break
    #     else:
    #         motor_command(20000)



