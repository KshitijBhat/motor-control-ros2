import time

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
Enc_Pin_A = machine.Pin(16,machine.Pin.IN,machine.Pin.PULL_DOWN)
Enc_Pin_A.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=Enc_Handler)
Enc_Pin_B = machine.Pin(17,machine.Pin.IN,machine.Pin.PULL_DOWN)
Enc_Pin_B.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=Enc_Handler)

#Preset some variables to useful and known values
Enc_A_State_old = Enc_Pin_A.value()
Enc_B_State_old = Enc_Pin_B.value()
last_Enc_Counter = 0
Enc_Counter = 0
Last_Qtr_Cntr = 0
Qtr_Cntr = 0
error = 0

# Main program loop which runs continuously 
while True:
    time.sleep(.01)                # Sleep for a moment to slow things down.
    if Qtr_Cntr != Last_Qtr_Cntr:  # if the Qtr_Cntr changed since last time, print the counter value
        Last_Qtr_Cntr = Qtr_Cntr   # Update the variable to the current state
        print(Qtr_Cntr)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#     if Enc_Counter != last_Enc_Counter:  # if the Enc_Cntr changed since las time, print the value
#         last_Enc_Counter = Enc_Counter   #update the variable to the current state
#         print(Enc_Counter, Qtr_Cntr)
#         
