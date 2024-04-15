# NB! Requires root to run
import serial, keyboard, subprocess, time

ser = serial.Serial("/dev/ttyACM0")

# Hack for disabling the keys pressed showing up in the terminal
subprocess.run(["stty", "-echo"], check=True)

last_key = ""

# Delay for throttling the while loop (in ms)
delay = 10
last_delay_at = 0


def millis():
    return(time.time() * 1000)

def get_key_pressed():
    if keyboard.is_pressed("k"):
        return "k"
    elif keyboard.is_pressed("j"):
        return "j"
    elif keyboard.is_pressed("l"):
        return "l"
    elif keyboard.is_pressed("h"):
        return "h"
    elif keyboard.is_pressed("8"):
        return "8"
    elif keyboard.is_pressed("9"):
        return "9"
    else:
        return ""

# Vim keys are superior :)
while True:
    if millis() - last_delay_at >= delay:
        last_delay_at = millis()
        if keyboard.is_pressed("q"):
            print("Exiting...")
            break
        elif get_key_pressed() == "k" and last_key != "k":
            # Drive forward (motor_set 100,100)
            #ser.write(b'000045030564,6433G')
            
            # Drive forward (motor_set 200,200)
            #ser.write(b'000045030596,963DG')
            
            # Drive forward (motor_set 300,300)
            ser.write(b'000045030712C,12CADG')

            # Drive forward (motor_set 500,500)
            #ser.write(b'00004503071F4,1F4B7G')
            last_key = "k"
        elif get_key_pressed() == "j" and last_key != "j":
            # Drive backwards (motor_set -100,-100)
            #ser.write(b'0000450307-64,-648FG')
            
            # Drive backwards (motor_set -200,-200)
            #ser.write(b'0000450307-96,-9699G')
            
            # Drive backwards (motor_set -300,-300)
            ser.write(b'0000450309-12C,-12C0AG')

            # Drive backwards (motor_set -500,-500)
            #ser.write(b'0000450309-1F4,-1F414G')

            last_key = "j"
        elif get_key_pressed() == "l" and last_key != "l":
            # Turn right (motor_set 300,-300)
            #ser.write(b'000069030812C,-12CE1G')
            
            # Turn right (motor_set 200,-200)
            ser.write(b'0000450306C8,-C883G')

            last_key = "l"
        elif get_key_pressed() == "h" and last_key != "h":
            # Turn left (motor_set -300,300)
            #ser.write(b'0000690308-12C,12CE1G')
            
            # Turn left (motor_set -200,200)
            ser.write(b'0000450306-C8,C883G')
            
            last_key = "h"
        elif get_key_pressed() == "8" and last_key != "8":
            # Drive 2000 mm (2 m) backwards
            ser.write(b'0000690108-7D0,1F4E9G')
            last_key = "8"
        elif get_key_pressed() == "9" and last_key != "9":
            # Drive 2000 mm (2 m) forward
            ser.write(b'00006901077D0,1F4BBG')
            last_key = "9"
        elif get_key_pressed() == "" and last_key != "" and last_key != "8" and last_key != "9":
            # Send END (stop) command 
            ser.write(b'000045000105BG')
            last_key = ""


input("Press enter to continue...")
print()

subprocess.run(["stty", "echo"], check=True)

ser.close()
