import utime

class Motor:
    def __init__(self, port):
        self.motor = eval("hub_runtime.hub.port." + port + ".motor")

    def start(self, voltage):
        self.motor.run_at_speed(voltage)
    
    def stop(self):
        self.motor.brake()
    
    def resetEncoder(self):
        self.motor.preset(0)
        utime.sleep_ms(2)

    def getEncoder(self):
        return self.motor.get()[1]

    def runDegrees(self, degrees, speed):
        self.motor.run_for_degrees(degrees, speed)


class MotorPair: 
    def __init__(self, portL, portR):
        self.rMotor = eval("hub_runtime.hub.port." + portR + ".motor")
        self.lMotor = eval("hub_runtime.hub.port." + portL + ".motor")


    def startTank(self, lSpeed, rSpeed): 
        self.rMotor.run_at_speed(rSpeed)
        self.lMotor.run_at_speed(-(lSpeed))

    def stop(self):
        self.rMotor.brake()
        self.lMotor.brake()

    def start(self, steering, speed):
        steering = round(steering)
        if(steering == 0):
            self.rMotor.run_at_speed(speed)
            self.lMotor.run_at_speed(-(speed))

        elif(steering > 0):
            v = round(speed - (steering * 2 / 100)* speed)
            self.lMotor.run_at_speed(-(speed))
            self.rMotor.run_at_speed(v)

        elif(steering < 0):
            v = round(speed + (steering *2 / 100)* speed)
            self.rMotor.run_at_speed(speed)
            self.lMotor.run_at_speed(-(v))

        else:
            raise ValueError("MotorPair.start.steering != int")

class ColorSensor:
    def __init__(self, port):
        self.cS = eval("hub_runtime.hub.port." + port + ".device")
    
    def getValue(self):
        return self.cS.get()[0]

    def getColor(self):
        return self.cS.get()[1]

class Hub:
    def __init__(self):
        self.hub = eval("hub_runtime.hub")

    def batteryCapacity(self):
        return self.hub.battery.capacity_left()

    def batteryTemp(self):
        return self.hub.battery.temperature()

    def beep(self, volume):
        self.hub.sound.volume(volume)
        self.hub.sound.beep()

    def powerOFF(self):
        self.hub.power_off()

    def restart(self):
        self.hub.reset()
        
    def update(self):
        self.hub.config.update()

    def hubStatus(self):
        return self.hub.status()
    
    def temperature(self):
        return self.hub.temperature()
    
    def buttonCheck(self, btn):
        return eval("hub_runtime.hub.button." + btn + ".is_pressed()")
        
    def getGyroAngle(self):
        return self.hub.motion.yaw_pitch_roll()[0]
    
    def getAllGyro(self):
        return self.hub.motion.yaw_pitch_roll()

    def resetGyro(self):
        self.hub.motion.yaw_pitch_roll(0)
        utime.sleep_ms(2)

    def display(self, smth):
        self.hub.display.show(smth)