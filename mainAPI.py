import utime

class Motor:
    def __init__(self, port):
        self.motor = eval("hub_runtime.hub.port." + port + ".motor")

    def start(self, voltage):
        self.motor.pwm(voltage)
    
    def stop(self):
        self.motor.brake()
    
    def resetEncoder(self):
        self.motor.preset(0)
        utime.sleep_ms(2)

    def getEncoder(self):
        return self.motor.get()[1]

    def runDegrees(self, degrees, speed):
        self.motor.run_for_degrees(degrees, speed)


class MotorPair: #Multiproccessing
    def __init__(self, portL, portR):
        self.rMotor = eval("hub_runtime.hub.port." + portR + ".motor")
        self.lMotor = eval("hub_runtime.hub.port." + portL + ".motor")


    def startTank(self, lSpeed, rSpeed): 
        self.rMotor.pwm(rSpeed)
        self.lMotor.pwm(lSpeed)

    def stop(self):
        self.rMotor.brake()
        self.lMotor.brake()

    def start(self, steering, speed):
        if(steering > 0):
            r = round(speed - speed * (steering * 0.01))
            self.lMotor.pwm(-(speed))
            self.rMotor.pwm(-(r))

        elif(steering < 0):
            l = round(speed + speed * (steering * 0.01))
            self.rMotor.pwm(speed)
            self.lMotor.pwm(l)

        elif(steering == 0):
            self.rMotor.pwm(speed)
            self.lMotor.pwm(-(speed))

        else:
            print("Error in MotorPair.")

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
        self.hub.temperature()
    
    def buttonCheck(self, btn):
        return eval("self.hub.button." + btn + ".is_pressed()")
        
    def getGyroAngle(self):
        return self.hub.motion.yaw_pitch_roll()[0]
