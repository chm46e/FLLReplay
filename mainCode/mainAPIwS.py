#Import section:
import utime

#API/class section:
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

class Timer:
    def __init__(self):
        self.hub = eval("hub_runtime")

    def start(self):
        self.hub.runtime.timer.reset()
    
    def get(self):
        return self.hub.runtime.timer.get()

#Skeleton section:
mHub = Hub()
lMm = Motor("A")
rMm = Motor("B")
lDm = Motor("E")
rDm = Motor("F")
bDm = MotorPair("E", "F")
lCs = ColorSensor("C")
rCs = ColorSensor("D")
timer = Timer()

correction = 0

def acceleration(counter, speed, dASpeed, powerIntegral, forwardBool):
    if(forwardBool == True):
        power = speed
        if(powerIntegral >= power):
            return powerIntegral
        elif(powerIntegral < power and counter % dASpeed == 0):
            powerIntegral+=1
            return powerIntegral
        else:
            return powerIntegral
    elif(forwardBool == False):
        power = speed
        if(powerIntegral <= power):
            return powerIntegral
        elif(powerIntegral > power and counter % dASpeed == 0):
            powerIntegral-=1
            return powerIntegral
        else:
            return powerIntegral
    else:
        raise ValueError("acceleration.forwardBool != bool")

def deceleration(dCounter, dDSpeed, stopSpeed, powerIntegral, forwardBool):
    if(forwardBool == True):
        if(powerIntegral <= stopSpeed):
            return stopSpeed
        elif(powerIntegral > stopSpeed and dCounter % dDSpeed == 0):
            powerIntegral-=1
            return powerIntegral
        else:
            return powerIntegral
    elif(forwardBool == False):
        if(powerIntegral >= stopSpeed):
            return stopSpeed
        elif(powerIntegral < stopSpeed and dCounter % dDSpeed == 0):
            powerIntegral+=1
            return powerIntegral
        else:
            return powerIntegral
    else:
        raise ValueError("deceleration.forwardBool != bool")

def straight(target, dASpeed, dDSpeed, speed, decStart, stopSpeed, accelBool, decBool):
    lDm.resetEncoder()
    rDm.resetEncoder()
    mHub.resetGyro()
    tGa = mHub.getGyroAngle()
    power = speed
    isAccelTime = True
    forward = speed > 0
    powerIntegral = 0
    counter = 0
    dCounter = 0
    isDecTime = False
    q = 1
    m = 1
    a = 1
    while a == 1:
        counter+=1
        cGa = mHub.getGyroAngle()
        if(forward == True):
            steering = (tGa - cGa)*2 + correction
        elif(forward == False):
            steering = (cGa - tGa) * 2 + correction
        else:
            raise Exception("straight.forward != bool")
        if(accelBool == True and isAccelTime == True):
            if(forward == True):
                powerIntegral = acceleration(counter, power, dASpeed, powerIntegral, True)
            elif(forward == False):
                powerIntegral = acceleration(counter, power, dASpeed, powerIntegral, False)
            else:
                raise Exception("straight.forward != bool")
        elif(accelBool == False and isAccelTime == True):
            powerIntegral = power
        elif(isAccelTime == False):
            q+=1
        else:
            raise ValueError("straight.accelBool != bool")
        
        if(decBool == True and isDecTime == True):
            if(m == 1):
                isAccelTime = False
                m = 0
            dCounter+=1
            if(forward == True):
                powerIntegral = deceleration(dCounter, dDSpeed, stopSpeed, powerIntegral, True)
            elif(forward == False):
                powerIntegral = deceleration(dCounter, dDSpeed, stopSpeed, powerIntegral, False)
            else:
                raise Exception("straight.forward != bool")
        elif(decBool == False and isDecTime == True):
            powerIntegral = power
        elif(isDecTime == False):
            q+=1
        else:
            raise ValueError("straight.decBool != bool")

        bDm.start(steering, powerIntegral)
        cRdMv = rDm.getEncoder()
        cLdMv = lDm.getEncoder()
        average = round(abs((cRdMv + -(cLdMv))/2))
        isDecTime = decStart/100 * target <= average

        if(average >= target):
            bDm.stop()
            a = 0

def turn(targetGDeg, speed): 
    mHub.resetGyro()
    fGa = mHub.getGyroAngle()
    b = 1
    if(targetGDeg > 0 or targetGDeg < 0 or targetGDeg == 0):
        if(targetGDeg > 0):
            bDm.start(50, speed)
        elif(targetGDeg < 0):
            bDm.start(-50, speed)
        elif(targetGDeg == 0):
            b = 0
        else:
            raise Exception("turn.targetGDeg == failed")
        while b == 1:
            cGa = mHub.getGyroAngle()
            if(targetGDeg > 0):
                current = cGa - fGa

                if(current >= targetGDeg - 4):
                    bDm.stop()
                    b = 0
            elif(targetGDeg < 0):
                current = -(abs(cGa) - abs(fGa))
        
        
                if(current <= targetGDeg + 3):
                    bDm.stop()
                    b = 0
            else:
                raise Exception("turn.while.targetGDeg == failed")
    else:
        raise ValueError("turn.targetGDeg != |<|>|==| 0")

def lags(tLi, power, tarDeg, port, dASpeed, dDSpeed, decStart, stopSpeed, accelBool, decBool):
    #Designed by: Brickwolves Waring FLL
    #I added acceleration and deceleration
    lDm.resetEncoder()
    rDm.resetEncoder()
    mHub.resetGyro()
    isAccelTime = True
    powerIntegral = 0
    dCounter = 0
    isDecTime = False
    q = 1
    m = 1
    c = 1
    integral = 0
    fGa = mHub.getGyroAngle()
    counter = 0

    if(port[0] == "r" or port[0] == "l"):
        while c == 1:
            counter += 1
            if(port == "rr"):
                cRcSv = rCs.getValue()
                cCsV = (cRcSv - tLi)*0.01
            elif(port == "rl"):
                cRcSv = rCs.getValue()
                cCsV = (tLi - cRcSv)*0.01
            elif(port == "ll"):
                cRcSv = lCs.getValue()
                cCsV = (tLi - cRcSv)*0.01
            elif(port == "lr"):
                cRcSv = lCs.getValue()
                cCsV = (cRcSv - tLi)*0.01
            else:
                raise ValueError("lags.port[1] != 'r' || 'l'")

            integral += cCsV
            cGa = mHub.getGyroAngle()
            cCgA = (fGa - (cCsV + cGa))*2

            if(accelBool == True and isAccelTime == True):
                powerIntegral = acceleration(counter, power, dASpeed, powerIntegral)
            elif(accelBool == False and isAccelTime == True):
                powerIntegral = power
            elif(isAccelTime == False):
                q+=1
            else:
                raise ValueError("straight.accelBool != bool")
            
            if(decBool == True and isDecTime == True):
                if(m == 1):
                    isAccelTime = False
                    m = 0
                dCounter+=1
                powerIntegral = deceleration(dCounter, dDSpeed, stopSpeed, powerIntegral)
            elif(decBool == False and isDecTime == True):
                powerIntegral = power
            elif(isDecTime == False):
                q+=1
            else:
                raise ValueError("straight.decBool != bool")

            bDm.start(cCgA, powerIntegral)
            average = round((-(lDm.getEncoder()) + rDm.getEncoder())/2)
            isDecTime = decStart/100 * tarDeg <= average

            if(average >= tarDeg):
                bDm.stop()
                c = 0
    else:
        raise ValueError("lags.port[0] != 'r' || 'l'")

    global correction
    correction = integral / counter

def mediumMotor(degrees, speed, portIndex):
    if(portIndex == "r"):
        rMm.runDegrees(degrees, speed)
    elif(portIndex == "l"):
        lMm.runDegrees(degrees, speed)
    else:
        raise ValueError("mediumMotor.portIndex != 'r' || 'l'")


def PIDLineFollower(targetDeg, speed):
    rDm.resetEncoder()
    lDm.resetEncoder()
    d = 1
    lastError = 0
    integral = 0
    while d == 1:
        cRcSv = rCs.getValue()
        error = 75 - cRcSv
        proportional = error * 0.03 
        integral+=error
        fIntegral = integral * 0.000005
        derivative = error - lastError
        fDerivative = derivative * 0.0005
        lastError = error
        steering = proportional + fIntegral + fDerivative
        bDm.start(steering, speed)

        cRdMeV = rDm.getEncoder()
        cLdMeV = lDm.getEncoder()

        average = round((cRdMeV + -(cLdMeV)) / 2)

        if(average >= targetDeg or mHub.buttonCheck("left") == True):
            bDm.stop()
            d = 0

def lineSquaring(blackInt, whiteInt, startSpeed):
    g = 1
    counter = 0
    while g == 1:
        counter+=1
        e = 1
        f = 1
        if(counter == 1):
            bDm.startTank(startSpeed, startSpeed)
        elif(counter == 2):
            bDm.startTank(startSpeed, startSpeed)
        elif(counter == 3):
            bDm.startTank(-(startSpeed), -(startSpeed))
        elif(counter == 4):
            bDm.startTank(startSpeed, startSpeed)
        elif(counter == 5):
            bDm.startTank(-(startSpeed), -(startSpeed))
        elif(counter == 6):
            g = 0
            continue
        else:
            raise Exception("lineSquaring.counter != int|> 6")
        while e == 1:
            cRcSv = rCs.getValue()
            cLcSv = lCs.getValue()
            if(counter == 1 or counter == 3 or counter == 5):
                if(cRcSv <= blackInt):
                    rDm.stop()
                    while f == 1:
                        cLcSv = lCs.getValue()
                        if(cLcSv <= blackInt):
                            lDm.stop()
                            f = 0
                            e = 0
                
                elif(cLcSv <= blackInt):
                    lDm.stop()
                    while f == 1:
                        cRcSv = rCs.getValue()
                        if(cRcSv <= blackInt):
                            rDm.stop()
                            f = 0
                            e = 0
                        
            elif(counter == 2 or counter == 4):
                if(cRcSv >= whiteInt):
                    rDm.stop()
                    while f == 1:
                        cLcSv = lCs.getValue()
                        if(cLcSv >= whiteInt):
                            lDm.stop()
                            f = 0
                            e = 0
                
                elif(cLcSv >= whiteInt):
                    lDm.stop()
                    while f == 1:
                        cRcSv = rCs.getValue()
                        if(cRcSv >= whiteInt):
                            rDm.stop()
                            f = 0
                            e = 0
            else:
                raise Exception("lineSquaring.counter != 1,2,3,4,5")

#Program section:
def first():
    global screen
    screen = 0
def second():
    global screen
    screen = 1
def third():
    global screen
    screen = 2
def fourth():
    global screen
    screen = 3
def fifth():
    global screen
    screen = 4
def sixth():
    global screen
    screen = 5
def seventh():
    global screen
    screen = 6
def eighth():
    global screen
    screen = 7
def ninth():
    global screen
    screen = 8
def tenth():
    global screen
    screen = 9

#Loop section:
loop = True
screen = 0
while loop:
    h = 1
    isLaunchTime = False
    if(mHub.buttonCheck("left") == True):
        timer.start()
        while h == 1:
            if(mHub.buttonCheck("left") == False):
                timerNow = timer.get()
                h = 0
        if(timerNow < 0.5):
            screen = abs(screen - 1)
            mHub.display(str(screen))
            utime.sleep_ms(250)
        else:
            isLaunchTime = True
    elif(mHub.buttonCheck("right") == True):
        timer.start()
        while h == 1:
            if(mHub.buttonCheck("right") == False):
                timerNow = timer.get()
                h = 0
        if(timerNow < 0.5):
            if(screen < 9):
                screen+=1
                mHub.display(str(screen))
                utime.sleep_ms(250)
            elif(screen >= 9):
                screen = 0
                mHub.display(str(screen))
                utime.sleep_ms(250)
            else:
                raise Exception("loop.screen != int")
        else:
            isLaunchTime = True
    if(isLaunchTime == True):
        if(screen == 0):
            first()
            mHub.display(str(screen))
        elif(screen == 1):
            second()
            mHub.display(str(screen))
        elif(screen == 2):
            third()
            mHub.display(str(screen))
        elif(screen == 3):
            fourth()
            mHub.display(str(screen))
        elif(screen == 4):
            fifth()
            mHub.display(str(screen))
        elif(screen == 5):
            sixth()
            mHub.display(str(screen))
        elif(screen == 6):
            seventh()
            mHub.display(str(screen))
        elif(screen == 7):
            eighth()
            mHub.display(str(screen))
        elif(screen == 8):
            ninth()
            mHub.display(str(screen))
        elif(screen == 9):
            tenth()
            mHub.display(str(screen))
        else:
            raise ValueError("loop.screen == failed")