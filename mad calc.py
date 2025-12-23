import turtle
from tkinter import filedialog
import copy
import math
from matplotlib import pyplot as plt
import numpy as np


CL_SAFETY_FACTOR = 1.5


SCALE = 1
modelName = ""
contraintsModified = False

turtle.hideturtle()
turtle.speed(0)
turtle.tracer(False)

def goto(x,y):
    turtle.goto(SCALE*x, SCALE*y)

def shaded_rect(x1, y1, x2, y2, spacing=4, dotSize=2):
    turtle.penup()
    for x in range(int(x1*SCALE), int(x2*SCALE), spacing):
        for y in range(int(y1*SCALE), int(y2*SCALE), spacing):
            turtle.goto(x, y)
            turtle.dot(dotSize)

def draw_rect(x1, y1, x2, y2):
    turtle.penup()
    goto(x1, y1)
    turtle.pendown()
    goto(x2, y1)
    goto(x2, y2)
    goto(x1, y2)
    goto(x1, y1)


def vector_add(v1, v2):
    return [v1[0] + v2[0], v1[1] + v2[1]]
def vector_sub(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1]]

class Airfoil:
    def __init__(self, string):
        string = string.split(",")
        self.chord = float(string[0])
        self.span = float(string[1])
        self.x1 = float(string[3])
        self.y1 = float(string[4])
        self.x2 = self.x1 + self.chord
        self.y2 = self.y1 + self.span
        self.area = self.chord * self.span / 1000000 # m^2
        self.center = [self.x1 + float(string[2]) * self.chord, self.y1 + self.span / 2]
        self.coefData = []
        self.isStabilizer = False
        if string[5] == "stabilizer":
            self.isStabilizer = True

        file = open("airfoils/" + string[6], "r")
        csv = file.read().split("\n")
        file.close()
        for line in csv[1:]:
            if line == "": continue
            line = line.split(",")
            # [AoA, C_L, C_D, C_M]
            self.coefData.append([float(line[0]), float(line[1]) / CL_SAFETY_FACTOR, float(line[2]), float(line[4])])

    def get_coefData(self, AoA): #by linear interpolation
        if AoA == None:
            return [None, None, None]
        for i in range(1, len(self.coefData)):
            if AoA >= self.coefData[i-1][0] and AoA <= self.coefData[i][0]:
                interpol = (AoA - self.coefData[i-1][0]) / (self.coefData[i][0] - self.coefData[i-1][0])
                C_L = self.coefData[i-1][1] + interpol * (self.coefData[i][1] - self.coefData[i-1][1])
                C_D = self.coefData[i-1][2] + interpol * (self.coefData[i][2] - self.coefData[i-1][2])
                C_M = self.coefData[i-1][3] + interpol * (self.coefData[i][3] - self.coefData[i-1][3])
                return [C_L, C_D, C_M]
        return [None, None, None]

    def get_lift(self, AoA, v, ifStabOffset = 0):
        if AoA == None: return None
        
        if self.isStabilizer:
            AoA = ifStabOffset - AoA
        C_L = self.get_coefData(AoA)[0]
        if C_L == None: return None
        L = 0.5 * 1.225 * v**2 * self.area * C_L
        
        if self.isStabilizer:
            return -L
        return L

    def get_drag(self, AoA, v, ifStabOffset = 0):
        if AoA == None: return None
        
        if self.isStabilizer:
            AoA = ifStabOffset - AoA
        C_D = self.get_coefData(AoA)[1]
        if C_D == None: return None
        
        return 0.5 * 1.225 * v**2 * self.area * C_D
        
    def get_x_moment(self, AoA, v, cgx, offsetx, ifStabOffset = 0):
        if AoA == None: return None
        
        if self.isStabilizer:
            AoA = ifStabOffset - AoA
        
        dx = (cgx - self.x1 - offsetx) / 1000
        C_L, C_D, C_M = self.get_coefData(AoA)
        if C_L == None: return None
        L = 0.5 * 1.225 * v**2 * self.area * C_L
        M = dx * L + 0.5 * 1.225 * v**2 * self.area * self.chord/1000 * C_M
        
        if self.isStabilizer:
            return -M
        return M

class Part:
    def __init__(self):
        self.color = "black"
        self.name = ""
        self.rects = [] # recatangles to draw
        self.rods = [] # rods to draw
        self.mass = 0
        self.offset = [0, 0]
        self.mates = dict()
        self.cg = [0, 0]
        self.attachment = []
        self.airfoils = []
        self.maxThrust = 0
        self.thrust = 0
        self.thrustCenter = [0,0]
        self.pitchControl = 0 # if motor, indicate control direction in hover
        self.rollControl = 0

        self.mates["origin"] = [0, 0]

    def apply_constraint(self):
        if self.attachment == []: return
        global contraintsModified
        selfMate = self.mates[self.attachment[0]]
        otherMate = self.attachment[1].mates[self.attachment[2]]
        otherOffset = self.attachment[1].offset
        newOffset = vector_sub(vector_add(otherOffset, otherMate), selfMate)
        if newOffset != self.offset:
            self.offset = newOffset
            contraintsModified = True

    def abs_cg(self):
        return vector_add(self.offset, self.cg)

    def abs_thrustCenter(self):
        return vector_add(self.offset, self.thrustCenter)

    def get_lift(self, AoA, v, ifStabOffset = 0):
        L = 0
        for airfoil in self.airfoils:
            subLift = airfoil.get_lift(AoA, v, ifStabOffset)
            if subLift == None: return None
            L += subLift
        return L

    def get_drag(self, AoA, v, ifStabOffset = 0):
        D = 0
        for airfoil in self.airfoils:
            subDrag = airfoil.get_drag(AoA, v, ifStabOffset)
            if subDrag == None:
                return None
            D += subDrag
        return D

    def get_x_moment(self, AoA, v, cgx, ifStabOffset = 0):
        M = 0
        for airfoil in self.airfoils:
            subMoment = airfoil.get_x_moment(AoA, v, cgx, self.offset[0], ifStabOffset)
            if subMoment == None:
                return None
            M += subMoment
        return M

    def draw(self):
        turtle.pensize(2)
        turtle.color(self.color)
        turtle.penup()
        goto(self.abs_cg()[0], self.abs_cg()[1])
        turtle.dot(7)
        
        for rect in self.rects:
            draw_rect(rect[0] + self.offset[0], rect[1] + self.offset[1], rect[2] + self.offset[0], rect[3] + self.offset[1])

        for rod in self.rods:
            turtle.penup()
            goto(rod[0] + self.offset[0], rod[1] + self.offset[1])
            turtle.pensize(5)
            turtle.pendown()
            goto(rod[2] + self.offset[0], rod[3] + self.offset[1])
            turtle.penup()
            turtle.pensize(2)

        for a in self.airfoils:
            if a.isStabilizer: turtle.color("orange")
            else: turtle.color("lime")
            shaded_rect(a.x1 + self.offset[0], a.y1 + self.offset[1], a.x2 + self.offset[0], a.y2 + self.offset[1])
            goto(a.center[0] + self.offset[0], a.center[1] + self.offset[1])
            turtle.dot(10, "black")
            turtle.dot(7)

        if self.maxThrust != 0:
            turtle.penup()
            goto(self.abs_thrustCenter()[0], self.abs_thrustCenter()[1])
            turtle.dot(10,"red")




def remove_comments(content):
    content = content.replace("\r", "")
    content = content.split("\n")
    for i in range(0, len(content)):
        if "#" in content[i]:
            content[i] = content[i][0:content[i].find("#")]
        content[i] = content[i].strip()
    content = "\n".join(content)
    return content


def load_file(path):
    file = open(path, "r")
    content = file.read()
    file.close()
    content = remove_comments(content)

    if "END VARIABLES\n" in content:
        variables, content = content.split("END VARIABLES\n")
        variables = variables.split("\n")
        for var in variables:
            if var == "": continue
            var = var.split("=")
            content = content.replace(var[0], var[1])

    
    content = content.split("NEW PART\n")
    genericParts = dict()

    for p in content[1:]:
        if p.count("\n") == 0: continue
        p = p.split("\n")
        part = Part()

        if p[0] == "specialtype:rod":
            dx = 0
            dy = 0
            lengthdensity = 0
            for line in p[1:]:
                if line == "": continue
                line = line.split(":")

                if line[0] == "name":
                    part.name = line[1]
                elif line[0] == "color":
                    part.color = line[1]
                elif line[0] == "dx":
                    dx = float(line[1])
                elif line[0] == "dy":
                    dy = float(line[1])
                elif line[0] == "lengthdensity":
                    lengthdensity = float(line[1])
                elif line[0] == "mate":
                    mate = line[1].split(",")
                    mate[1] = float(mate[1])
                    mate[2] = float(mate[2])
                    part.mates[mate[0]] = mate[1:]
                else:
                    print("Unknown part line in file:")
                    print(line)
            part.rods.append([0, 0, dx, dy])
            part.cg = [dx/2, dy/2]
            part.mates["farend"] = [dx, dy]
            part.mates["center"] = [dx/2, dy/2]
            part.mass = lengthdensity * math.sqrt(dx**2 + dy**2)
            
        else:
            for line in p:
                if line == "": continue
                line = line.split(":")
                
                if line[0] == "name":
                    part.name = line[1]
                elif line[0] == "color":
                    part.color = line[1]
                elif line[0] == "drawrect":
                    r = line[1].split(",")
                    for i in range(0, len(r)):
                        r[i] = float(r[i])
                    part.rects.append(r)
                elif line[0] == "mass":
                    part.mass = float(line[1])
                elif line[0] == "cg":
                    cg = line[1].split(",")
                    cg[0] = float(cg[0])
                    cg[1] = float(cg[1])
                    part.cg = cg
                elif line[0] == "mate":
                    mate = line[1].split(",")
                    mate[1] = float(mate[1])
                    mate[2] = float(mate[2])
                    part.mates[mate[0]] = mate[1:]
                elif line[0] == "rod":
                    r = line[1].split(",")
                    for i in range(0, len(r)):
                        r[i] = float(r[i])
                    part.rods.append(r)
                elif line[0] == "airfoil":
                    part.airfoils.append(Airfoil(line[1]))
                elif line[0] == "maxthrust":
                    part.maxThrust = float(line[1])
                elif line[0] == "thrustcenter":
                    ct = line[1].split(",")
                    ct[0] = float(ct[0])
                    ct[1] = float(ct[1])
                    part.thrustCenter = ct
                else:
                    print("Unknown part line in file:")
                    print(line)
            
        genericParts[part.name] = part

    parts = dict()
    for line in content[0].split("\n"):
        if line == "": continue
        line = line.split(":")
        
        if line[0] == "add":
            detail = line[1].split(",")
            parts[detail[1]] = copy.deepcopy(genericParts[detail[0]])
            parts[detail[1]].name = detail[1]
        elif line[0] == "attach":
            detail = line[1].split(",")
            detail[0] = detail[0].split(".")
            detail[1] = detail[1].split(".")
            parts[detail[0][0]].attachment = [detail[0][1], parts[detail[1][0]], detail[1][1]]
        elif line[0] == "scale":
            global SCALE
            SCALE = float(line[1])
        elif line[0] == "modify":
            detail = line[1].split(",")
            if detail[1] == "pitchControl":
                parts[detail[0]].pitchControl = float(detail[2])
            elif detail[1] == "rollControl":
                parts[detail[0]].rollControl = float(detail[2])
            #add more here if needed later

    apply_constraints(parts)
    return parts
                    
def apply_constraints(model):
    global contraintsModified
    contraintsModified = True

    numIterations = 0
    while contraintsModified and numIterations < 10:
        contraintsModified = False
        for part in model.values():
            part.apply_constraint()

    if contraintsModified:
        print("ERROR: Unable to satisfy contraints")
    
def draw(model):
    for part in model.values():
        part.draw()
    turtle.penup()
    cgx, cgy = get_cg(model)
    goto(cgx, cgy)
    turtle.dot(20, "black")
    turtle.dot(16, "yellow")
    turtle.dot(12, "black")
    turtle.dot(8, "yellow")
    turtle.dot(4, "black")
    turtle.update()

def get_mass(model):
    mass = 0
    for part in model.values():
        mass += part.mass
    return mass

def get_cg(model):
    cg = [0, 0]
    mass = 0
    for part in model.values():
        cg[0] += part.mass * part.abs_cg()[0]
        cg[1] += part.mass * part.abs_cg()[1]
        mass += part.mass

    cg[0] = cg[0] / mass
    cg[1] = cg[1] / mass
    return cg

def get_model_lift(model, AoA, v, tailIncidence):
    L = 0
    for part in model.values():
        subLift = part.get_lift(AoA, v, tailIncidence)
        if subLift == None: return 0
        L += subLift
    return L

def get_model_drag_manual_AoA(model, AoA, v, tailIncidence):
    D = 0
    for part in model.values():
        subDrag = part.get_drag(AoA, v, tailIncidence)
        if subDrag == None: return None
        D += subDrag
    return D

def get_model_x_moment_manual_AoA(model, AoA, v, tailIncidence):
    cgx = get_cg(model)[0]
    M = 0
    for part in model.values():
        subMoment = part.get_x_moment(AoA, v, cgx, tailIncidence)
        if subMoment == None: return None
        M += subMoment
    return M

def get_AoA(model, v, tailIncidence):
    reqLift = 9.81 * get_mass(model) / 1000
    minAoA = 0
    maxAoA = 0
    
    while get_model_lift(model, maxAoA, v, tailIncidence) < reqLift:
        maxAoA += 1
        if maxAoA > 90:
            #print("STALLED")
            return None
    
    while get_model_lift(model, minAoA, v, tailIncidence) > reqLift:
        maxAoA -= 1
        if minAoA < -90:
            #print("STALLED")
            return None

    while maxAoA - minAoA > 0.0001:
        testAoA = (minAoA + maxAoA) / 2
        if get_model_lift(model, testAoA, v, tailIncidence) > reqLift:
            maxAoA = testAoA
        else:
            minAoA = testAoA

    return (minAoA + maxAoA) / 2

def get_model_x_moment(model, v, tailIncidence):
    AoA = get_AoA(model, v, tailIncidence)
    return get_model_x_moment_manual_AoA(model, AoA, v, tailIncidence)

def get_model_drag(model, v, tailIncidence):
    AoA = get_AoA(model, v, tailIncidence)
    return get_model_drag_manual_AoA(model, AoA, v, tailIncidence)

def graph_model_x_moment(model, vMax, tailIncidence, doAxes = True):
    if doAxes:
        plt.figure()
        plt.title(modelName)
        plt.axhline(0, color="black")
        plt.xlabel("Velocity - $ms^{-1}$")
        plt.ylabel("Pitching Moment - $Nm$")

    if type(tailIncidence) == list:
        for i in tailIncidence:
            graph_model_x_moment(model, vMax, i, False)
        if doAxes:
            plt.legend(markerscale = 8)
            plt.show(block = False)
        return
    
    precision = 20
    x = []
    y = []
    for v in range(0, int(vMax * precision)):
        v = v / precision
        M = get_model_x_moment(model, v, tailIncidence)
        if M != None:
            x.append(v)
            y.append(M)
    plt.scatter(x, y, marker=".", s=1, label = "tail incidence = " + str(tailIncidence))
    if doAxes:
        plt.legend(markerscale = 8)
        plt.show(block = False)

def graph_model_drag(model, vMax, tailIncidence, doAxes = True):
    if doAxes:
        plt.figure()
        plt.title(modelName)
        plt.xlabel("Velocity - $ms^{-1}$")
        plt.ylabel("Idealized Wing Drag - $N$")

    if type(tailIncidence) == list:
        for i in tailIncidence:
            graph_model_drag(model, vMax, i, False)
        if doAxes:
            plt.legend(markerscale = 8)
            plt.show(block = False)
        return

    precision = 20
    x = []
    y = []
    for v in range(0, int(vMax * precision)):
        v = v / precision
        D = get_model_drag(model, v, tailIncidence)
        if D != None:
            x.append(v)
            y.append(D)

    plt.scatter(x, y, marker=".", s=1, label = "tail incidence = " + str(tailIncidence))
    if doAxes:
        plt.legend(markerscale = 8)
        plt.show(block = False)

def graph_AoA(model, vMax, tailIncidence, doAxes = True):
    if doAxes:
        plt.figure()
        plt.title(modelName)
        plt.xlabel("Velocity - $ms^{-1}$")
        plt.ylabel("Angle of Attack - (°)")

    if type(tailIncidence) == list:
        for i in tailIncidence:
            graph_AoA(model, vMax, i, False)
        if doAxes:
            plt.legend(markerscale = 8)
            plt.show(block = False)
        return

    precision = 20
    x = []
    y = []
    for v in range(0, int(vMax * precision)):
        v = v / precision
        AoA = get_AoA(model, v, tailIncidence)
        if AoA != None:
            x.append(v)
            y.append(AoA)

    plt.scatter(x, y, marker=".", s=1, label = "tail incidence = " + str(tailIncidence))
    if doAxes:
        plt.legend(markerscale = 8)
        plt.show(block = False)


def get_motor_list(model):
    motors = []
    for part in model.values():
        if part.maxThrust > 0:
            motors.append(part)
    return motors

def get_total_thrust(model):
    motors = get_motor_list(model)
    thrust = 0
    for motor in motors:
        if motor.thrust < 0:
            motor.thrust = 0
        if motor.thrust > motor.maxThrust:
            motor.thrust = motor.maxThrust
        thrust += motor.thrust
    return thrust

def get_total_motor_moment(model):
    motors = get_motor_list(model)
    cgx, cgy = get_cg(model)
    M = [0, 0]
    for motor in motors:
        if motor.thrust < 0:
            motor.thrust = 0
        if motor.thrust > motor.maxThrust:
            motor.thrust = motor.maxThrust
        dx, dy = motor.abs_thrustCenter()
        M[0] += motor.thrust * -dx / 1000
        M[1] += motor.thrust * -dy / 1000
    return M

def update_thrusts_from_powerVal(model, powerVal, pitchVal, rollVal):
    motors = get_motor_list(model)
    out = True
    for motor in motors:
        motor.thrust = motor.maxThrust * (powerVal + pitchVal * motor.pitchControl + rollVal * motor.rollControl)
        if motor.thrust < 0:
            motor.thrust = 0
            out = False
        if motor.thrust > motor.maxThrust:
            motor.thrust = motor.maxThrust
            out = False
    return out

def update_thrusts_from_totThrust(model, totThrust, pitchVal, rollVal, thrustIsRaw = False):
    if thrustIsRaw:
        return update_thrusts_from_powerVal(model, totThrust, pitchVal, rollVal)
    
    motors = get_motor_list(model)
    maxPow = 1
    minPow = 0
    out = False
    while maxPow - minPow >= 0.0000001:
        testPow = (minPow + maxPow) / 2
        out = update_thrusts_from_powerVal(model, testPow, pitchVal, rollVal)
        testThrust = get_total_thrust(model)
        if testThrust > totThrust:
            maxPow = testPow
            maxThrust = testThrust
        else:
            minPow = testPow
            minThrust = testThrust
            
    return out

INITIAL_EXPANSION_RESOLUTION = 0.1

def expand_pitch(model, totThrust, dPitchM, rollVals, thrustIsRaw = False):
    pitchVals = [0, 0]
    
    for ri in [0,1]:
        update_thrusts_from_totThrust(model, totThrust, pitchVals[0], rollVals[ri])
        while get_total_motor_moment(model)[0] > dPitchM:
            pitchVals[0] -= INITIAL_EXPANSION_RESOLUTION
            if update_thrusts_from_totThrust(model, totThrust, pitchVals[0], rollVals[ri], thrustIsRaw) == False:
                return [None, None]
            if pitchVals[0] < -1: return [None, None]
            
        update_thrusts_from_totThrust(model, totThrust, pitchVals[1], rollVals[ri])
        while get_total_motor_moment(model)[0] < dPitchM:
            pitchVals[1] += INITIAL_EXPANSION_RESOLUTION
            if update_thrusts_from_totThrust(model, totThrust, pitchVals[1], rollVals[ri], thrustIsRaw) == False:
                return [None, None]
            if pitchVals[1] > 1: return [None, None]
    
    return pitchVals

def expand_roll(model, totThrust, dRollM, pitchVals, thrustIsRaw = False):
    rollVals = [0, 0]
    
    for pi in [0,1]:
        update_thrusts_from_totThrust(model, totThrust, pitchVals[pi], rollVals[0])
        while get_total_motor_moment(model)[1] > dRollM:
            rollVals[0] -= INITIAL_EXPANSION_RESOLUTION
            if update_thrusts_from_totThrust(model, totThrust, pitchVals[pi], rollVals[0], thrustIsRaw) == False:
                return [None, None]
            if rollVals[0] < -1: return [None, None]
            
        update_thrusts_from_totThrust(model, totThrust, pitchVals[pi], rollVals[1])
        while get_total_motor_moment(model)[1] < dRollM:
            rollVals[1] += INITIAL_EXPANSION_RESOLUTION
            if update_thrusts_from_totThrust(model, totThrust, pitchVals[pi], rollVals[1], thrustIsRaw) == False:
                return [None, None]
            if rollVals[1] > 1: return [None, None]
    
    return rollVals
        

def update_thrusts_from_moment(model, totThrust, dPitchM, dRollM, thrustIsRaw = False):
    minPitchVal = 0
    maxPitchVal = 0
    minRollVal = 0
    maxRollVal = 0

    for mult in [0, 0.1, 0.4, 0.8, 1, 1]:
        minPitchVal, maxPitchVal = expand_pitch(model, totThrust, dPitchM * mult, [minRollVal, maxRollVal], thrustIsRaw)
        if minPitchVal == None: return False
        minRollVal, maxRollVal = expand_roll(model, totThrust, dRollM * mult, [minPitchVal, maxPitchVal], thrustIsRaw)
        if minRollVal == None: return False

    limit = 0.0000001

    while maxPitchVal - minPitchVal > limit or maxRollVal - minRollVal > limit:
        # collapse pitch
        delta = (maxPitchVal - minPitchVal) / 10
        testMax = maxPitchVal - delta
        testMin = minPitchVal + delta
        testMaxMom = [None, None]
        testMinMom = [None, None]
        modified = False
        update_thrusts_from_totThrust(model, totThrust, testMin, minRollVal, thrustIsRaw)
        testMinMom[0] = get_total_motor_moment(model)[0]
        update_thrusts_from_totThrust(model, totThrust, testMin, maxRollVal, thrustIsRaw)
        testMinMom[1] = get_total_motor_moment(model)[0]
        update_thrusts_from_totThrust(model, totThrust, testMax, minRollVal, thrustIsRaw)
        testMaxMom[0] = get_total_motor_moment(model)[0]
        update_thrusts_from_totThrust(model, totThrust, testMax, maxRollVal, thrustIsRaw)
        testMaxMom[1] = get_total_motor_moment(model)[0]

        if testMaxMom[0] >= dPitchM and testMaxMom[1] >= dPitchM:
            maxPitchVal = testMax
            modified = True
        if testMinMom[0] <= dPitchM and testMinMom[1] <= dPitchM:
            minPitchVal = testMin
            modified = True

        
        # collapse roll
        delta = (maxRollVal - minRollVal) / 10
        testMax = maxRollVal - delta
        testMin = minRollVal + delta
        testMaxMom = [None, None]
        testMinMom = [None, None]
        update_thrusts_from_totThrust(model, totThrust, minPitchVal, testMin, thrustIsRaw)
        testMinMom[0] = get_total_motor_moment(model)[1]
        update_thrusts_from_totThrust(model, totThrust, maxPitchVal, testMin, thrustIsRaw)
        testMinMom[1] = get_total_motor_moment(model)[1]
        update_thrusts_from_totThrust(model, totThrust, minPitchVal, testMax, thrustIsRaw)
        testMaxMom[0] = get_total_motor_moment(model)[1]
        update_thrusts_from_totThrust(model, totThrust, maxPitchVal, testMax, thrustIsRaw)
        testMaxMom[1] = get_total_motor_moment(model)[1]

        if testMaxMom[0] >= dRollM and testMaxMom[1] >= dRollM:
            maxRollVal = testMax
            modified = True
        if testMinMom[0] <= dRollM and testMinMom[1] <= dRollM:
            minRollVal = testMin
            modified = True
        
        if modified == False:
            print("Failed to Converge")
            return False
    return [(minPitchVal + maxPitchVal) / 2, (minRollVal + maxRollVal) / 2]

def print_motor_thrusts(model):
    print("---------------------")
    print("Motor configuration:")
    for motor in get_motor_list(model):
        print(motor.name, motor.thrust, "N")
    print()
    print("Total thrust:", get_total_thrust(model), "N")
    print("Total moments:", get_total_motor_moment(model), "Nm")
    print("---------------------")


MICROADJUST = 0.0002

def get_min_max_hover_moments_pitch(model, rollVal, oppositeIsMoment):
    rollMom = 0
    if oppositeIsMoment:
        rollMom = rollVal
        rollVal = 0

    precision = 200
    thrusts = [[], []]
    pitches = [[], []]
    rolls = [[], []]

    for thrustVal in range(0, precision):
        thrustVal = thrustVal / precision

        if oppositeIsMoment: # ONLY WORKS ASSUMING PITCH AND ROLL ARE INDEPENDANT
            rollVal = update_thrusts_from_moment(model, thrustVal, 0, rollMom, True)
            if rollVal == False: continue
            rollVal = rollVal[1]
        
        lastWorked = False
        lastThrust = 0
        lastPitch = 0
        for pitchVal in range(-precision, precision):
            pitchVal = pitchVal / precision
            thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)
            
            while get_total_motor_moment(model)[1] > rollMom + MICROADJUST: # micro-adjustments due to roll dependancy
                rollVal -= MICROADJUST
                thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)
            while get_total_motor_moment(model)[1] < rollMom - MICROADJUST:
                rollVal += MICROADJUST
                thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)

            if lastWorked and not thisWorks:
                update_thrusts_from_powerVal(model, lastThrust, lastPitch, rollVal)
                
                while get_total_motor_moment(model)[1] > rollMom + MICROADJUST: # micro-adjustments due to roll dependancy
                    rollVal -= MICROADJUST
                    update_thrusts_from_powerVal(model, lastThrust, lastPitch, rollVal)
                while get_total_motor_moment(model)[1] < rollMom - MICROADJUST:
                    rollVal += MICROADJUST
                    update_thrusts_from_powerVal(model, lastThrust, lastPitch, rollVal)
                
                thrusts[1].append(get_total_thrust(model))
                m = get_total_motor_moment(model)
                pitches[1].append(m[0])
                rolls[1].append(m[1])
            elif thisWorks and not lastWorked:
                thrusts[0].append(get_total_thrust(model))
                m = get_total_motor_moment(model)
                pitches[0].append(m[0])
                rolls[0].append(m[1])
            if thisWorks:
                lastThrust = thrustVal
                lastPitch = pitchVal
            lastWorked = thisWorks

    #print("pitch")
    #print(thrusts, pitches, rolls)
    return [thrusts, pitches, rolls]


def get_min_max_hover_moments_roll(model, pitchVal, oppositeIsMoment):
    pitchMom = 0
    if oppositeIsMoment:
        pitchMom = pitchVal
        pitchVal = 0
    
    precision = 200
    thrusts = [[], []]
    pitches = [[], []]
    rolls = [[], []]

    for thrustVal in range(0, precision):
        thrustVal = thrustVal / precision

        if oppositeIsMoment: # ONLY WORKS ASSUMING PITCH AND ROLL ARE INDEPENDANT
            pitchVal = update_thrusts_from_moment(model, thrustVal, 0, pitchMom, True)
            if pitchVal == False: continue
            pitchVal = pitchVal[0]
        
        lastWorked = False
        lastThrust = 0
        lastRoll = 0
        for rollVal in range(-precision, precision):
            rollVal = rollVal / precision
            thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)
            
            while get_total_motor_moment(model)[0] > pitchMom + 0.0002: # micro-adjustments due to pitch dependancy
                pitchVal -= 0.0002
                thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)
            while get_total_motor_moment(model)[0] < pitchMom:
                pitchVal += 0.0002
                thisWorks = update_thrusts_from_powerVal(model, thrustVal, pitchVal, rollVal)

            if lastWorked and not thisWorks:
                update_thrusts_from_powerVal(model, lastThrust, pitchVal, lastRoll)
                
                while get_total_motor_moment(model)[0] > pitchMom: # micro-adjustments due to pitch dependancy
                    pitchVal -= 0.0002
                    update_thrusts_from_powerVal(model, lastThrust, pitchVal, lastRoll)
                while get_total_motor_moment(model)[0] < pitchMom:
                    pitchVal += 0.0002
                    update_thrusts_from_powerVal(model, lastThrust, pitchVal, lastRoll)
                
                thrusts[1].append(get_total_thrust(model))
                m = get_total_motor_moment(model)
                pitches[1].append(m[0])
                rolls[1].append(m[1])
                #print(pitchVal, m[0])
            elif thisWorks and not lastWorked:
                thrusts[0].append(get_total_thrust(model))
                m = get_total_motor_moment(model)
                pitches[0].append(m[0])
                rolls[0].append(m[1])
            if thisWorks:
                lastThrust = thrustVal
                lastRoll = rollVal
            lastWorked = thisWorks

    #print("roll")
    #print(thrusts, pitches, rolls)
    return [thrusts, pitches, rolls]
            
def hover_moment_subgraph(thrusts, mainAxis, checkAxis, label, doCheck = True):
    thrusts = thrusts[0] + thrusts[1][::-1]
    mainAxis = mainAxis[0] + mainAxis[1][::-1]
    checkAxis = checkAxis[0] + checkAxis[1][::-1]
    
    if len(checkAxis) > 1 and doCheck:
        mean = 0
        for val in checkAxis:
            mean += val
        mean = mean / len(checkAxis)
        for val in checkAxis:
            if abs(val - mean) > 0.1 * abs(mean) and abs(val - mean) > 0.005:
                print(mean, val)
                label += " (INVALID)"
                break
    
    plt.fill(thrusts, mainAxis, alpha=0.5, label = label)

def graph_hover_moments(model, oppositeVal, oppositeIsMoment):
    plt.figure()
    if oppositeIsMoment:
        plt.title(modelName + " - opposite moment = " + str(oppositeVal))
    else:
        plt.title(modelName + " - opposite control = " + str(oppositeVal))
    plt.xlabel("Thrust - $N$")
    plt.ylabel("Hover Moment - $Nm$")
    plt.axvline(get_mass(model) * 9.81 / 1000, color = "black")
    plt.axhline(0, color = "black")
    
    thrusts, pitches, rolls = get_min_max_hover_moments_pitch(model, oppositeVal, oppositeIsMoment)
    hover_moment_subgraph(thrusts, pitches, rolls, "Pitch", oppositeIsMoment)
     
    thrusts, pitches, rolls = get_min_max_hover_moments_roll(model, oppositeVal, oppositeIsMoment)
    hover_moment_subgraph(thrusts, rolls, pitches, "Roll", oppositeIsMoment)
    thrusts = thrusts[0] + thrusts[1][::-1]
    rolls = rolls[0] + rolls[1][::-1]

    
    plt.legend()
    plt.show(block = False)
        

def graph_hover_moments_3D(model):
    precision = 70
    x = []
    y = []
    z = []
    colors = []
    
    for rollVal in range(-precision, precision):
        rollVal = rollVal / precision
        thrusts, pitches, rolls = get_min_max_hover_moments_pitch(model, rollVal, False)
        for i in range(0, len(thrusts[0])):
            x.extend([thrusts[0][i], thrusts[1][i]])
            y.extend([pitches[0][i], pitches[1][i]])
            z.extend([rolls[0][i], rolls[1][i]])
            colors.extend(["blue", "lime"])
    
    for pitchVal in range(-precision, precision):
        pitchVal = pitchVal / precision
        thrusts, pitches, rolls = get_min_max_hover_moments_roll(model, pitchVal, False)
        for i in range(0, len(thrusts[0])):
            x.extend([thrusts[0][i], thrusts[1][i]])
            y.extend([pitches[0][i], pitches[1][i]])
            z.extend([rolls[0][i], rolls[1][i]])
            colors.extend(["red", "orange"])
    

    fig = plt.figure()
    ax = fig.add_subplot(projection = "3d")
    ax.set_xlabel("Thrust - $N$")
    ax.set_ylabel("Pitch Moment - $Nm$")
    ax.set_zlabel("Roll Moment - $Nm$")
    ax.scatter(x, y, z, marker = ".", alpha=1, c=colors)
    plt.show(block = False)

    

def load_new_model():
    path = filedialog.askopenfilename(initialdir = __file__, filetypes=(("mad calc model", "*.madm"),))
    if path == () or path == "":
        return load_new_model()
    global modelName
    modelName = path[path.rfind("/")+1:-5]
    turtle.title("MAD Calc  -  " + modelName)
    turtle.clear()
    turtle.penup()
    turtle.goto(0, 0)
    turtle.write("LOADING...", align="center", font=(None, 40, "bold"))
    turtle.update()
    turtle.clear()
    model = load_file(path)
    draw(model)
    print("total mass:", get_mass(model))
    return model


def follow_preset_options(model, options):
    options = remove_comments(options).split("\n")
    vMax = 30
    tailIncidence = 0
    for line in options:
        if line == "": continue
        line = line.split(":")
        if line[0] == "vmax":
            vMax = float(line[1])
        elif line[0] == "tailincidence":
            tailIncidence = [float(i) for i in line[1].split(",")]
        elif line[0] == "sethover":
            acceleration, pitch, roll = [float(i) for i in line[1].split(",")]
            update_thrusts_from_moment(model, (get_mass(model)/1000) * (9.81+acceleration), pitch, roll)
        elif line[0] == "printmotors":
            print_motor_thrusts(model)
        elif line[0] == "graph":
            if line[1] == "x_moment":
                graph_model_x_moment(model, vMax, tailIncidence)
            elif line[1] == "drag":
                graph_model_drag(model, vMax, tailIncidence)
            elif line[1] == "AoA":
                graph_AoA(model, vMax, tailIncidence)
            elif line[1] == "hover_moments3d":
                graph_hover_moments_3D(model)
            elif line[1].find("hover_moments") == 0:
                options = line[1].split(",")
                isM = False
                if options[2] == "M":
                    isM = True
                graph_hover_moments(model, float(options[1]), isM)
            else:
                print("ERROR, unknown graph:", line[1])
        else:
            print("ERROR, unknown instruction:")
            print(line)

def options_from_file(model, path = None):
    if path == None:
        path = filedialog.askopenfilename(initialdir = __file__, filetypes=(("mad calc presets file", "*.madp"),))
        if path == () or path == "":
            return load_new_model()

    file = open(path, "r")
    options = file.read()
    file.close()
    follow_preset_options(model, options)


model = load_new_model()

running = True
while running:
    choice = turtle.numinput("MAD Calc", """Pick an option:
 1. Load another model
 2. Analyze using preset file
 3. Quit
""", default = 3, minval = 1, maxval = 3)
    if choice == 1:
        model = load_new_model()
    elif choice == 2:
        options_from_file(model)
    elif choice == 3:
        running = False


