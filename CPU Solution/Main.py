import OpenGL.GL as gl
import OpenGL.GLUT as glut
import numpy as np

import sys
import time
import math
import random
import threading

import ThreadSafeQueue
import Particle
import vec3
import Ray

#########################################
###### Global, Events, and Locks ########
#########################################

# Bool toggles
povRay = False # World space rays
cheap = True # True : Non-RayTracing output; False : RayTracing output

# OpenGL
height, width = 1000, 1000
minCoord, maxCoord = -50, 50
minz, maxz = -100, 0

output = []

# Constants
moveStep = 0.01
speedOutputConstant = 8
forceConstant = 5
forceOutputConstant = 1
centreOfMassOutputConstant = 4

# Particles
particleCount = 10
maxParticle = particleCount
particles = []
writingParticlesLock = threading.Lock()
particleCopyLock = threading.Lock()

# Collision & Basic 1-4
collRecord = []
collRecordLock = threading.Lock()

flag = "Basic"

letGravity = threading.Event()
letSpeed = threading.Event()
letForce = threading.Event()
letCenter = threading.Event()

moveCenterLock = threading.Lock()
centerOfMass = [0, 0, 0]

#  Movement
moveWorkAdded = threading.Event()
moveWorkersWakeUp = threading.Event()

moveThreadCount = 0
moveThreadCountLock = threading.Lock()

moveCompleted = threading.Event()
fullMoveCompleted = threading.Event()
nextMove = threading.Event()

# Gravity
gravityQueue = ThreadSafeQueue.Queue()

gravityWorkAdded = threading.Event()
gravityWorkersWakeUp = threading.Event()

gravityThreadCount = 0
gravityThreadCountLock = threading.Lock()

gravityCompleted = threading.Event()

# RayTracing
eye = vec3.vec3(0, 0, 300)

rayWorkAdded = threading.Event()
rayWorkersWakeUp = threading.Event()

rayThreadCount = 0
rayThreadCountLock = threading.Lock()

rayCompleted = threading.Event()
outputReady = threading.Event()

# Output
outputParticlesList = []

outputRequestsParticles = threading.Event()

outputLock = threading.Lock()

# Threading
running = True
numberOfCores = 4
workload = 0
remainder = 0

# Misc
initialiseLock = threading.Lock()
initialisationCompleted = threading.Event()

killSystem = threading.Event()
cleanupComplete = threading.Event()

keyboardLock = threading.Lock()

#########################################
############### OpenGL ##################
#########################################

def displayCallback() :
    global output, width, height

    outputReady.wait()

    with outputLock:
        outputReady.clear()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()

        # Draw image
        gl.glRasterPos2i(-1, -1)
        gl.glPixelZoom(width/(maxCoord*2), height/(maxCoord*2))
        gl.glDrawPixels(width, height, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, output)
        glut.glutSwapBuffers()

def reshapeCallback(width, height):
    gl.glClearColor(1, 1, 1, 1)
    gl.glViewport(0, 0, width, height)
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    gl.glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0)
    gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)

def clearBasicEvents():
    letSpeed.clear()
    letForce.clear()
    letCenter.clear()

def keyboardCallback(key, x, y):
    global running, flag
    with keyboardLock:
        moveCompleted.wait()
        if key == b'1':
            clearBasicEvents()
            print("1. Basic")
        elif key == b'2':
            if letSpeed.isSet():
                letSpeed.clear()
            else:
                clearBasicEvents()
                letSpeed.set()   
                flag = "Speed"
                print("2. Speed")
        elif key == b'3':
            if letForce.isSet():
                letForce.clear()
            else:
                clearBasicEvents()
                letForce.set()
                flag = "Force"
                print("3. Force")
        elif key == b'4':
            if letCenter.isSet():
                letCenter.clear()
            else:
                clearBasicEvents()
                letCenter.set()
                flag = "Center"
                print("4. Center of mass")
        elif key == b'q':
            if letGravity.isSet():
                letGravity.clear()
                letChaos()
                print("Gravity: OFF")
            else:
                letGravity.set()
                print("Gravity: ON")
        elif key == b'5':
            killSystem.set()
            print("Attempting shut down")
            cleanupComplete.wait()
            print("Shut down successful")
            sys.exit( )

def arrowKeysCallback(key, x, y):
    global speedOutputConstant, forceOutputConstant, centreOfMassOutputConstant, particleCount, maxParticle
    with keyboardLock:

        # Increase/Decrease constants that influence output with LEFT/RIGHT arrow keys
        if key == glut.GLUT_KEY_LEFT:
            if letSpeed.isSet():
                if speedOutputConstant > 2:
                    speedOutputConstant -= 1
                    print("Speed constant:", speedOutputConstant)
            elif letForce.isSet():
                if forceOutputConstant > 0.06:
                    forceOutputConstant -= 0.05
                    print("Force constant:", forceOutputConstant)
            elif letCenter.isSet():
                if centreOfMassOutputConstant > 2:
                    centreOfMassOutputConstant -= 1
                    print("Center of mass constant:", centreOfMassOutputConstant)
        elif key == glut.GLUT_KEY_RIGHT:
            if letSpeed.isSet():
                speedOutputConstant += 1
                print("Speed constant:", speedOutputConstant)
            elif letForce.isSet():
                forceOutputConstant += 0.05
                print("Force constant:", forceOutputConstant)
            elif letCenter.isSet():
                centreOfMassOutputConstant += 1
                print("Center of mass constant:", centreOfMassOutputConstant)
        # Increase/Decrease particle count with UP/DOWN arrow keys by ~10%
        elif key == glut.GLUT_KEY_UP:
            diff = int(particleCount * 1.1) - particleCount
            with outputLock:
                particleCount += diff if diff > 0 else 1
                workloadCalculator()
                if particleCount > len(particles):
                    initialiseParticles(particleCount - len(particles))
            print("Particle count:", particleCount)            
        elif key == glut.GLUT_KEY_DOWN:
            diff = particleCount - int(particleCount * 0.9)
            with outputLock:
                particleCount -= diff if diff > 0 else 1
                workloadCalculator()
            print("Particle count:", particleCount)

def startOpenGL():
    glut.glutInit()
    glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGBA | glut.GLUT_DEPTH)
    glut.glutInitWindowSize(width, height)
    glut.glutInitWindowPosition(2600, 365)    
    glut.glutCreateWindow('Example window')
    glut.glutDisplayFunc(displayCallback)
    glut.glutIdleFunc(displayCallback)
    glut.glutReshapeFunc(reshapeCallback)
    glut.glutKeyboardFunc(keyboardCallback)  
    glut.glutSpecialFunc(arrowKeysCallback)

    glut.glutMainLoop()

#########################################
############## Utilities ################
#########################################

# Initialise Particles at start-up
def initialiseParticles(pCount):
    if pCount == -1:
        count = particleCount
    else:
        count = pCount
    for i in range(count):

        # if i == 0:
        #     location = vec3.vec3(-50, -50, 0)
        #     velocity = vec3.vec3(5, 5, 0)
        #     particles.append(Particle.Particle(location, velocity))
        #     continue
        # if i == 1:
        #     location = vec3.vec3(25, 25, 0)
        #     velocity = vec3.vec3(2.5, 2.5, 0)
        #     p = Particle.Particle(location, velocity)
        #     p.radius = 4
        #     particles.append(p)
        #     continue

        # Random x, y, z (Docs reccomend to hard code range)
        x = random.randrange(-50, 50)
        y = random.randrange(-50, 50)
        z = random.randrange(-100, 0)
        loc = vec3.vec3(x, y, z) # Create location vector

        # Random Velocity
        xvel = random.randrange(-500, 500)/100
        yvel = random.randrange(-500, 500)/100
        zvel = random.randrange(-500, 500)/100
        vel = vec3.vec3(xvel, yvel, zvel) # Create velocity vector

        # Create Particle object and add to particles
        particle = Particle.Particle(loc, vel)
        particles.append(particle)

# Re-introduce velocity to particles
# When GRAVITY goes from ON -> OFF
def letChaos():
    # For each particle
    for i in range(particleCount):
        p = particles[i]
        # Random velocity
        xvel = random.randrange(-500, 500)/100
        yvel = random.randrange(-500, 500)/100
        zvel = random.randrange(-500, 500)/100
        if xvel == 0:
            xvel = 1
        if yvel == 0:
            yvel = 1
        if zvel == 0:
            zvel = 1
        # Add velocity to particle
        p.vel = vec3.vec3(xvel, yvel, zvel)
        particles[i] = p

# Calculate Force between 2 points
def force2points(vec1, vec2):
    global forceConstant
    # Squareroot of (difference between x's, y's, z'z squared and added)
    distance = math.sqrt(math.pow(vec2.x - vec1.x, 2) + math.pow(vec2.y - vec1.y, 2) + math.pow(vec2.z - vec1.z, 2))

    if distance == 0:
        distance = 0.01

    force = forceConstant/distance
    return force

# Calculateds workload distribution based on system cores
def workloadCalculator():
    global numberOfCores, particleCount, workload, remainder

    if particleCount < numberOfCores:
        return
    elif particleCount % numberOfCores == 0:
        workload = remainder = int(particleCount / numberOfCores)
    else:
        workload = particleCount // (numberOfCores - 1)
        remainder = particleCount - (workload * (numberOfCores - 1))

# Set relevant events to allow for graceful thread terminations
def killThreads():

    moveWorkAdded.set()
    moveCompleted.set()
    fullMoveCompleted.set()

    letGravity.set()

    outputReady.set()

#########################################
########## System Management ############
#########################################

# Manage all other managers.
# Creates Managers 
# -> Managers create threads based on system cores 
# -> Threads in wait state until work queue populated 
# -> Threads complete work then back to wait state
def systemManager():
    global running

    managerList = []

    # Movement Manager
    movementManagerThread = threading.Thread(target=movementManager, args=())
    managerList.append(movementManagerThread)
    movementManagerThread.setName("movementManager")
    movementManagerThread.start()

    # Gravity Manager
    gravityManagerThread = threading.Thread(target=gravityManager, args=())
    managerList.append(gravityManagerThread)
    gravityManagerThread.setName("gravityManager")
    gravityManagerThread.start()

    # Output Manager
    outputManagerThread = threading.Thread(target=outputManager, args=())
    managerList.append(outputManagerThread)
    outputManagerThread.setName("outputManager")
    outputManagerThread.start()

    # Kill System
    killSystem.wait()
    running = False
    killThreads()

    for m in managerList:
        m.join(0.1)
    
    cleanupComplete.set()

def movementManager():
    global running, outputParticlesList, moveStep
    # moves particles every 0.1 seconds
    # delivers particle list if ray requests
    queue = ThreadSafeQueue.Queue()
    threadList = []

    for i in range(numberOfCores):
        t = threading.Thread(target=moveWorker, args=(queue,))
        threadList.append(t)
        t.setName("moveWorker")
        t.start()

    ### Main loop
    while running:

        time.sleep(moveStep)
        #tic = time.perf_counter_ns()
        if not fullMoveCompleted.isSet():
            moveParticle(queue)
        else:
            time.sleep(0.001)
        #toc = time.perf_counter_ns()
        #print("Movement took:", (toc - tic)/1000000, "ms")
    
    for t in threadList:
        t.join()

def outputManager():
    global running, output, particles, cheap

    queue = ThreadSafeQueue.Queue()
    threadList = []

    

    for i in range(numberOfCores):
        t = threading.Thread(target=rayWorker, args=(queue,))
        threadList.append(t)
        t.setName("rayWorker")
        t.start()
    
    while running:
        
        if not outputReady.isSet():
            
            fullMoveCompleted.wait()
            
            outputParticles = []

            with writingParticlesLock:
                for p in particles:
                    loc = p.loc
                    vel = p.vel
                    force = p.force
                    pout = Particle.Particle(loc, vel)
                    pout.force = force
                    outputParticles.append(pout)
            
            if cheap:
                with outputLock:
                    output = np.zeros((width, height, 4), dtype=np.ubyte)
                    cheapOutput(outputParticles)     
            else:
                with outputLock:
                    output = np.zeros((width, height, 4), dtype=np.ubyte)
                    rtOutput(queue, outputParticles)
            
            outputReady.set()
            fullMoveCompleted.clear()
        else:
            time.sleep(0.0001)


    for t in threadList:
        t.join()

def gravityManager():
    threadList = []

    for i in range(numberOfCores):
        t = threading.Thread(target=gravityWorker, args=())
        threadList.append(t)
        t.setName("gravityWorker")
        t.start()

    while running:
        time.sleep(0.5)

    for t in threadList:
        t.join()

#########################################
### Movement, Collision, and grvaity ####
#########################################

def moveParticle(queue):
    global numberOfCores, workload, remainder, flag, collRecord, gravityQueue

    gravityFlag = letGravity.isSet()

    if particleCount > numberOfCores:
        currIndex = 0
        nextIndex = 0
        for i in range(numberOfCores):
            if i == 0:
                queue.push([i, remainder, gravityFlag])
                currIndex += remainder
            else:
                nextIndex = currIndex + workload
                queue.push([currIndex, nextIndex, gravityFlag])
                currIndex = nextIndex
    else:
        for i in range(particleCount):
            queue.push([i, i+1, gravityFlag])
    
    for i in range(numberOfCores):
        queue.push("Finished")
    
    with writingParticlesLock:
        moveWorkAdded.set()
        moveCompleted.wait()
        
    if running:
        moveCompleted.clear()

    if gravityFlag:
        gravityCompleted.wait()
        gravityCompleted.clear()
    
    with keyboardLock:
        if letForce.isSet() or letCenter.isSet():

            if particleCount > numberOfCores:
                currIndex = 0
                nextIndex = 0
                for i in range(numberOfCores):
                    if i == 0:
                        queue.push([i, remainder, flag, gravityFlag])
                        currIndex += remainder
                    else:
                        nextIndex = currIndex + workload
                        queue.push([currIndex, nextIndex, flag, gravityFlag])
                        currIndex = nextIndex
            else:
                for i in range(particleCount):
                    queue.push([i, i+1, flag, gravityFlag])
            
            for i in range(numberOfCores):
                queue.push("Finished")
            
            with writingParticlesLock:
                moveWorkAdded.set()
                moveCompleted.wait()
            
            if running:
                moveCompleted.clear()

            if letCenter.isSet():
                centerOfMass[0] = centerOfMass[0] / particleCount
                centerOfMass[1] = centerOfMass[1] / particleCount
                centerOfMass[2] = centerOfMass[2] / particleCount
                #print(centerOfMass[0], centerOfMass[1], centerOfMass[2])

    collRecord = []
    if particleCount > numberOfCores:
            currIndex = 0
            nextIndex = 0
            for i in range(numberOfCores):
                if i == 0:
                    queue.push([i, remainder, "Coll", gravityFlag])
                    currIndex += remainder
                else:
                    nextIndex = currIndex + workload
                    queue.push([currIndex, nextIndex, "Coll", gravityFlag])
                    currIndex = nextIndex
    else:
        for i in range(particleCount):
            queue.push([i, i+1, "Coll", gravityFlag])
    
    for i in range(numberOfCores):
        queue.push("Finished")
    
    with writingParticlesLock:
        moveWorkAdded.set()
        moveCompleted.wait()
    
    if running:
        moveCompleted.clear()
    
    for coll in collRecord:
        index = coll[0]
        vel = coll[1]
        particles[index].vel = vel
        
    fullMoveCompleted.set()

def moveWorker(queue):
    global moveWorkAdded, moveThreadCount, moveThreadCountLock, numberOfCores
    #moveWorkersWakeUp = threading.Event()
    while running:
        work = queue.pop()

        if(work == "Empty"):
            moveWorkAdded.clear()
            moveWorkAdded.wait()
            continue
        
        if (len(work) == 3):
            moveBasicWork(work)
        elif (len(work) == 4):
            if (work[2] == "Force"):
                moveForceWork(work)
            elif (work[2] == "Center"):
                moveCenterWork(work)
            elif (work[2] == "Coll"):
                checkCollision(work)
        elif (work == "Finished"):
            with moveThreadCountLock:
                moveThreadCount += 1
            
            if moveThreadCount != numberOfCores:
                moveWorkersWakeUp.wait()
            else:
                with moveThreadCountLock:
                    moveThreadCount = 0
                moveWorkersWakeUp.set()
                moveCompleted.set()

def moveBasicWork(work):
    global gravityQueue, numberOfCores
    start = work[0]
    finish = work[1]
    gravityFlag = work[2]
    for i in range(start, finish):
        particles[i].reVel()
        particles[i].move()
        if gravityFlag:
            gravityQueue.push(particles[i])
            if i == len(particles) - 1:
                for j in range(numberOfCores):
                    gravityQueue.push("Finished")
            gravityWorkAdded.set()

def moveForceWork(work):
    start = work[0]
    finish = work[1]
    for i in range(start, finish):
        p1 = particles[i]
        p1.force = 0
        for j in range(particleCount):
            if i != j:
                p2 = particles[j]
                p1.force = p1.force + force2points(p1.loc, p2.loc)

def moveCenterWork(work):
    global centerOfMass

    start = work[0]
    finish = work[1]
    for i in range(start, finish):
        with moveCenterLock:
            centerOfMass[0] = centerOfMass[0] + particles[i].loc.x
            centerOfMass[1] = centerOfMass[1] + particles[i].loc.y
            centerOfMass[2] = centerOfMass[2] + particles[i].loc.z

def checkCollision(work):
    start = work[0]
    finish = work[1]
    for i in range(start, finish):
        for j in range(particleCount):
            if i != j:
                if particles[i].loc.distance(particles[j].loc) < 2:
                    vel = particles[i].collisionReVel(particles[j])
                    freshColl = True

                    # If colliding with multiple particles
                    for coll in collRecord:
                        if coll[0] == i:
                            freshColl = False
                            coll[1] + vel

                    # If fresh collision
                    if freshColl:
                        with collRecordLock:
                            record = [i, vel]
                            collRecord.append(record)

def gravityWorker():
    global gravityWorkAdded, gravityThreadCount, gravityThreadCountLock, numberOfCores, gravityQueue
    while running:
        work = gravityQueue.pop()

        if(work == "Empty"):
            gravityWorkAdded.clear()
            gravityWorkAdded.wait()
            continue
        elif (work == "Finished"):
            with gravityThreadCountLock:
                gravityThreadCount += 1
            
            if gravityThreadCount != numberOfCores:
                gravityWorkersWakeUp.wait()
            else:
                with gravityThreadCountLock:
                    gravityThreadCount = 0
                gravityWorkersWakeUp.set()
                gravityCompleted.set()
        else:
            applyGravity(work)

def applyGravity(work):
    if(work.loc.x + work.vel.x >= -48):
        work.vel.x = work.vel.x - 0.5
    elif(work.loc.x + work.vel.x < -48 and work.vel.x < -0.5):
        work.vel.x = abs(work.vel.x) - 0.5
    else:
        work.vel.x = 0
        work.vel.y = 0
        work.vel.z = 0

#########################################
########### Output Handling #############
#########################################

def cheapOutput(pOutputParticles):
    global output, speedOutputConstant, forceOutputConstant, centreOfMassOutputConstant

    for i in range(particleCount):
        p = pOutputParticles[i]

        brightness = 255 - abs((p.loc.z / 100) * 255)

        if letSpeed.isSet():
            brightness = (math.sqrt((p.vel.x*p.vel.x) + (p.vel.y*p.vel.y) + (p.vel.z*p.vel.z)) / speedOutputConstant) * 255

        if letCenter.isSet() and centerOfMass:
            brightness = 255 - (math.sqrt(math.pow(p.loc.x - centerOfMass[0], 2) + math.pow(p.loc.y - centerOfMass[1], 2) + math.pow(p.loc.z - centerOfMass[2], 2)) * centreOfMassOutputConstant)

        if letForce.isSet():
            brightness = (p.force * 150) * forceOutputConstant
        
        if brightness > 255:
            brightness = 255
        
        if brightness < 0:
            brightness = 0
        
        outx, outy= int(p.loc.x +50), int(p.loc.y+50)
        output[outx][outy][0] = brightness

def rtOutput(queue, pOutputParticles):
    global numberOfCores, eye, povRay

    for i in range(minCoord, maxCoord):
        for j in range(minCoord, maxCoord):
            
            if povRay:
                pixelPos = vec3.vec3(i, j, eye.z - 250)
                rayDir = pixelPos - eye           
                ray = Ray.Ray(eye, rayDir.unitVec())
            else:
                pixelPos = vec3.vec3(i, j, 2)
                rayDir = vec3.vec3(0, 0, -1)
                ray = Ray.Ray(pixelPos, rayDir)

            # for k in range(particleCount):
            #     castRay(ray, particles[k], i, j)
            for k in range(particleCount):
                queueRequest = [ray, pOutputParticles[k], i, j]
                queue.push(queueRequest)
    
    for i in range(numberOfCores):
        queue.push("Finished")

    rayWorkAdded.set()
    rayCompleted.wait()
    
    if running:
        rayCompleted.clear()

def rayWorker(rayQueue):
    global rayThreadCount, numberOfCores
    while running:
        work = rayQueue.pop()
        
        if(work == "Empty"):
            rayWorkAdded.clear()
            rayWorkAdded.wait()
            continue
        elif (work == "Finished"):
            with rayThreadCountLock:
                rayThreadCount += 1
            
            if rayThreadCount != numberOfCores:
                rayWorkersWakeUp.wait()
            else:
                with rayThreadCountLock:
                    rayThreadCount = 0
                rayWorkersWakeUp.set()
                rayCompleted.set()
        else:
            rayWork(work)

def rayWork(work):
    ray = work[0]
    p = work[1]
    x = work[2]
    y = work[3]
    
    castRay(ray, p, x, y)

def castRay(ray, particle, x, y):

    result = particle.hit(ray)

    if result != -1:
        x = result.x
        y = result.y
        z = result.z

        outx, outy= int(x +50), int(y+50)
        #print(particle.loc.x, particle.loc.y, particle.loc.z, "\n")
        brightness = 255 - abs((z / 100) * 255)

        if letSpeed.isSet():
            brightness = (math.sqrt((particle.vel.x*particle.vel.x) + (particle.vel.y*particle.vel.y) + (particle.vel.z*particle.vel.z)) / speedOutputConstant) * 255

        if letCenter.isSet() and centerOfMass:
            brightness = 255 - (math.sqrt(math.pow(particle.loc.x - centerOfMass[0], 2) + math.pow(particle.loc.y - centerOfMass[1], 2) + math.pow(particle.loc.z - centerOfMass[2], 2)) * centreOfMassOutputConstant)

        if letForce.isSet():
            brightness = (particle.force * 150) * forceOutputConstant
        
        if brightness > 255:
            brightness = 255
        
        if brightness < 0:
            brightness = 0

        output[int(outx)][int(outy)][0] = brightness

if __name__ == "__main__":

    # Thread workload distribution based on system cores
    workloadCalculator()

    # Initialise particles
    initialiseParticles(-1)

    # Start system manager
    systemManagerThread = threading.Thread(target=systemManager)
    systemManagerThread.setName("systemManager")
    systemManagerThread.start()
    
    # Run OpenGL setup and loop
    startOpenGL()