fps = 300.0
W = 500
H = 650
xoffset = W/10
yoffset = 50
goalsize = 200
speed = 300
time = 0
gens = 0
genLength = 5
randomSeed(0)



def listAdd(n, l1, l2):
    for i in range(len(l1)):
        l1[i] = l1[i] + n*l2[i]
    return l1
def listScale(n,l1):
    for i in range(len(l1)):
        l1[i] *= n
    return l1

class AI(object):
    def __init__(self, l1, l2, l3):
        self.HL1 = [0 for i in range(l1)]
        self.HL2 = [0 for i in range(l2)]
        self.OL = [0 for i in range(l3)]
        self.w1 = range(inputLength * l1)
        self.w2 = range(l1 * l2)
        self.w3 = range(l2 * l3)
        self.score = 0
        self.justReverted = False
        self.highscore = -100000000
        self.bestw1 = range(inputLength * l1)
        self.bestw2 = range(l1 * l2)
        self.bestw3 = range(l2 * l3)
        
    def setWeight(self, n, w):
        if(n==1):
            self.w1 = w
        elif(n==2):
            self.w2 = w
        elif(n==0 or n==3):
            self.w3 = w
    def setWeights(self, w1, w2, w3):
        self.setWeight(1,w1)
        self.setWeight(2,w2)
        self.setWeight(3,w3)
    def output(self, inputs):
        self.calculate(inputs, self.w1, self.HL1)
        self.HL1[len(self.HL1)-1] = 1
        self.calculate(self.HL1, self.w2, self.HL2)
        self.HL2[len(self.HL2)-1] = 1
        self.calculate(self.HL2, self.w3, self.OL)
        if(frameCount%50 == 0):
            #print(inputs)
            #print(self.w1)
            #print(self.HL1)
            #print(self.w2)
            #print(self.HL2)
            #print(self.w3)
            #print(self.OL)
            pass
    def control(self, car, otherCar, ball):
        self.carStats = [0 for i in range(inputLength)]
        self.carStats[0] = car.getLoc().x
        self.carStats[1] = car.getLoc().y
        self.carStats[2] = car.getVel().x
        self.carStats[3] = car.getVel().y
        self.carStats[4] = ball.getLoc().x
        self.carStats[5] = ball.getLoc().y
        self.carStats[6] = ball.getVel().x
        self.carStats[7] = ball.getVel().y
        self.carStats[8] = otherCar.getLoc().x
        self.carStats[9] = otherCar.getLoc().y
        self.carStats[10] = otherCar.getVel().x
        self.carStats[11] = otherCar.getVel().y
        self.carStats[12] = 1
        self.output(self.carStats)
        self.newVel = PVector(self.OL[0], self.OL[1])
        self.newVel.normalize()
        
        car.setVel(self.newVel.x*speed, self.newVel.y*speed)
        
    def getStats(self):
        return self.carStats
    def calculate(self, input, weight, output):
        for i in range(len(output)):
            for j in range(len(input)):
                output[i] += input[j]*weight[j*len(output)+i]
            output[i] = -1 + 2/(1+exp(-output[i]))
            #output[i] = max(0,output[i])
    def fitnessUpdate(self, n, car, other, ball):
        self.score += 100*car.getGoals()
        self.score -= 10*other.getGoals()
        self.score += (1000/(car.getLoc().dist(ball.getLoc())**2))**2
        #self.score -= 10000/(other.getLoc().dist(ball.getLoc())**2)
        self.score += (((W/2 - ball.getLoc().x)**2 + ((1+n)*(H/2) - ball.getLoc().y)**2)**.5)/100
    def getFitnessScore(self):
        return self.score
    def resetFitness(self):
        self.score = 0
    def getWeights(self, n):
        if(n==1):
            return self.w1
        elif(n==2):
            return self.w2
        elif(n==0 or n==3):
            return self.w3
    def improved(self):
        self.bestw1 = self.w1
        self.bestw2 = self.w2
        self.bestw3 = self.w3
        self.highscore = self.score
    def setReverted(self, bool):
        self.justReverted = bool
    def getReverted(self):
        return self.justReverted
    def getHighScore(self):
        return self.highscore
    def printBestWeights(self):
        print
        print self.bestw1
        print
        print self.bestw2
        print
        print self.bestw3
        print
    def revertToBest(self):
        self.w1 = self.bestw1
        self.w2 = self.bestw2
        self.w3 = self.bestw3
        self.score = self.highscore
        self.justReverted = True


class physicsBall(object):
    def __init__(self,x,y,r,m,s):
        self.loc = PVector(x,y)
        self.vel = PVector(0.0,0.0)
        self.r = r
        self.m = m
        self.acc = s
    def show(self):
        circle(self.loc.x,self.loc.y,self.r)
    def update(self):
        self.loc += self.vel/fps
    def reset(self,n):
        self.vel *= 0.0
        self.loc = PVector(W/2.0, H/2.0 + n*200)
    def collision(self, other):
        distVector = other.getLoc()-self.loc
        if(distVector.mag()<(self.r+other.getR())/2):
            repulsion = distVector.mag()-(self.r+other.getR())
            repulsion *= -other.getM()/self.m
            repulsion *= ((self.r+other.getR())/2)/(distVector.mag()+.0001)
            distVector.normalize()
            distVector.rotate(PI)
            self.vel += repulsion*distVector
    def getLoc(self):
        return self.loc
    def getVel(self):
        return self.vel
    def getR(self):
        return self.r
    def getM(self):
        return self.m
    def setLoc(self, x, y):
        self.loc.x = x
        self.loc.y = y
    def setVel(self,x,y):
        self.vel.x = x
        self.vel.y = y

class Car(physicsBall):
    def __init__(self,x,y,r,m,s,n):
        physicsBall.__init__(self,x,y,r,m,s)
        self.goals = 0
        self.name = n
        self.moveUp = False
        self.moveLeft = False
        self.moveDown = False
        self.moveRight = False
    def show(self):
        physicsBall.show(self)
        self.mouseHeading = PVector(mouseX-self.loc.x,mouseY-self.loc.y).normalize()
        line(self.loc.x, self.loc.y, self.loc.x + 30*self.mouseHeading.x, self.loc.y + 30*self.mouseHeading.y)
        fill(0)
        textAlign(CENTER)
        textSize(10)
        text(self.name, self.loc.x, self.loc.y+30)
        textSize(30)
        text(self.goals, self.loc.x, self.loc.y+10)
        fill(255)
    def changeMove(self,k,bool):
        if(k == 'w'):
            self.moveUp = bool
        if(k == 'a'):
            self.moveLeft = bool
        if(k == 's'):
            self.moveDown = bool
        if(k == 'd'):
            self.moveRight = bool
    def update(self):
        if(self.moveUp):
            self.vel.y-=self.acc
        if(self.moveLeft):
            self.vel.x-=self.acc
        if(self.moveDown):
            self.vel.y+=self.acc
        if(self.moveRight):
            self.vel.x+=self.acc
        physicsBall.update(self)
        self.restrict()
        self.vel.x*=0.9/log(abs(self.vel.x)+3)
        self.vel.y*=0.9/log(abs(self.vel.y)+3)
    def restrict(self):
        if(self.loc.x > W-xoffset-self.r/2):
            self.loc.x = W-xoffset-self.r/2
        if(self.loc.x < xoffset+self.r/2):
            self.loc.x = xoffset+self.r/2
        if(self.loc.y > H-yoffset-self.r/2):
            self.loc.y = H-yoffset-self.r/2
        if(self.loc.y < yoffset+self.r/2):
            self.loc.y = yoffset+self.r/2
    def score(self):
        self.goals+=1
    def hardReset(self,x,y):
        self.loc = PVector(x,y)
        self.vel = PVector(0.0,0.0)
        self.goals = 0
    def getGoals(self):
        return self.goals

class Ball(physicsBall):
    def check(self):
        if(abs(self.loc.x - W/2) < goalsize/2 - self.r/2):
            if(self.loc.y > H-yoffset-self.r/2):
                self.reset(0)
                myCar.reset(1)
                notMyCar.reset(-1)
                notMyCar.score()
            if(self.loc.y < yoffset+self.r/2):
                self.reset(0)
                myCar.reset(1)
                notMyCar.reset(-1)
                myCar.score()
        else:
            if(self.loc.y > H-yoffset-self.r/2):
                self.loc.y = H-yoffset-self.r/2
                self.vel.y *= -1
            if(self.loc.y < yoffset+self.r/2):
                self.loc.y = yoffset+self.r/2
                self.vel.y *= -1
        if(self.loc.x > W-xoffset-self.r/2):
            self.loc.x = W-xoffset-self.r/2
            self.vel.x *= -1
        if(self.loc.x < xoffset+self.r/2):
            self.loc.x = xoffset+self.r/2
            self.vel.x *= -1
    def update(self):
        physicsBall.update(self)
        self.vel *= (1-3/fps)
        self.check()

myCar = Car(W/2.0, H/2.0+200, 40, 5, speed, "player1")
notMyCar = Car(W/2.0, H/2.0 - 200, 40, 5, speed, "player2")
myBall = Ball(W/2, H/2, 20, 1, speed*2)

inputLength = 12+1
length1 = 8+1
length2 = 4+1
outputLength = 2
carAI1 = AI(length1, length2, outputLength)
carAI2 = AI(length1, length2, outputLength)
W11 = [random(2)-1 for i in range(inputLength * length1)]
W21 = [random(2)-1 for i in range(length1 * length2)]
W31 = [random(2)-1 for i in range(length2 * outputLength)]
#W12 = [random(2)-1 for i in range(inputLength * length1)]
#W22 = [random(2)-1 for i in range(length1 * length2)]
#W32 = [random(2)-1 for i in range(length2 * outputLength)]
carAI1.setWeights(W11, W21, W31)
carAI2.setWeights(W11, W21, W31)

winWeight1 = [0.36453427432212515, 0.5464000743993156, -0.6465748256400782, 
              -0.0490002435748646, 0.6815712903706632, -0.28760381213309494,
               0.016761766872020814, -0.901548374237068, 0.6824928947754649, 
               0.6030281631806804, -0.6153438745929349, -0.36244448011203306, 
               -0.4887792013765019, 0.4239999755096238, 0.9884464268065929, 
               1.3684115354225155, 0.735773891158855, -0.7450379623024356, 
               0.27755358822658643, -1.0830238721153918, -0.8150547939614157, 
               -0.2886273839048988, -1.2473947471264204, 0.46988975347365486, 
               -0.8485629407684864, 0.6736196441542397, -0.5954312243680524, 
               -0.4587245944746053, -0.24596245382194834, -1.0377603281298182, 
               0.5982348241618086, -0.51469134020663, -1.1777637326174075, 
               -0.9046740214340102, 0.23774373619874448, -0.44038119017209304, 
               -0.7089484054739033, -1.2367682816053203, 0.7613338848974949, 
               1.7504167705999845, 0.7728132378878424, 0.2487405442561369, 
               0.309428153791155, 0.6768053122627878, 0.21106804350172603, 
               0.8334928590358862, 0.44678446889912515, 0.28429636061830177, 
               0.6485605379082864, -0.785396600743007, 0.5186022665470004, 
               0.09240105160110239, -0.04085684183127713, 0.5876188044797638, 
               0.8818807186834486, 1.4554407437097605, -0.7876484174859053, 
               -0.31350167536622425, -1.3404859425618343, -0.5118853126180855, 
               1.1202687043513937, -0.4191501805831566, 0.7862265381192745, 
               -0.096248462192356, 0.47594455722719653, -0.8494691872818557, 
               -1.4769503815361669, -0.44108930501698385, -0.5317533638615254, 
               -0.42000657488758514, -1.0637237329283098, 0.44677793156794265, 
               -0.6137454946452703, -0.1059469707844129, -0.3149832025720038, 
               0.8265010243820592, -0.8310331054588399, 0.05563918206736966, 
               0.07324453035038236, 0.5861873356775277, 0.9852033863955095, 
               0.24991406951966655, 0.7377380930194819, -0.12582637354019094, 
               0.3695527991844971, -1.2815261205559527, -0.7478743349041177, 
               -0.003563532043130691, -0.11637669768050947, -0.748985061994708, 
               0.6979856393972896, -0.6001193064144683, -1.0014975547483647, 
               0.18996195960048495, 0.6098622367634917, -0.5528304465112948, 
               0.2907777318590669, 1.2244619892861575, 0.33295760160047894, 
               0.7828062623111621, 0.030608257681651714, -0.371709750776849, 
               0.7506899368721004, -0.4479407224472327, 0.4956877095767553, 
               0.7548703455907451, -0.8080070381585274, 0.6518456100777515, 
               -0.24552445117971866, 0.7533605938569673, 0.9163871152156156, 
               -0.6973561909542494, 0.9683292175161727, 0.7794590159130639, 
               1.186743943345887, -0.09156200599207065, 0.34754848371816105]

winWeight2 = [0.16573685115223633, -0.8803872871761292, -1.0940460983794282, 
              1.228009580710121, 0.01252772644914637, 0.9205581866700534, 
              0.4502361607422758, 0.7440894113376967, -0.48366061380747294, 
              0.3956329748972424, 1.0158026413702617, -0.2540683201605447, 
              -0.7875880737883777, 0.4466979412989122, -0.7964075125244207, 
              0.4496309397520887, -0.4635652837724471, 0.005772570275734891, 
              0.2134769574540607, 0.9529312825630555, -0.07704533811767818, 
              0.3590487218109626, 0.096785620028578, -0.5499605736642351, 
              -0.44814788515173043, -0.7544433041936183, 1.1738623941199413, 
              -0.8544364600897502, -1.3042570754853346, -1.444946093412842, 
              -0.5448409867419123, 0.09149302069346131, -0.9771179995192004, 
              0.9778923347599442, -0.5072205589021941, 0.6331999572377223, 
              -0.4880147207032174, 0.472282985160465, 0.7213229530241477, 
              -0.24536362224304642, 0.3674464409322176, -0.6630644632023245, 
              -0.2862181532042513, -0.6875955658450499, 0.7774851137088798]

winWeight3 = [0.6065817386633692, -0.17443486679698755, -0.7554938289694832, 
              -0.5984234225948067, -1.050189659767642, -0.42558487327966366, 
              -0.4789309640651013, 0.5492267341923206, -0.6593152119133614, 
              1.0522278892553845]

carAI2.setWeights(winWeight1, winWeight2, winWeight3)

change1 = [0 for i in range(inputLength*length1)]
change2 = [0 for i in range(length1*length2)]
change3 = [0 for i in range(length2*outputLength)]

var = 1.0
lastScore = -100000000
won = False

def setup():
    size(W, H)
    frameRate(fps)
    

def draw():
    global gens
    global genLength
    global change1
    global change2
    global change3
    global var
    global lastScore
    global won
    
    background(255)
    rect(xoffset,yoffset,W-2*xoffset,H-2*yoffset)
    rect(W/2 - goalsize/2, 10, goalsize, yoffset-10)
    rect(W/2 - goalsize/2, H-yoffset, goalsize, yoffset-10)
    fill(0)
    textAlign(CORNER)
    textSize(20)
    text("Generation: ", 10, 30)
    text(gens, 110,30)
    fill(255)
    
    #carAI1.control(myCar, notMyCar, myBall)
    carAI2.control(notMyCar, myCar, myBall)
    #myCar.setVel(-myCar.getVel().x, -myCar.getVel().y)
    newCarHeading = PVector(myBall.getLoc().x - myCar.getLoc().x, myBall.getLoc().y - myCar.getLoc().y)
    newCarHeading.normalize()
    #myCar.setVel(-500,500)
    myCar.setVel(speed*newCarHeading.x/3, speed*newCarHeading.y/3)
    
    
    myBall.collision(myCar)
    myBall.collision(notMyCar)
    myCar.collision(myBall)
    notMyCar.collision(myBall)
    myCar.collision(notMyCar)
    notMyCar.collision(myCar)
    
    
    myCar.update()
    myCar.show()
    notMyCar.update()
    notMyCar.show()
    myBall.update()
    myBall.show()
    
    if(frameCount%(.1*fps) == 0):
        carAI1.fitnessUpdate(1, myCar, notMyCar, myBall)
        carAI2.fitnessUpdate(-1, notMyCar, myCar, myBall)
    
    if(frameCount%(genLength*fps) == 0):
        print
        print "___________"
        print gens
        gens += 1
        
        if(carAI2.getReverted()):
            carAI2.setReverted(False)
        else:        
            print var
            
            improvement = carAI2.getFitnessScore() - lastScore
            lastScore = carAI2.getFitnessScore()
            improved = improvement > 0
            print improvement
            
            won = (notMyCar.getGoals() - myCar.getGoals() >= 2)
            
            if(improved):
                print "Improved"
                var = 1.0
                carAI2.improved()
            else:
                carAI2.revertToBest()
                print "Reverted"
                var = 1/var
                if(var>=1):
                    var *= 1.1
                if(var<1):
                    var *= .9
                if(var>10):
                    var = 1
                genVar = .01
                change1 = [random(2)-1 for i in range(inputLength*length1)]
                change1 = listScale(genVar*var, change1)
            
                change2 = [random(2)-1 for i in range(length1*length2)]
                change2 = listScale(genVar*var, change2)
            
                change3 = [random(2)-1 for i in range(length2*outputLength)]
                change3 = listScale(genVar*var, change3)
            
            if(not won):
                carAI2.setWeight(1, listAdd(1, carAI2.getWeights(1), change1))
                carAI2.setWeight(2, listAdd(1, carAI2.getWeights(2), change2))
                carAI2.setWeight(3, listAdd(1, carAI2.getWeights(3), change3))
            else:
                carAI2.printBestWeights()
                print "WON!!"
                
                
        myCar.hardReset(W/2.0, H/2.0+200)
        notMyCar.hardReset(W/2.0, H/2.0-200)
        myBall.reset(0)
        #myBall.setLoc(W/2 + random(200)-100,H/2)
        carAI1.resetFitness()
        carAI2.resetFitness()

def keyPressed():
    carAI2.printBestWeights()
    #if(key == "r"):
        #carAI2.revertToBest()
    #print carAI1.getWeights(1)
    #print carAI1.getWeights(2)
    #print carAI1.getWeights(0)
#    global genLength
    #if(key == "p"):
        #print carAI1.getWeights(1)
    #else:
    #myCar.changeMove(key,True)

#def keyReleased():
    #myCar.changeMove(key,False)
