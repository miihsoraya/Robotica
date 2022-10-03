#Codigo baseado em https://www.youtube.com/watch?v=zHboXMY45YU&list=WL&index=1

import pygame
import math

class Envir: # Environment class
    def __init__(self, dimentions):
        #Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)

        #Dimensions
        self.height = dimentions[0]
        self.width = dimentions[1]
        self.pontox = 80    #local do ponto X#####
        self.pontoy = 150

        #Window settings
        pygame.display.set_caption("Mobile robot")
        self.map = pygame.display.set_mode((self.width, self.height))

        #X do ponto
        font = pygame.font.Font('freesansbold.ttf', 32)
        self.text2 = font.render('default', True, self.white, self.black)
        self.textRect2 = self.text2.get_rect()
        self.textRect2.center = (self.pontox+35, self.pontoy) #local X

        
        #Text configuration
        self.font=pygame.font.Font('freesansbold.ttf', 30)
        self.text=self.font.render('default', True, self.white, self.black)
        self.textRect=self.text.get_rect()
        self.textRect.center = (dimentions[1]-500,dimentions[0]-50)

    
        #Trail
        self.trail_set=[]


        
    def write_info(self, x, y, theta,gama):
        Pose=f"x = {x} y = {y} theta = {int(math.degrees(theta))} gama = {int(math.degrees(gama))}"
        self.text=self.font.render(Pose, True, self.white, self.black)
        self.text2 = self.font.render('X', True, self.green, self.black)
        self.map.blit(self.text, self.textRect)
        self.map.blit(self.text2, self.textRect2)

    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yel, (self.trail_set[i][0], self.trail_set[i][1]), (self.trail_set[i+1][0], self.trail_set[i+1][1]))
            
        if self.trail_set.__sizeof__()>17000: #Numbe of elements on the trail
            self.trail_set.pop(0)
            
        self.trail_set.append(pos)

    def robot_frame(self, pos, rotation):
        n = 80
        centerx, centery = pos
        x_axis = (centerx + n * math.cos(-rotation),
                  centery + n * math.sin(-rotation))
        y_axis = (centerx + n * math.cos(-rotation + math.pi/2),
                  centery + n * math.sin(-rotation + math.pi/2))
        pygame.draw.line(self.map, self.red, (centerx, centery), x_axis, 3) # X axis in red
        pygame.draw.line(self.map, self.green, (centerx, centery), y_axis, 3) # Y axis in green
    
          

class Robot:
    def __init__(self, startpos, robotImg, width):
      #Initial conditions
        self.w=width
        self.x=startpos[0]
        self.y=startpos[1]
        self.theta=0
        self.m2p = 3779.52 # meters to pixels
        self.vl=0.01*self.m2p
        self.vr=0.01*self.m2p
        self.maxspeed=0.02*self.m2p
        self.minspeed=-0.02*self.m2p
        self.gama=0
        #self.ponto=0

        #Robot
        self.img=pygame.image.load(robotImg)
        self.rotated=self.img
        self.rect=self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self,env):
        
#        if event is not None:
#            if event.type == pygame.KEYDOWN:
#                #Left wheel - velocity
#                if event.key == pygame.K_q: #Increase
#                    
#                elif event.key == pygame.K_a: #Decrease
#                    self.gama-=0.00001*self.m2p

                    
##            if event.type == pygame.KEYDOWN:
##                #Left wheel - velocity
##                if event.key == pygame.K_KP4:
##                    self.vl+=0.001*self.m2p
##                elif event.key == pygame.K_KP1:
##                    self.vl-=0.001*self.m2p
##                #Rigth wheel - velocity
##                elif event.key == pygame.K_KP6:
##                    self.vr+=0.001*self.m2p
##                elif event.key == pygame.K_KP3:
##                    self.vr-=0.001*self.m2p
        
         #Mathematical model
        #v=20            
        #self.x+=v*math.cos(self.theta)*dt
        #self.y-=v*math.sin(self.theta)*dt
        #delta_v=self.vr-self.vl
        #self.theta+=(delta_v/self.w)*dt
        #self.theta+=(v/self.w)*math.tan(self.gama)*dt
        ##
        if((env.pontox - self.x) == 0 and (env.pontoy - self.y) == 0): #qnd tiver na posição
            self.x=env.pontox
            self.y=env.pontoy
            exit()
            #para
        else:
            v=0.05*math.sqrt(pow((env.pontox - self.x),2) + pow((env.pontoy - self.y),2))      
            self.x+=v*math.cos(self.theta)*dt
            self.y-=v*math.sin(self.theta)*dt
            self.theta_des = (math.atan2((self.y - env.pontoy),(env.pontox - self.x)))
            self.gama =5*(self.theta_des - self.theta)
            self.theta+=(v/self.w)*math.tan(self.gama)*dt
            print(self.theta)



        ###
        #Reset theta
        if(self.theta>2*math.pi or self.theta<-2*math.pi):
          self.theta=0

        #Keep the current speed or its maximum
        self.vr=min(self.vr, self.maxspeed)
        self.vl=min(self.vl, self.maxspeed)

         #Keep the current speed or its minimum
        self.vr=max(self.vr, self.minspeed)
        self.vl=max(self.vl, self.minspeed)

        
        #Change in orientation
        self.rotated=pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1) # Rotate image 'theta' with a scale operation of 1 - no change in size
        self.rect=self.rotated.get_rect(center=(self.x, self.y))
        
       
#Initialisation
pygame.init()

#Dimensions
dims = (600, 1200) #dimensão da janela

#Status
running = True

#Environment
environment = Envir(dims)

#Robot
start_pos =(80, 260) #posição inicial
img_add="robo.png"
#robot_width = 0.01*3779.52 # 1cm
robot_width = 5 #  pixels  tamanho da bike
robot = Robot(start_pos, img_add, robot_width)

#dt
dt=0
lasttime=pygame.time.get_ticks()

#Simulation loop
while running:
    #Verify events
    for event in pygame.event.get():
        if event.type == pygame.QUIT: # Quit the window
            running = False
        #robot.move(environment)

    #Time change
    dt = (pygame.time.get_ticks() - lasttime)/1000 # Current minus last time # Time in seconds
    lasttime=pygame.time.get_ticks() #Update last time
    
    #Update
    pygame.display.update()
    environment.map.fill(environment.black)
    pygame.draw.rect(environment.map, (0, 0, 255), [200, 100, 770, 400], 2) #inicio(x),inicio(y),final(x),final(y)
    robot.move(environment)

    environment.write_info(int(robot.x), int(robot.y), robot.theta, robot.gama)

    robot.draw(environment.map)
    environment.robot_frame((robot.x, robot.y), robot.theta)
    
    environment.trail((robot.x, robot.y))
