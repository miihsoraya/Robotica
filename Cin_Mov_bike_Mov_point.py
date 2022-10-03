#Codigo adaptado  https://www.youtube.com/watch?v=zHboXMY45YU&list=WL&index=1

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

        #Window settings
        pygame.display.set_caption("Mobile robot")
        self.map = pygame.display.set_mode((self.width, self.height))

        #Text to view the desired position
        self.font=pygame.font.Font('freesansbold.ttf', 30)
        self.text_des_pos=self.font.render('default', True, self.white, self.black)
        self.textRect_des_pos=self.text_des_pos.get_rect()
        self.textRect_des_pos.center = (dimentions[1]-600,dimentions[0]-150)
        #Text configuration
        self.text=self.font.render('default', True, self.white, self.black)
        self.textRect=self.text.get_rect()
        self.textRect.center = (dimentions[1]-600,dimentions[0]-100)
        #Text to view the control system
        self.text_control=self.font.render('default', True, self.white, self.black)
        self.textRect_control=self.text_control.get_rect()
        self.textRect_control.center = (dimentions[1]-600,dimentions[0]-50)

        #Trail
        self.trail_set=[]
        
    def write_info(self, x, y, theta, gama, distance, velocity, orientation_angle, x_des, y_des) :
        #Pose information
        Pose=f"x = {int(x)} y = {int(y)} theta = {int(math.degrees(theta))} gama = {(math.degrees(gama)):.2f}"
        self.text=self.font.render(Pose, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        #Desired position
        Des_Pos=f"x = {int(x_des)} y = {int(y_des)}"
        self.text_des_pos=self.font.render(Des_Pos, True, self.blue, self.black)
        self.map.blit(self.text_des_pos, self.textRect_des_pos)
        #Control information
        Control_Inf=f"D = {distance:.2f} V = {velocity:.2f} OE = {(math.degrees(orientation_angle)):.2f}"
        self.text_control=self.font.render(Control_Inf, True, self.white, self.black)
        self.map.blit(self.text_control, self.textRect_control)

    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yel, (self.trail_set[i][0], self.trail_set[i][1]), (self.trail_set[i+1][0], self.trail_set[i+1][1]))
            
        if self.trail_set.__sizeof__()>10000: #Numbe of elements on the trail
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
    def __init__(self, startpos, robotImg, L, v, desired_pos, theta):
      #Initial conditions

        #Meters to pixels
        self.m2p = 3779.52 

        #Velocity
        self.v=v*self.m2p
        
        #Dimensions
        self.L = L
        
        self.x=startpos[0]
        self.y=startpos[1]
        self.theta=theta
        self.gama=0

        #Control parameters
        self.distance=0
        self.orientation_error=0

        #Robot
        self.img=pygame.image.load(robotImg)
        self.rotated=self.img
        self.rect=self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move_to_point(self, desired_pos, Kv, Kh):
        #Desired position
        x_desired=desired_pos[0]
        y_desired=desired_pos[1]

        #Calculate the distance from the actual to the desired point
        self.distance = math.sqrt((x_desired-self.x)**2 + (y_desired-self.y)**2)
        self.v = Kv*self.distance # P controler for the velocity

        #Calculate the desired orientation (angle) - steering
        #desired_orientation = math.atan2((y_desired-self.y), (x_desired-self.x))
        desired_orientation = math.atan2((self.y-y_desired), (x_desired-self.x))
        self.orientation_error=desired_orientation-self.theta # Error on orientation
        self.gama = Kh*self.orientation_error # P controler for the orientation/steering
                            
        #Mathematical model           
        self.x+=self.v*math.cos(self.theta)*dt
        self.y-=self.v*math.sin(self.theta)*dt
        self.theta+=((self.v*math.tan(self.gama))/self.L)*dt

        #Reset theta
        if(self.theta>2*math.pi or self.theta<-2*math.pi):
          self.theta=0

        #Change in orientation
        self.rotated=pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1) # Rotate image 'theta' with a scale operation of 1 - no change in size
        self.rect=self.rotated.get_rect(center=(self.x, self.y))        
        
#Initialisation
pygame.init()

#Dimensions
dims = (600, 1200)

#Status
running = True

#Environment
environment = Envir(dims)

#Robot
start_pos =(400, 200)
desired_pos =(200, 373) #Desired position
img_add="robo.png"
robot_length = 1 # 8 pixels / #robot_width = 0.01*3779.52 # 1cm
robot_speed = 0
theta_inicial=math.pi/2 # -math.pi/2
robot = Robot(start_pos, img_add, robot_length, robot_speed, desired_pos, theta_inicial)

# Gains for the controlers
Kv=0.1 # P controler - velocity
Kh=0.05 # P controler - orientation/steering

#dt
dt=0
lasttime=pygame.time.get_ticks()

#Simulation loop
while running:
    #Verify events
    for event in pygame.event.get():
        if event.type == pygame.QUIT: # Quit the window
            running = False
        robot.move_to_point(desired_pos, Kv, Kh)

    #Time change
    dt = (pygame.time.get_ticks() - lasttime)/1000 # Current minus last time # Time in seconds
    lasttime=pygame.time.get_ticks() #Update last time
    
    #Update
    pygame.display.update()
    environment.map.fill(environment.black)
    robot.move_to_point(desired_pos, Kv, Kh)

    environment.write_info(robot.x, robot.y, robot.theta, robot.gama, robot.distance, robot.v, robot.orientation_error, desired_pos[0], desired_pos[1])

    robot.draw(environment.map)
    environment.robot_frame((robot.x, robot.y), robot.theta)
    
    environment.trail((robot.x, robot.y))

#Close pygame window
pygame.quit()
