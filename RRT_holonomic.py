import pygame
from random import randint as ri
pygame.init()
import time
import numpy as np

screen = pygame.display.set_mode([600, 600])
SIDE_x = 20
SIDE_y = 40
WINDOW_width = 440
WINDOW_height = 400
GAME_border = 3
WHITE=(255,255,255)
BLUE=(0,0,255)
BLACK=(0,0,0)
RED=(255,0,0)
GREEN=(0,255,0)
RAND=(120,120,120)
YELLOW = (0,0,102)
C1=(39,38,53)
C2=(19,51,62)
C3=(31,3,24)
screen.fill(WHITE)
INT_MAX = 100000000000000
robot_radius = 8

class Environment:
    def __init__ (self, colour, x, y, width, height):
        self.colour = colour
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def create(self,screen):
        pygame.draw.rect(screen, self.colour, [self.x, self.y,self.width ,self.height])

    def point_inside_game(self,x,y):
        if x>SIDE_x+GAME_border and x<SIDE_x + WINDOW_width - GAME_border:
                    if y>SIDE_y+GAME_border and y < SIDE_y + WINDOW_height - GAME_border:
                        return(True)
        return(False)

    def random_point(self):        #Random Point Generator
        x_random = ri(SIDE_x+GAME_border , SIDE_x + WINDOW_width - GAME_border - 1)
        y_random = ri(SIDE_y+GAME_border , SIDE_y + WINDOW_height - GAME_border - 1 )
        return((x_random, y_random))

    def point_inside_rec(self,xr,yr,wr,hr,x,y): # Point inside given Rectangle ?
        if x> xr and x < xr + wr:
                    if y > yr and y < yr + hr:
                        return(True)
        return(False)

    def p2p_dist(self,p1,p2):          # Point to Point Distance
        x1,y1=p1
        x2,y2=p2
        return ( ( (x1-x2)**2 + (y1-y2)**2 )**0.5 )

    def ClickText(self):          # Text on Environment
        font = pygame.font.Font(None, 12)
        text = font.render('CLICK HERE', True, WHITE)
        textRect = text.get_rect()
        textRect.center = (75, 495)
        screen.blit(text, textRect)

    def DesText(self,s,x=350,y=485):    # Description Text
        pygame.draw.rect(screen,WHITE,(125,470,500,30))
        font = pygame.font.SysFont('segoeuisemilight', 15)
        text = font.render('%s'%(s), True, BLACK)
        textRect = text.get_rect()
        #textRect.center = (255, 460)
        textRect.center = (x, y)
        screen.blit(text, textRect)

def ConfSpace():
    #CS for circle
    pygame.draw.circle(screen,BLACK,(100,150),20+robot_radius)
    pygame.draw.circle(screen,BLACK,(360,300),30+robot_radius)

    #CS for rectangle
    pygame.draw.circle(screen,BLACK,(200,150),robot_radius)
    pygame.draw.circle(screen,BLACK,(300,200),robot_radius)
    pygame.draw.circle(screen,BLACK,(200,200),robot_radius)
    pygame.draw.circle(screen,BLACK,(300,150),robot_radius)

    # left side of rectangle 
    pygame.draw.rect(screen,BLACK,(200-robot_radius,150,100,50))
    # right side of rectangle 
    pygame.draw.rect(screen,BLACK,(300,150,robot_radius,50))
    # bottom side of rectangle 
    pygame.draw.rect(screen,BLACK,(200,150-robot_radius,100,robot_radius))
    # top side of rectangle 
    pygame.draw.rect(screen,BLACK,(200,200,100,robot_radius))

def WheelTraj_holonomic(r, points):
    r=10
    theta_list=[]

    print(len(points))
    for i in range(len(points)-1):
        px,py=points[i]
        cx,cy=points[i+1]

        theta=np.arctan2((py-cy),(px-cx))

        theta_list.append(float(theta))

        cx1=cx+r*np.cos(theta)
        cy1=cy+r*np.sin(theta)
        cx2=cx+r*np.cos(theta-2*(np.pi/3))
        cy2=cy+r*np.sin(theta-2*(np.pi/3))
        cx3=cx+r*np.cos(theta+2*(np.pi/3))
        cy3=cy+r*np.sin(theta+2*(np.pi/3))
        px1=px+r*np.cos(theta)
        py1=py+r*np.sin(theta)
        px2=px+r*np.cos(theta-2*(np.pi/3))
        py2=py+r*np.sin(theta-2*(np.pi/3))
        px3=px+r*np.cos(theta+2*(np.pi/3))
        py3=py+r*np.sin(theta+2*(np.pi/3))
        pygame.draw.line(screen, C1, (cx1,cy1), (px1,py1), 3)

        pygame.draw.line(screen, C2, (cx2,cy2), (px2,py2), 3)
        pygame.draw.line(screen, C3, (cx3,cy3), (px3,py3), 3)

        if(len(theta_list)>1):
            theta_old=theta_list[-2]
            theta_new=theta_list[-1]
            
            step=(theta_new-theta_old)/10

            for j in range(10):
                temp=theta_old+step*j
                pygame.draw.circle(screen, C1, (int(cx+r*np.cos(temp)), int(cy+r*np.sin(temp))), 1)
                pygame.draw.circle(screen, C2, (int(cx+r*np.cos(temp-2*(np.pi/3))), int(cy+r*np.sin(temp-2*(np.pi/3)))), 1)
                pygame.draw.circle(screen, C3, (int(cx+r*np.cos(temp+2*(np.pi/3))), int(cy+r*np.sin(temp+2*(np.pi/3)))), 1)
            

def RRT(x,y,parent):
    if (x,y) not in parent and screen.get_at((x,y)) != (0,0,0,255):
        x_m,y_m=-1,-1
        cur_min=INT_MAX
        for v in parent:
            if B1.p2p_dist(v,(x,y))<cur_min:
                x_m,y_m=v
                cur_min =  B1.p2p_dist(v,(x,y))

        good = True
        ans=[]
        theta=np.arctan2((y-y_m),(x-x_m));
        for i in range(Step):
            x_mid=x_m+i*np.cos(theta)
            y_mid=y_m+i*np.sin(theta)
            if screen.get_at((int(x_mid),int(y_mid))) == (0,0,0,255):
                good=False
                break
        if(good):
            ans=[int(x_m+(Step)*np.cos(theta)),int(y_m+Step*np.sin(theta))]
            return(good,x_m,y_m,ans)

    return(False,-1,-1,[])

running = True
#Environment for Game

# Grid with random Obstacles
pygame.draw.rect(screen,BLACK,(SIDE_x,SIDE_y,WINDOW_width,WINDOW_height),GAME_border)
pygame.draw.rect(screen,BLACK,(200,150,100,50))
pygame.draw.circle(screen,BLACK,(100,150),20)
pygame.draw.circle(screen,BLACK,(360,300),30)

B1 = Environment(BLACK, 25, 470, 100, 50)
B1.create(screen)
OBS=dict()


points = []

#Number of forward Steps towards random sampled point
# Step = 10
Step = 30
#Start stores a single point [Starting point- RED Point]
Start=[]

#End stores a set of destination point [Destination point- Green Point]
#Multiple points allowed to make the point appear bigger, and fast discovery,
#due to huge number of pixels in this game
End=set()


#parent stores the graph
parent=dict()

level=0
B1.ClickText()
B1.DesText("Instruction :",y=460)
B1.DesText("Click the BLACK button below to view Configuration Space for Obstacles")
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
        if running==False:
            break
        m = pygame.mouse.get_pressed()
        x,y = pygame.mouse.get_pos()

        if m[0]==1:
            if B1.point_inside_rec(B1.x,B1.y, B1.width, B1.height,x,y):
                    if level==0:
                        level+=1
                        B1.colour=RED
                        ConfSpace()
                        B1.DesText("Click the RED button and select the STARTING POINT")
                    elif level==1 and Start==[]:
                        level+=1
                        B1.colour=GREEN
                        B1.DesText("Click the GREEN button and select the DESTINATION POINT")
                    elif level==2 and Start:
                        level+=1
                        B1.colour=BLUE
                        B1.DesText("Click the BLUE button to view the RRT path and wheel trajectories")
                    elif level==3 and End!=set():
                        level+=1
                        B1.colour=BLUE
                        B1.DesText("Path is being explored using RRT Algorithm with wheel trajectories")

                    B1.create(screen)
                    B1.ClickText()
                    continue
            elif level==1:
                    OBS[(x,y)]=1

            elif level == 2 and Start==[]:
                if B1.point_inside_game(x,y):
                    Start=(x,y)
                    pygame.draw.circle(screen, RED, (x, y), 10)
            elif level == 3 :
                if B1.point_inside_game(x,y):
                    End.add((x,y))
                    pygame.draw.circle(screen, GREEN, (x, y), 10)
        if level>=4:
            running = False
            break
    pygame.display.update()

running = True
parent[Start]=(-1,-1)
Trace=[]

Timer =  time.time()
while(running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    x,y =B1.random_point()
    if (time.time() - Timer) > 5:
        Step=5
    good,x_m,y_m,ans=RRT(x,y,parent)

    if good and ans:
        x_cur = ans[0]
        y_cur = ans[1]
        if screen.get_at((x_cur,y_cur)) != (0,0,0,255) and (x_cur,y_cur) not in parent:
            parent[(x_cur,y_cur)]=(x_m,y_m)
            if screen.get_at((x_cur,y_cur)) == (0, 255, 0, 255):
                Trace=(x_cur,y_cur)
                running = False
            pygame.draw.line(screen, BLUE, (x_cur,y_cur), (x_m,y_m), 2)
    pygame.display.update()

running = True

#This loop gets the route back to Start point
while(Trace and running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    while(Trace!=Start):
        points.append(Trace)         
        x,y = parent[Trace]
        pygame.draw.line(screen, GREEN, (x,y), Trace, 2)
        Trace=(x,y)
    pygame.display.update()
    points.append(Start)
    WheelTraj_holonomic(robot_radius, points)    
    

#Quit
pygame.quit()
