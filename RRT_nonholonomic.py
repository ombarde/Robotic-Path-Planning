
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
C1=(39,38,53)
C2=(19,51,62)
C3=(31,3,24)
theta_init=0
theta_goal=np.pi/4
step_size=30
#constraints initialization
max_steering_angle=0.7
max_linear_vel=40
new_node_array_iter = 1 
distance_new_node = 0 
dt=0.1
L=20
L1=int(L/2)
final_point=step_size-1
dist_threshold=1
theta_threshold=np.pi/9



screen.fill(WHITE)
INT_MAX = 100000000000000


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


    #Function Definition : Random Point Generator inside Game
    def random_point(self):
        x_random = ri(SIDE_x+GAME_border , SIDE_x + WINDOW_width - GAME_border - 1)
        y_random = ri(SIDE_y+GAME_border , SIDE_y + WINDOW_height - GAME_border - 1 )
        theta_random= (2*np.pi - 0.0001)*ri(0,1)
        return((x_random, y_random,theta_random))


    #Function Definition : Point inside given Rectangle ?
    def point_inside_rec(self,xr,yr,wr,hr,x,y):
        if x> xr and x < xr + wr:
                    if y > yr and y < yr + hr:
                        return(True)
        return(False)


    #Function Definition : Point to Point Distance
    def p2p_dist(self,p1,p2):
        x1,y1,theta1=p1
        x2,y2,theta2=p2
        return ((x1-x2)**2 + (y1-y2)**2)**0.5
        # return ( ( (x1-x2)**2 + (y1-y2)**2 + ((180/np.pi)**2)*min( [ (theta1 - theta2)**2, (theta1- theta2 - np.pi)**2, (theta1 - theta2 + np.pi)**2 ] ))**0.5 )
        # return ( ( (x1-x2)**2 + (y1-y2)**2 + ((180/np.pi)**2)* (theta1 - theta2)**2  )**0.5 )


    #Function Definition : Text on Environment
    def ClickText(self):
        font = pygame.font.Font(None, 12)
        text = font.render('CLICK HERE', True, WHITE)
        textRect = text.get_rect()
        textRect.center = (75, 495)
        screen.blit(text, textRect)


    #Function Definition : Description Text
    def DesText(self,s,x=350,y=485):
        pygame.draw.rect(screen,WHITE,(125,470,500,30))
        font = pygame.font.SysFont('segoeuisemilight', 15)
        text = font.render('%s'%(s), True, BLACK)
        textRect = text.get_rect()
        #textRect.center = (255, 460)
        textRect.center = (x, y)
        screen.blit(text, textRect)

def ConfSpace():
    #CS for circle
    pygame.draw.circle(screen,BLACK,(100,150),20+L1)
    pygame.draw.circle(screen,BLACK,(360,300),30+L1)

    #CS for rectangle
    pygame.draw.rect(screen,BLACK,(200-L1,150-L1,100+L1,50+L1))


#Function Definition :RRT Algorithm
def RRT(x,y,theta,parent):
    x_m,y_m,theta_m= Start
    if (x,y,theta) not in parent and screen.get_at((x,y)) != (0,0,0,255):
        cur_min=INT_MAX
        for v in parent:
            if B1.p2p_dist(v,(x,y,theta))<cur_min:
                x_m,y_m,theta_m=v
                cur_min =  B1.p2p_dist(v,(x,y,theta))


    
    return(x_m,y_m,theta_m)



running = True
#Environment for Game
pygame.draw.rect(screen,BLACK,(SIDE_x,SIDE_y,WINDOW_width,WINDOW_height),GAME_border)
B1 = Environment(BLACK, 25, 470, 100, 50)
B1.create(screen)

# Random Obstacles
pygame.draw.rect(screen,BLACK,(200,150,100,50))
pygame.draw.circle(screen,BLACK,(100,150),20)
pygame.draw.circle(screen,BLACK,(360,300),30)

OBS=dict()

#Number of forward Steps towards random sampled point
Step = 30
#Start stores a single point [Starting point- RED Point]
Start=[]

#End stores a set of destination point [Destination point- Green Point]
#Multiple points allowed to make the point appear bigger, and fast discovery,
#due to huge number of pixels in this game
End=set()


#parent stores the graph
parent=dict()
steer=dict()

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
                    Start=(x,y,theta_init)
                    pygame.draw.circle(screen, RED, (x, y), 10)
            elif level == 3 :
                if B1.point_inside_game(x,y):
                    End=(x,y,theta_goal)
                    pygame.draw.circle(screen, GREEN, (x, y), 10)
        if level>=4:
            running = False
            break
    pygame.display.update()



running = True
parent[Start]=(-1,-1,0)
steer[((-1,-1,0),Start)]=0

Trace=[]
Timer =  time.time()
print("Start:",Start)
print("End:",End)



def collision_check_and_inside_game(path):

#No collision
    collision=0
    inside_game=1
    for i in range(1,step_size):
        initial=path[i-1]
        final=path[i]
        step=(final-initial)/20
        for j in range(20):
            new=initial+j*step
            if( B1.point_inside_game(int(new[0]),int(new[1]))== False):
                inside_game=0
                break
            else:
                if screen.get_at((int(new[0]),int(new[1]))) == (0,0,0,255):
                    collision =1
                    break
        if(inside_game == 0 or collision == 1):
            break
            return 0

    if(inside_game ==1 and collision ==0):
        return 1
    else:
        return 0

def target_check(path):

#No collision
    reached=0
    for i in range(1,step_size):
        initial=path[i-1]
        final=path[i]
        step=(final-initial)/20
        for j in range(20):
            new=initial+j*step
            distance=((new[0]-End[0])**2 + (new[1]-End[1])**2)**0.5
            print("Distance from goal",distance)
            if(distance<40):
                print("Reached in target check")
                reached =1
                break
            if screen.get_at((int(new[0]),int(new[1]))) == (0,255,0,255):
                reached =1
                break
            

    return reached



index=0

while(running):
    index=index+1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break

    x,y,theta =B1.random_point()
   
    min_dist=INT_MAX
    steering_angle = -1*max_steering_angle
    linear_vel=max_linear_vel
    # while(linear_vel <= max_linear_vel):
    while(steering_angle <= max_steering_angle):
        path=np.zeros((step_size,3))
        x_near,y_near,theta_near=RRT(x,y,theta,parent)
        path[0] = np.asarray([x_near, y_near, theta_near])
        # print("In steering")
        for i in range(1,step_size):
            path[i][0]=path[i-1][0]+linear_vel*np.cos(path[i-1][2])*dt
            path[i][1]=path[i-1][1]+linear_vel*np.sin(path[i-1][2])*dt
            path[i][2]=(path[i-1][2]+(linear_vel/L)*np.tan(steering_angle)*dt)
            

        new_dist = B1.p2p_dist((x,y,theta),(path[final_point][0],path[final_point][1],path[final_point][2]))

        if( collision_check_and_inside_game(path)):
            for i in range(1,step_size):
                # pygame.draw.line(screen, BLUE, (path[i-1][0],path[i-1][1]), (path[i][0],path[i][1]), 2)
                pygame.display.update() 

            if new_dist < min_dist:
                path_target=path
                steer_t_new = steering_angle
                min_dist =  new_dist

        steering_angle = steering_angle + 0.05

     

    xn_int=int(path_target[final_point][0])
    yn_int=int(path_target[final_point][1])
    t_new=path_target[final_point][2]

    if t_new<0:
        t_new=2*np.pi-abs(t_new)
    if t_new>2*np.pi:
        while t_new>2*np.pi:
            t_new=t_new-2*np.pi

    if(collision_check_and_inside_game(path_target)):

        for i in range(1,step_size):
            pygame.draw.line(screen, BLUE, (path_target[i-1][0],path_target[i-1][1]), (path_target[i][0],path_target[i][1]), 2)
            pygame.display.update()
        if screen.get_at((xn_int,yn_int)) != (0,0,0,255) and (xn_int,yn_int,t_new) not in parent:
            parent[(xn_int,yn_int,t_new)]=(path_target[0][0],path_target[0][1],path_target[0][2])
            steer[((path_target[0][0],path_target[0][1],path_target[0][2]),(xn_int,yn_int,t_new))]=steer_t_new
        if index==500:
            running = False
            break
        if(target_check(path_target)):
            Trace=(xn_int,yn_int,t_new)
            print("Target reached")
            running=False
            
pygame.display.update()

running = True
#This loop gets the route back to Start point

points=[]
while(Trace and running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    while(Trace!=Start):

        points.append(Trace)
        x,y,t = parent[Trace]
        xc,yc,tc=tuple(Trace)
        print("Trace and its parent")
        print(Trace,parent[Trace])
        steer_angle=steer[((x,y,t),(xc,yc,tc))]
        path_temp=np.zeros((step_size,3))
        path_temp[0]=list(parent[Trace])

        for i in range(1,step_size):

            path_temp[i][0]=path_temp[i-1][0]+linear_vel*np.cos(path_temp[i-1][2])*dt
            path_temp[i][1]=path_temp[i-1][1]+linear_vel*np.sin(path_temp[i-1][2])*dt
            path_temp[i][2]=(path_temp[i-1][2]+(linear_vel/L)*np.tan(steer_angle)*dt)

            x1=path_temp[i-1][0]
            y1=path_temp[i-1][1]
            x2=path_temp[i][0]
            y2=path_temp[i][1]
            theta_temp=np.arctan((y2-y1)/(x2-x1))



        for i in range(1,step_size):
            pygame.draw.line(screen, GREEN, (path_temp[i-1][0],path_temp[i-1][1]), (path_temp[i][0],path_temp[i][1]), 2)
            pygame.display.update()

        
        for i in range(1,step_size):
            x1=path_temp[i-1][0]
            y1=path_temp[i-1][1]
            x2=path_temp[i][0]
            y2=path_temp[i][1]
            theta_temp=np.arctan((y2-y1)/(x2-x1))
            xl1= x1 + L1*np.cos(theta_temp+np.pi/2)
            yl1= y1 + L1*np.sin(theta_temp+np.pi/2)
            xr1= x1 + L1*np.cos(theta_temp-np.pi/2)
            yr1= y1 + L1*np.sin(theta_temp-np.pi/2)

            xl2= x2 + L1*np.cos(theta_temp+np.pi/2)
            yl2= y2 + L1*np.sin(theta_temp+np.pi/2)
            xr2= x2 + L1*np.cos(theta_temp-np.pi/2)
            yr2= y2 + L1*np.sin(theta_temp-np.pi/2)

            pygame.draw.line(screen, C1, (xl1,yl1), (xl2,yl2), 2)
            pygame.draw.line(screen, C2, (xr1,yr1), (xr2,yr2), 2)
            # time.sleep(2)

        Trace=(x,y,t)


    B1.DesText("Green Colored Path is the Required Path")
    pygame.display.update()


#Quit the Game
pygame.quit()
