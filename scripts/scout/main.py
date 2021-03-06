from Tkinter import *
from ReactiveRobot import *
from RobotPose import *
from math import *

class Main:
  def __init__(self, parent, host = "127.0.0.1", port = 20000) :
    h = Frame(parent)
    h.pack(padx=15, pady=15, side=TOP)
    self.photo  = PhotoImage(file='snap-shot.png')
    self.image = Label(h, image=self.photo)
    self.image.pack(side=TOP)
    self.f = Frame(parent)
    self.f.pack(padx=15, pady=15,side=TOP)
    Button(self.f, text='<<', command=self.left).pack(side=LEFT)
    Button(self.f, text='^', command=self.forward).pack(side=LEFT)
    Button(self.f, text='>>', command=self.right).pack(side=RIGHT)
    g = Frame(parent)
    g.pack(padx=15, pady=15,side=TOP)
    Button(g, text='snap', command=self.snap).pack(side=LEFT)
    Button(g, text='process', command=self.process).pack(side=LEFT)
    Button(g, text='quit', command=self.quit).pack(side=RIGHT)
    self.imageBuffer = None
    self.robot = ReactiveRobot(host, port)
    self.robot.startup()


  def calculatePose(self, pose, dt, dd) :
    newpose = RobotPose()
    newpose.t = pose.t + dt;
    newpose.x = pose.x + dd * cos(pose.t * 3.1415 / 180);
    newpose.y = pose.y + dd * sin(pose.t * 3.1415 / 180);
    return newpose

  def turn(self, angle) :
    pose = self.robot.queryPose()
    newPose = self.calculatePose(pose, angle, 0.0);
    self.robot.setGoal(newPose)
    print "old pose " + str(pose)
    print "new goal " + str(newPose)

  def left(self) :
    pose = self.robot.queryPose()
    newPose = self.calculatePose(pose, 45.0, 0.0);
    self.robot.setGoal(newPose)
    print "old pose " + str(pose)
    print "new goal " + str(newPose)
    print " go left"

  def right(self) :
    pose = self.robot.queryPose()
    newPose = self.calculatePose(pose, -45.0, 0.0);
    self.robot.setGoal(newPose)
    print " go right"

  def forward(self) :
    print " go forward"
    pose = self.robot.queryPose()
    newPose = self.calculatePose(pose, 0.0, 0.25);
    self.robot.setGoal(newPose)

  def snap(self) :
    self.robot.snapPicture()
    self.imageBuffer = self.robot.getImage()
    q = self.robot.getImageHeader() + self.imageBuffer
    f = open('dump.ppm', 'w')
    f.write(q)
    f.close()
    photo2  = PhotoImage(file='dump.ppm')
    self.image.configure(image=photo2)
    self.photo = photo2
    print "snaped"

  def process(self) :
    print "process"
    if self.imageBuffer == None :
      print "no buffer to process"
    else :
      n = 0
      sumx = 0.0
      tr = ''
      for i in range(640) :
        for j in range(480) :
          off2 = i * 480 + j
          r = self.imageBuffer[3*off2]
          g = self.imageBuffer[3*off2+1]
          b = self.imageBuffer[3*off2+2]
          if (r < b'\x3f') and (g < b'\x3f') and (b > b'\x54') :
            tr = tr + b'\x00\x00\xff'
            n = n + 1
            sumx = sumx + j
          else :
            tr = tr + self.imageBuffer[3*off2:3*off2+3]
      q = self.robot.getImageHeader() + tr
      f = open('dump.ppm', 'w')
      f.write(q)
      f.close()
      photo2  = PhotoImage(file='dump.ppm')
      self.image.configure(image=photo2)
      self.photo = photo2
      if n == 0 :
        mid = -1
      else :
        mid = sumx / n
      print "There were " + str(n) + " blue pixels"
      print "blue direction is " + str(mid)
     
 
    # move robot here
    # ...
    
  def quit(self) :
    self.robot.shutdown()
    self.f.quit()


root = Tk()
root.title('Robot control')
main = Main(root)
root.mainloop()
