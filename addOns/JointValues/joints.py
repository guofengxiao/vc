from vcCommand import *
from vcHelpers.Application import *
global diags
app = getApplication()
sim = app.Simulation
sel = app.getSelection(VC_SELECTION_COMPONENT)
diags = []
listeners = []


def showJoints(c):
  global joints, diags, listeners
  #print '1'
  sel.OnPostChange = None
  #print '2'
  joints = []
  d = None
  listener = EventListener(c)
  #print '3'
  diags.append( [c, d, joints, listener] )
  #print '4'
  diags[-1][1] = getDialog( Caption = RomoveSharp(c.Name) + ': Joint Values')
  #print '5'
  contrls = c.findBehavioursByType(VC_SERVOCONTROLLER) 
  #print '6'
  run = 0
  for i in contrls:
    for j in i.Joints:
      run += 1
      jProp = listener.Component.getProperty(j.Name)
      diags[-1][2].append(jProp)
      diags[-1][2][-1].OnChanged = diags[-1][3].updateDialogValue
      p = diags[-1][1].addProperty(jProp.Name, VC_REAL, diags[-1][3].updateRobot)
      p.Group = run
      p.Value = jProp.Value
      listener.Joints.append(jProp)
  listener.Diag = diags[-1][1]
  #print '7'
  if run < 1:
    deleteDiag(c)
    print 'Selected device, "'+c.Name+'", has no kinematic joints.'
  else:
    diags[-1][1].show()

def selectedComp(s):
  if s.Objects:
    c = s.Objects[0]
    showJoints(c)

def first_state():
  global diags
  diags = []
  if sel.Objects:
    for c in sel.Objects:
      #c = sel.Objects[0]
      showJoints(c)
  else:
    print 'Show Joint Values:    Select a device!'
    sel.OnPostChange = selectedComp


def deleteDiag(c):
  global diags
  for x in diags:
    if x[0] == c:
      break
  else:
    x = None
  if x:
    x[1].destroyWindow()
    diags.remove( x )

def RomoveSharp(str):
  SplitedStr = str.split('#')
  NoneStr = ''
  return NoneStr.join(SplitedStr)
  

class EventListener:
  # Event listener to get handle to owner component from triggering property
  def __init__(self, icomp):
    self.Component = icomp
    self.Joints = []
    self.Diag = None
  def updateDialogValue(self, arg):
    self.Diag.getProperty(arg.Name).Value = arg.Value
  def updateRobot(self, arg):
    if not sim.IsRunning:
      self.Component.getProperty(arg.Name).Value = arg.Value
      app.render()


app.OnComponentRemoved = deleteDiag
addState( first_state )