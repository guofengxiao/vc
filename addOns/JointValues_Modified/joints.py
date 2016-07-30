from vcCommand import *
from vcHelpers.Application import *
import vcMatrix
global diags, lastDiag
app = getApplication()
sim = app.Simulation
sel = app.getSelection(VC_SELECTION_COMPONENT)
diags = []
listeners = []
# Common
def PrintvcMatrix( Matrix ):
    print("%s"%Matrix)
    print("\tN: \t%f \t%f \t%f \t0"%(Matrix.N.X,Matrix.N.Y,Matrix.N.Z))
    print("\tO: \t%f \t%f \t%f \t0"%(Matrix.O.X,Matrix.O.Y,Matrix.O.Z))
    print("\tA: \t%f \t%f \t%f \t0"%(Matrix.A.X,Matrix.A.Y,Matrix.A.Z))
    print("\tP: \t%f \t%f \t%f \t1"%(Matrix.P.X,Matrix.P.Y,Matrix.P.Z))
    print("\tWPR: \t%f \t%f \t%f "%(Matrix.WPR.X,Matrix.WPR.Y,Matrix.WPR.Z))



#]--------------------------------------------------------------------------------[#
def showJoints(c):
  global joints, diags, listeners
  sel.OnPostChange = None
  joints = []
  d = None
  listener = EventListener(c)
  diags.append( [c, d, joints, listener] )
  diags[-1][1] = getDialog( Caption = c.Name + ': Joint Values')
  contrls = c.findBehavioursByType(VC_SERVOCONTROLLER) 
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
  if run < 1:
    deleteDiag(c)
    print 'Selected device, "'+c.Name+'", has no kinematic joints.'
  else:
    diags[-1][1].show()
#]--------------------------------------------------------------------------------[#
def selectedComp(s):
  if s.Objects:
    c = s.Objects[0]
    showJoints(c)
    createButtons()
#]--------------------------------------------------------------------------------[#
def featBehNode(CompOrNode,featname=None,behname=None,depth=0):
  # returns list [vcNode, vcFeature Or vcBehaviour] or None, if the Feature/Behaviour wasn't found from this component or in it's node tree
  if featname:
    FeatBeh = CompOrNode.getFeature(featname)
  elif behname:
    FeatBeh = CompOrNode.getBehaviour(behname)
  if FeatBeh:
    #printme(text="feature found from rootnode!")
    return [CompOrNode, FeatBeh]
      
  # if the Feature/Behaviour wasn't in the CompOrNode-Node check if it can be found from the CompOrNode's child-tree
  retval = None
  depth += 1 # rootnode is depth 0, first child-branch depth 1, and so on...
  for child in CompOrNode.Children:
    if child.Type == 32768: # check if the child is a Node (ie. not connected component)
      retval = featBehNode(child,featname,behname,depth+1)
      if retval: break
    
  return retval
#]--------------------------------------------------------------------------------[#
def featNode(component,featname):
  return featBehNode(component,featname,None)
#]--------------------------------------------------------------------------------[#
def BehNode(component,behname):
  return featBehNode(component,None,behname)
#]--------------------------------------------------------------------------------[#
def frameWPM(component, framename):
  fn = featNode(component, framename)
  if fn:
    if fn[0].Type == 16384: # this is a component type node
      return component.WorldPositionMatrix*fn[1].PositionMatrix
    elif fn[0].Type == 32768: # this is a child node
      return fn[0].WorldPositionMatrix*fn[1].PositionMatrix
  else:
    return fn
#]--------------------------------------------------------------------------------[#
def printValues(property):
  global diags, lastDiag
  #print property.Name, property.Type, property.Value
  #diag = None
  selObjName = property.Name.split('-')[0] # parse the component name from the beginning of the button name...
  diag = findDiag(selObjName)
  if diag:
    joints = diag[2]
    if joints:
      #take the first found robotcontroller, from the diags[0][0] component
      contrls = diag[0].findBehavioursByType(VC_ROBOTCONTROLLER)[0]
      text = '%s(%r).JointValues(%s..%s) =' % (diag[0].Name, contrls.Name, joints[0].Name, joints[-1].Name)
      temp = ''
      for each in joints:
        temp += str(each.Value) + ','
      print text + '[' + temp[:-1] + ']'
      
      #TODO: check if the robot has gripper, if it has, the toolpoint.WorldPositionMatrix should be printed instead of the flangeFrame.WPM
      #    temp = behNode(diag[0],'Tool') # temp == None / temp == [Node, vcBehaviour(Tool OneToOne-Interface)]
      #    if temp: 
      #      if connected component found: 
      #        #don't know yet how to get the toolpoint..
      #        #with the toolframe name, we could find the correct 'Position' transferFunction from the Robot::Controller:Tools
      
      fframe = frameWPM(diag[0],'FlangeFrame')
      if fframe:
        # if found print the FlangeFrames WorldPositionMatrix
        print '%s(%r).WorldPositionMatrix = %r' % (diag[0].Name, 'FlangeFrame', [fframe.P.X, fframe.P.Y, fframe.P.Z, fframe.WPR.X, fframe.WPR.Y, fframe.WPR.Z])
    else:
      print joints
  else:
    print "dialog not found!"
#]--------------------------------------------------------------------------------[#
# 引起craches 3DCreate
def deldiag(property):
  pass
  return
  global diags
  selObjName = property.Name.split('-')[0] # parse the component name from the beginning of the button name...
  diag = findDiag(selObjName)
  diag[1].hide()
  print 'deleteDiag pressed.. calling deleteDiag for %r:%r'%(selObjName,diag[0])
  deleteDiag(diag[0]) #sel.Objects[0])
#]--------------------------------------------------------------------------------[#
def print44MatrixValues(property):
  global diags
  selObjName = property.Name.split('-')[0]# 获取当前传入参数的组件名
  diag = findDiag(selObjName)
  #print diag
  #print "="*47# 可以打印多行相同的值 解决 清除屏幕 不能答应多行相同的值
  if diag:
    joints = diag[2]
    rootMatrixInW = diag[0].WorldPositionMatrix
    for each in joints:
      print("="*47)
      print( each.Name )
      mat = vcMatrix.new( rootMatrixInW )
      mat.invert()
      eachNode = diag[0].findNode(each.Name)
      PrintvcMatrix(mat*eachNode.WorldPositionMatrix) # 打印每一个关节相对机器人root的坐标
#]--------------------------------------------------------------------------------[#
def createButtons(diag):
  #nonmodal addProperty(name,type,onchanged,constaints)
  buttonA = diag[1].addProperty(diag[0].Name + '- Print JointValues and WPM', VC_BUTTON, printValues, VC_PROPERTY_DEFAULT)
  
  #This is commented out, because with the modifications to the deleteDiag-function it craches 3DCreate most of the time.
  #in it's original form the function didn't do much: x was either set to None or if x[0] was equal to c it just broke out of the loop.. 
  #buttonB = diag[1].addProperty(diag[0].Name + '- deleteDiag', VC_BUTTON, deldiag, VC_PROPERTY_DEFAULT)
  buttonB = diag[1].addProperty(diag[0].Name + '- Print Every Joint Values(4*4)', VC_BUTTON, print44MatrixValues, VC_PROPERTY_DEFAULT)
#]--------------------------------------------------------------------------------[#
def findDiag(selObjName):
  global diags
  diagExist = None
  #print "selected component object: ", selObject, selObject.Type, selObject.Name
  for diag in diags:
    #print "dialog component object: ", diag[0], diag[0].Type, diag[0].Name
    if diag[0].Name == selObjName:
      #print "match found!"
      diagExist = diag
      break
  return diagExist
#]--------------------------------------------------------------------------------[#
def first_state():
  global diags, lastDiag
  #print "diags= ", diags

  if sel.Objects:
    #find out if dialog for selected robot exist
    diagExist = findDiag(sel.Objects[0].Name)
  
    if diagExist:
      print "Dialog for this robot already exist: ", diagExist
      diagExist[1].show()
      lastDiag = diagExist
      #diags[-1][1].show()
    else:
      #dialog for the selected component does not exist
      #check if the component has controller( if it does not.. the selected component isn't propably a robot.. dont open a dialog
      contrls = sel.Objects[0].findBehavioursByType(VC_SERVOCONTROLLER)
      if contrls:
        print "Dialog for this robot does not exist!, create one"
        showJoints(sel.Objects[0])
        createButtons(diags[-1])
        lastDiag = diags[-1] # if dialog was created, current dialog is the last one on the diags-list 
      else:
        print "The selected component(%r) does not have a 'VC_SERVOCONTROLLER', please select a robot before starting the add-on!" % (sel.Objects[0].Name)
  else:
    print 'Show Joint Values: Select a robot before starting the add-on!'
#]--------------------------------------------------------------------------------[#
def deleteDiag(c):
  global diags
  #print "deleteDiag, \ndiags = %r \nc = %r" % (diags,c)
  # diags = [[<vcComponent object at 0x193DA530>, <vcHelpers.Application.DialogObject instance at 0x193D2F80>, [<vcProperty object at 0x193F66B0>, <vcProperty object at 0x193F6720>, <vcProperty object at 0x193F6790>, <vcProperty object at 0x193F6800>, <vcProperty object at 0x193F6870>, <vcProperty object at 0x193F68E0>], <__main__.EventListener instance at 0x193D2C60>]] 
  # c = <vcComponent object at 0x193DA530>
  for x in diags:
    if x[0] == c:
      print "x[0] == c"
      diag = x
      break
    else:
      print "x[0] <> c: -> x[0] = ", x[0],", c = ", c
      x = None
  print "diag= ", diag
  if diag:
    print "diag[1].destroyWindow()"
    diag[1].destroyWindow()
    diags.remove( diag )
    #else:
    #  print "if not x, x = ",x
  #print "diags should be empty list: ", diags
#]--------------------------------------------------------------------------------[#
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
#]--------------------------------------------------------------------------------[#
app.OnComponentRemoved = deleteDiag
addState( first_state )