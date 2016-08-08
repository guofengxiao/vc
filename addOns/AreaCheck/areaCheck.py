#encoding: utf-8
from vcCommand import *
from vcHelpers.Application import *
#-------------------------------------------------------------------------------
app = getApplication()
thiscmd = getCommand()
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
GROUPSTEP = 100 
#-------------------------------------------------------------------------------
def DoCreateJointsTxtFile(arg):
  selSequence = app.getSelection( VC_SELECTION_RSLROUTINE ) # "RSL Routine"
  sequence = None
  if selSequence.Count > 0:
    sequence = selSequence.Objects[0]
  else:
    return
  jointStr = ExtractJointsStr( sequence )
  fileName = diag.getProperty("textFileName").Value
  #filePath = "D:\\" + fileName + ".txt"
  filePath = u"D:\\20140616- 南京中科川思特（南京）\\03 其他\\Temp\Medical\\库冒出差\\kr180_1\\" + fileName + ".txt"
  filePath = filePath.encode('gbk')
  writeToTxt(jointStr,filePath)
  print " have created!"
  
def DoCheckLimit(arg):
  global diag
  selComponent = app.getSelection( VC_SELECTION_COMPONENT ) # "RSL Routine"
  component = None
  if selComponent.Count > 0:
    component = selComponent.Objects[0]
  else:
    return
  robotController = component.findBehavioursByType( VC_ROBOTCONTROLLER )
  if robotController == []:
    return
  controller = robotController[-1]
  joints = controller.Joints
  jointDic = {}
  for i in joints:
    jointDic[i.Name[-1]] = [float(i.MinValue),float(i.MaxValue)]
  print jointDic

  selSequence = app.getSelection( VC_SELECTION_RSLROUTINE ) # "RSL Routine"
  sequence = None
  if selSequence.Count > 0:
    sequence = selSequence.Objects[0]
  else:
    return
  filterLimlt = []
  joint_names = [x.Name for x in controller.Joints]
  jointValues = []
  prejointValue = []
  diff = []
  pre = None
  for idx,statement in enumerate(sequence.Statements):
    motionTarget = controller.createTarget()
    motionTarget.UseJoints = False
    statement.writeToTarget( motionTarget )
    
    if idx == 0:
      pre = motionTarget.JointValues
    else:
      diff.append( ListSubstruction(motionTarget.JointValues,pre) )
      pre = motionTarget.JointValues

    statement.writeToTarget( motionTarget )
    jointValue = [float(x) for x in motionTarget.JointValues]
    jointValues.append( jointValue )
    for i,value in enumerate(jointValue):
      if value < jointDic[str(i+1)][0] or value > jointDic[str(i+1)][1]:
        filterLimlt.append( [statement.Name,"J"+str(i+1)] )
  print filterLimlt #sequence4 剔除关节超限区域 第五关节超限
  
  # sequence3 第四关节 第六关节超限
  index = []
  for i,lst in enumerate(diff):
    for j,num in enumerate(lst):
      if abs(num) > 350:
        index.append([i,j])
  print index
  if len(index) > 0:
    for i,lst in enumerate(jointValues):
      for j,num in enumerate(lst):
        if (i >= (index[-1][0] + 1)) and j == 3:
          jointValues[i][j] += 360
          #print jointValues[i]
  
def DoChangeInterpolationMode(arg):
  global diag 
  selSequence = app.getSelection( VC_SELECTION_RSLROUTINE ) # "RSL Routine"
  sequence = None
  if selSequence.Count > 0:
    sequence = selSequence.Objects[0]
  else:
    return
  currentRoutine = sequence
  for statement in currentRoutine.Statements:
    InterpolationModeValue = diag.getProperty("ModeIndex").Value
    statement.getProperty("OrientationInterpolationMode").Value = InterpolationModeValue
	
	
def DoChangeConfigurationMode(arg):
  global diag 
  selSequence = app.getSelection( VC_SELECTION_RSLROUTINE ) # "RSL Routine"
  sequence = None
  if selSequence.Count > 0:
    sequence = selSequence.Objects[0]
  else:
    return
  currentRoutine = sequence
  for statement in currentRoutine.Statements:
    ChangeInterpolationModeValue = diag.getProperty("ModeIndex").Value
    statement.getProperty("ConfigurationMode").Value = ChangeInterpolationModeValue
	
def ExtractJointsStr( sequence ):
  global diag
  selComponent = app.getSelection( VC_SELECTION_COMPONENT ) # "RSL Routine"
  component = None
  if selComponent.Count > 0:
    component = selComponent.Objects[0]
  else:
    return
  robotController = component.findBehavioursByType( VC_ROBOTCONTROLLER )
  if robotController == []:
    return
  controller = robotController[-1]
  result = ""
  for statement in sequence.Statements:
    motionTarget = controller.createTarget()
    statement.writeToTarget( motionTarget )
    motionTarget.UseJoints = False
    jointVals = ['%.2f' % x for x in motionTarget.JointValues]
    jointNames = [x.Name for x in controller.Joints]
    result += statement.Name
    result += ":"
    result += '|'.join(  [x+'='+y+' ' for x,y in zip(jointNames,jointVals) ] )
    result += "\n"
  return result

# 字符串 存 Txt file
def writeToTxt(str,filePath):
  try:
    fp = open(filePath,"w+")
    fp.write(str)
    fp.close()
  except IOError:
    print("fail to open file")

# list 相减
def ListSubstruction(lst1,lst2):
  result = []
  if len(lst1) != len(lst2):
    return result
  for i in range(len(lst1)):
    result.append(float(lst1[i]) - float(lst2[i]))
  return result

def firstState():
  global diag
  diag = getDialog("Area Check")
  diag.GenerateTabCaption = "Area Check"
  tab1_1 = diag.addProperty( 'textFileName',VC_STRING,None,VC_PROPERTY_DEFAULT )  
  tab1_2 = diag.addProperty( 'CreateJointsTxtFile',VC_BUTTON,None )
  tab1_3 = diag.addProperty( 'CheckLimit',VC_BUTTON,None )
  tab1_4 = diag.addProperty( 'ModeIndex',VC_INTEGER,None,VC_PROPERTY_DEFAULT )  
  tab1_5 = diag.addProperty( 'ChangeInterpolationMode',VC_BUTTON,None )
  tab1_6 = diag.addProperty( 'ChangeConfigurationMode',VC_BUTTON,None )
  
  tab1_2.OnChanged = DoCreateJointsTxtFile
  tab1_3.OnChanged = DoCheckLimit

  tab1_5.OnChanged = DoChangeInterpolationMode
  tab1_6.OnChanged = DoChangeConfigurationMode
  
  # Init
  textFileNameProperty = diag.getProperty("textFileName")
  textFileNameProperty.Value = "Sequence"
  groupStep = GROUPSTEP
  tab1_1.Group = groupStep
  groupStep += GROUPSTEP  
  tab1_2.Group = groupStep
  groupStep += GROUPSTEP
  tab1_3.Group = groupStep
  groupStep += GROUPSTEP
  tab1_4.Group = groupStep
  groupStep += GROUPSTEP
  tab1_5.Group = groupStep
  groupStep += GROUPSTEP
  tab1_6.Group = groupStep   
  diag.showTabs()
  diag.show()
  
  
addState(firstState)


