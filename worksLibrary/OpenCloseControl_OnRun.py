# WorksRobotController V3.6.1.1
from vcScript import *

app = getApplication()
comp = getComponent()
sim = getSimulation()

ClawNode = None
global open
global openFlag,closeFlag
openFlag = None
closeFlag = None

# 忽视大小写
def astrcmp(str1,str2):
    return str1.lower()==str2.lower()

def OnSignal( signal ):
  global openFlag,closeFlag
  if signal.Name == "Open":
    openFlag = signal.Value
  if signal.Name == "Close":
    closeFlag = signal.Value
  

# Statements 操作
def VerifyMotionStatement( statement ):
    if ((statement.Type == VC_STATEMENT_LINMOTION) or (statement.Type == VC_STATEMENT_PTPMOTION)):
        return True
    else:
        return False


def OnStart():
  global ClawNode,ClawNodeSC
  robot = comp.findBehaviour("RobotPnP").ConnectedComponent #robot in RobotPnP interface
  if not robot: return
  toolInterfaceComp = robot.findBehaviour("Tool").ConnectedComponent
  if not toolInterfaceComp: return

  ClawNode = toolInterfaceComp
  
  list = ClawNode.findBehavioursByType( VC_SERVOCONTROLLER )
  ClawNodeSC = None
  if len(list):
    ClawNodeSC = list[0]
  
def getExistSeq( executor, sequenceNames):
  result = []
  for sequenceName in sequenceNames:
      for i in executor.SubRoutines:
        if astrcmp(i.Name,sequenceName):
          result.append( i )
  return result    



def SetCycleTime(seq,time):
  for statement in seq.Statements:
    if VerifyMotionStatement( statement ):
      statement.CycleTime = time
  pass      
  

  
  
def OnRun():
  global ClawNode,ClawNodeSC
  global openFlag,closeFlag
  robotController = getComponent()
  
  delayTime = None
  if robotController.getProperty("HandleClaw::Enabled").Value:
    if(hasattr(robotController,"HandleClaw::SynchronnousFollowingTime")):
      delayTime = robotController.getProperty("HandleClaw::SynchronnousFollowingTime").Value
        
  if robotController == None:
    return
  if ClawNode == None:
    return
  
  handleClawSignal = robotController.findBehaviour("handleclawprocess")
  list = ClawNode.findBehavioursByType( VC_RSLEXECUTOR )
  openSignal = comp.findBehaviour( "Open" )
  closeSignal = comp.findBehaviour( "Close" )
  executor = None
  if len(list):
    executor = list[0]
  if executor == None:
    return
  
  OpenName = "Open"
  CloseName = "Close"
  OpenNames = []
  CloseNames = []
  OpenNames.append( OpenName )
  CloseNames.append( CloseName )
  OpenRoutine = getExistSeq( executor, OpenNames)
  CloseRoutine = getExistSeq( executor, CloseNames)
  if OpenRoutine == []:
    return
  if CloseRoutine == []:
    return
  
  #print("="*1000)
  open = None
  while True:
    delay(min(sim.SimSpeed,0.01))# 防止无限循环卡死
    if openFlag == True:
      #print( "open signal !!!" )
      executor.callRoutine( OpenRoutine[0].Name,True ) # True 等待卡爪动作完成执行接下来动作
      handleClawSignal.signal( "openFinish" )
      #ParentNodeSC.moveJoint(0,90)
      openSignal.signal( False )
      #print("True")
    if closeFlag == True:
      #print( "close signal!!!" )  
      SetCycleTime(CloseRoutine[0],delayTime)
      if delayTime > 0:
        executor.callRoutine( CloseRoutine[0].Name,False )
      else:
        executor.callRoutine( CloseRoutine[0].Name,True )
        
      handleClawSignal.signal( "closeFinish" )
      #ParentNodeSC.moveJoint(0,0)
      closeSignal.signal( False )
      
    