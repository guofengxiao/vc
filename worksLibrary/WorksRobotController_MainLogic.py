# 3.6.1.5.3
from vcScript import *
import vcVector
import vcMatrix
from operator import itemgetter
from vcHelpers.Robot import *

app = getApplication()
sim = getSimulation()
comp = getComponent()

# 机器人执行RSL
def doRSL(robo, mtnType, robotMove, Basenode = None,pickApproachFlag = False,moveAway = False):
  #print("local doRSL!")
  robo.Executor.deleteSubRoutine('vcHelperMove')
  subr = robo.Executor.createSubRoutine('vcHelperMove')
  statement = subr.createStatement(mtnType)
  
  # PTP点 
  if robo.Configuration != -1 and mtnType == VC_STATEMENT_PTPMOTION:
    if type(robo.Configuration) == str:
      if robo.Configuration in robo.ConfigurationsList:
        conf = robo.ConfigurationsList.index(robo.Configuration)
        robotMove.RobotConfig = conf
      else:
        print 'Configuration name "%s" not found' % (robo.Configuration)
    else:
      robotMove.RobotConfig = robo.Configuration
  if Basenode:
    robo.Controller.Bases[robo.ActiveBase].Node = Basenode  
  
  statement.readFromTarget(robotMove)

  # 更新机器人速度
  updSpeeds(robo, statement)
  if robo.MotionTime >= 0:
    statement.CycleTime = robo.MotionTime
  # 执行 vcHelperMove sequence
  # 函数原型：vcRSLProgramExecutor.callRoutine[String routine[boolean suspend]]
  # 默认形参 suspend = True 表示 等待消耗时间
  robo.Executor.callRoutine('vcHelperMove')
  
  delayTime = 0.0
  if comp.getProperty("HandleClaw::Enabled").Value and moveAway == False:
    HandleClawClose()
  if(hasattr(comp,"HandleClaw::SynchronnousFollowingTime")):
    delayTime = comp.getProperty("HandleClaw::SynchronnousFollowingTime").Value
  
  if pickApproachFlag == True:
    delayTime = 0.0
  # 利用当前点base移动拖动机器人移动
  # 时间设置 直到局部变量消失或者机器人执行下一点时刻
  delay(delayTime)
  # bug  ReferenceError: Object no longer exists.
  robo.Controller.Bases[robo.ActiveBase].Node = None # self.ActiveBase

  if Basenode:
    Basenode.rebuild()

    
  # 动态点后置输出
  if robo.RecordRSL:
    recRSL(robo, mtnType, robotMove, Basenode)
  mtx = vcMatrix.new()
  
  # ？？ 记录当前点移动
  # 
  robotMove.Target = mtx
  robo.MotionTime = 0

class vcRobotLocal(vcRobot):
  
  # 线性抓取离开
  # 水平速度递减为0 
  # fly功能
  def moveAway(self, Tz = 200,fly = False):
    #print("local move Away")
    robotMove = self.Controller.createTarget()
    if self.ActiveTool:
      try:
        robotMove.ToolName = self.Controller.Tools[self.ActiveTool-1].Name
      except:
        robotMove.ToolName = self.ActiveTool
    targetMtx = vcMatrix.new(robotMove.Target)
    targetMtx.translateRel(0,0,-Tz)
    rc = robotMove.RobotConfig
    robotMove.MotionType = VC_MOTIONTARGET_MT_LINEAR
    #robotMove.TargetMode = VC_MOTIONTARGET_TM_ROBOTROOT
    robotMove.Target = targetMtx

    # Call RSL
    #print("moveAway:")
    doRSL(self, VC_STATEMENT_LINMOTION, robotMove,None,True,True)
    
  def linearMoveToMtx_ExternalBase(self, Basenode, ToMtx = idMTX, Tx = 0,Ty = 0,Tz = 0,Rx = 0,Ry = 0,Rz = 0, pickApproach = 0):
    targetMtx = vcMatrix.new(ToMtx)
    targetMtx.translateRel(Tx, Ty, Tz)
    targetMtx.rotateRelX(Rx)
    targetMtx.rotateRelY(Ry)
    targetMtx.rotateRelZ(Rz)
    base = self.Controller.Bases[self.ActiveBase]
    self.Controller.Bases[self.ActiveBase].Node = Basenode # <= 2011 bug fix
    if Basenode:
      Basenode.update()
      self.Component.update()
    base.PositionMatrix = vcMatrix.new()
    robotMove = self.Controller.createTarget()
    rc = robotMove.RobotConfig
    robotMove.BaseName = self.Controller.Bases[self.ActiveBase].Name
    robotMove.TargetMode = VC_MOTIONTARGET_TM_NORMAL
    robotMove.Target = targetMtx
    #printMatrix(targetMtx)
    robotMove.MotionType = VC_MOTIONTARGET_MT_LINEAR
    if self.ActiveTool:
      try:
        robotMove.ToolName = self.Controller.Tools[self.ActiveTool-1].Name
      except:
        robotMove.ToolName = self.ActiveTool

    # Call RSL
    if pickApproach == 0:
      doRSL(self, VC_STATEMENT_LINMOTION, robotMove, Basenode)
    else:
      doRSL(self, VC_STATEMENT_LINMOTION, robotMove, Basenode,pickApproachFlag = True)
  
  def pickMovingPart(self, Part, Approach = 200, Tx = 0,Ty = 0,Tz = 0,Rx = 0,Ry = 0,Rz = 0, ForceUpdatePos = True, rrx = -1, rry = -1, rrz = -1,fromcomponent = None): # (debug args: ForceUpdatePos = True, rrx = -1, rry = -1, rrz = -1) (future arg#Advancement = 200)
      if Part:
        if self.RecordRSL:
          print 'Warning: vcHelpers.Robot: From component', self.Component.Name
          print '       - RSL-statement base configuration may need reconfiguration before post-processing.'
          print '       - External base node is overwritten on every statement creation.'
        
  
        ToMtx = vcMatrix.new()
        partHeight = Part.Geometry.BoundDiagonal.Z*2
        #self.jointMoveToMtx_ExternalBase(ToMtx, Part, Tx = Advancement, Tz = partHeight+Approach, Rx = 180)
        tx, ty, tz, rx, ry, rz = Tx, Ty, Tz, Rx, Ry, Rz
        rots = fromcomponent.getProperty("Advanced::PickRotation").Value.split(",")
        Rx = float(rots[0])
        Ry = float(rots[1])
        Rz = float(rots[2])
        
        self.linearMoveToMtx_ExternalBase(Part, ToMtx, Tx = Tx, Ty = Ty, Tz = partHeight + Tz + Approach, Rx = 180 + Rx, Ry = 0 + Ry, Rz = Rz, pickApproach = Approach)
        
        self.linearMoveToMtx_ExternalBase(Part, ToMtx, Tx = Tx, Ty = Ty, Tz = partHeight + Tz, Rx = 180 + Rx, Ry = 0 + Ry, Rz = Rz)
  
        #delay(0.5)
        #self.linearMoveToMtx_ExternalBase(Part, ToMtx, Tx = Tx, Ty = Ty, Tz = partHeight + Tz, Rx = 180 + Rx, Ry = 0 + Ry, Rz = Rz)
        self.graspComponent(Part)
        # Force part on right position
        if ForceUpdatePos:
          # Get tool position
          if self.ActiveTool:
            if type(self.ActiveTool) == type('DummyString'):
              for i in self.Controller.Tools:
                if i.Name == self.ActiveTool:
                  toolpos = i.PositionMatrix
            else:
              toolpos = self.Controller.Tools[self.ActiveTool-1].PositionMatrix
          else:
            toolpos = vcMatrix.new()
          toolpos.rotateRelZ(rz*rrz)
          toolpos.rotateRelY(ry*rry)
          toolpos.rotateRelX(rx*rrx)
          toolpos.translateRel(-tx,ty,tz+partHeight)


          partpos = vcMatrix.new(Part.PositionMatrix)
          v = partpos.P
          v.X = 0.0 + toolpos.P.X
          v.Y = 0.0 + toolpos.P.Y
          v.Z = 0.0 + toolpos.P.Z
          partpos.P = v
          #printMatrix( Part.PositionMatrix )
          Part.PositionMatrix = partpos
          #printMatrix( Part.PositionMatrix )
          Part.update()


        # Move away
        self.moveAway(Approach)

      
def OnSignal(signal):
  global queue, tasksignal
  if signal == tasksignal:
    val = signal.Value
    task_data = val.split("&")
    priority_value = 9999
    queue.append( [priority_value, sim.SimTime, task_data] )


def OnStart():
  # clean up cycle times props
  for p in comp.Properties:
    if "CycleTimes::" in p.Name:
      comp.deleteProperty(p)
  for c in app.Components:
    if c.getProperty("InitialLoc"):
      c.InitialLoc = c.WorldPositionMatrix


def getRobotLocal(arg = None, SetActionMode = True):
  argType = None
  comp = getComponent
  # Test if the given argument is a robot component (vcComponent), interface name (string) or None
  
  # Component - arg
  try:
    arg.VCID
    argType = 'component'
  
  except:
    if not argType and type(arg) == type('dummystring'):
  # Iface name - arg
      argType = 'iface'
  # None - arg
  
  # Find robot component and save it to roboComp - variable
  if argType == None:
    roboComp = getComponent()
  elif argType == 'component':
    roboComp = arg
  elif argType == 'iface':
    iface = getComponent().findBehaviour(arg)
    if not iface or iface.Type != VC_ONETOONEINTERFACE:
      print 'Error: vcHelpers.Robot was unable to find interface called',arg,'in component "',getComponent().Name,'".'
      return None
    else:
      roboComp = iface.ConnectedComponent
      if not roboComp:
        print 'Error: vcHelpers.Robot was unable to find robot.'
        print '       - Connect robot to "',iface.Name,'"-interface.'
    
  # Find Controller, Executor, GraspContainer and signalMap
  controllers = roboComp.findBehavioursByType(VC_ROBOTCONTROLLER)
  executors = roboComp.findBehavioursByType(VC_RSLEXECUTOR)
  try:
    controller = controllers[0]    
  except:
    print 'Error: vcHelpers.Robot was unable to find controller in component "',roboComp.Name,'".'
    return
  
  try:
    executor = executors[0]
    executor.ActionMode = SetActionMode
    signalMapOut = executor.DigitalMapOut
    signalMapIn = executor.DigitalMapIn
  except:
    print 'Error: vcHelpers.Robot was unable to find executor in component "',roboComp.Name,'".'
    return
    
  graspContainers = roboComp.findBehavioursByType(VC_COMPONENTCONTAINER)
  if not graspContainers:
    print 'Error: vcHelpers.Robot was unable to find graspContainers in component "',roboComp.Name,'".'
    print '       - Robot model is being manipulated: GraspContainer added.'
    graspContainer = controller.FlangeNode.createBehaviour(VC_COMPONENTCONTAINER,'GraspContainer')
  else:
    graspContainer = graspContainers[-1]
    

  
  return vcRobotLocal(roboComp, controller, executor, graspContainer, signalMapIn, signalMapOut)

def ExistPlaceCouple( pickTask,queueTask ):
  global queue
  pickComp, pickProdid, pickTaskname, eoat, tcp = pickTask[1:]
  #print "ExistPlaceCouple QUEUE:", queue
  for tasks_data_q in queue:
    priority, task_creation_time, task = tasks_data_q
    if task[0] == "Place":
      placeComp, PlaceProdid, taskname, eoat, tcp, to_location_xyz = task[1:]
      #print "PlaceProdid:",PlaceProdid
      #print "pickProdid:",pickProdid
      if PlaceProdid == pickProdid:
        return True
        break
  return False

def CheckactiveSignalValue(taskname):
  note = getComponent().getProperty("Advanced::Note")
  if taskname in note.Value:
    return True
  else:
    return False


def SortQueuePatternPriority():
  global queue
  result = []
  for index,tasks_data_q in enumerate(queue):
    priority, task_creation_time, task = tasks_data_q
    if task[0] == "Pick" and CountsTasks_data_q(tasks_data_q) == 1:
      priority = priority + 1
    result.append( [priority,task_creation_time,task])
  #print "result:",result
  
  result = sorted(result, key=lambda x:(x[0],x[1]))
  queue = result
  pass

def CountsTasks_data_q(tasks_data_q):
  global queue
  priority, task_creation_time, task = tasks_data_q
  count = 0
  for tasks_data_q in queue:
    priorityQ, task_creation_timeQ, taskQ = tasks_data_q
    if task_creation_timeQ == task_creation_time and SameTaskQ(task,taskQ):
      count += 1
  return count


def SameTaskQ(taskq1,taskq2):
  for i in range(len(taskq1)):
    if taskq1[i] != taskq2[i]:
      return False
      break
  return True

def CheckWorksProcessTask(worksProcess,task):
  taskNote = worksProcess.findBehaviour("Task::Task")
  if task in taskNote.Note:
    return True
  else:
    return False

def OnRun():
  global queue, tasksignal, track, servo, robo, robot_cont, robot_controller, statistics
  global prepicktime,preplacetime
  global robotLocal
  statistics = comp.findBehaviour('Statistics')
  comp.reserver = ""
  queue = [] #list of pending tasks
  robot = comp.findBehaviour("RobotPnP").ConnectedComponent #robot in RobotPnP interface
  if not robot: return
  tasksignal = comp.findBehaviour("Task")
  try:
    robot_controller = robot.findBehavioursByType(VC_ROBOTCONTROLLER)[0]
    track = servo = None
  except:
    for child in robot.ChildComponents:
      controllers = child.findBehavioursByType(VC_ROBOTCONTROLLER)
      if controllers:
        track = robot
        servo = track.findBehavioursByType(VC_SERVOCONTROLLER)[0]
        robot = child
        break
      #endif
    else:
      print "Error in robot controller %s, No robot found"%comp.Name
      suspendRun()
  robo = getRobot(robot)
  robotLocal = getRobotLocal( robot )
  robot_controller = robo.Controller
  configs = robo.ConfigurationsList[:]
  configs.append('Automatic')
  if config_prop.StepValues != configs:
    config_prop.StepValues = configs
  robot_executor = robo.Executor
  robot_cont = robot.findBehaviour(comp.getProperty("Advanced::GraspContName").Value)
  if not robot_cont:
    robot_cont = robo.GraspContainer
  robot_tool_iface = robot.findBehaviour(comp.getProperty("Advanced::ToolIntName").Value)
  if not robot_tool_iface:
    robot_tool_iface = robot_controller.FlangeNode.getBehavioursByType(VC_ONETOONEINTERFACE)[0]
    
  multiPickTaskList = [x.strip() for x in comp.MultiPickTaskList.split(",")]
  multiPickTaskList = [x for x in multiPickTaskList if x]
  serialTaskList = [x.strip() for x in comp.SerialTaskList.split(",")]
  serialTaskList = [x for x in serialTaskList if x]
  autohome_delay = comp.getProperty("AutoHoming::Delay").Value
  if autohome_delay <= 0: autohome_delay = 1.0
  autohome_enabled = comp.getProperty("AutoHoming::Enabled").Value
  robot_executor.createSubRoutine("home")
  for r in robot_executor.SubRoutines:
    if r.Name == "home":
      home_routine = r
      break
  res_tcp = []
  last_d = "None"
  cached_home_joint_values = [robo.Component.getProperty(x.Name).Value for x in robo.Controller.Joints if not x.ExternalController] #current joint values used for homing if homing statements are not taught
  at_home = True

  while True:
    comp.Busy = False
    statistics.State = 'Idle'
    if autohome_enabled and not at_home:
      trigged = condition(lambda: queue ,autohome_delay)
    else:
      trigged = condition(lambda: queue )
    calcStats()
    if not trigged:
      if home_routine.StatementCount:
        robot_executor.callRoutine("home")
      else:
        robo.driveJoints(*cached_home_joint_values)
        at_home = True
      continue
    at_home = False
    task_data = queue[0]
    comp.Busy = True
    priority, task_creation_time, task = task_data
    if task[3] in serialTaskList:
      ##
      ## serialTaskList
      ##
      delay(0.1) #so that all calls can come in at the same sim_time
      order = []
      for runner_i, d in enumerate(serialTaskList):
        delay(0.01)
        SortQueuePatternPriority()
        #print "queue:",queue
        for tasks_data_q in queue:
          #print "tasks_data_q:",tasks_data_q
          priority, task_creation_time, q = tasks_data_q
          delay(0.01)
          #print("="*47)
          if q[3] == d:
            task = q
            try: next = serialTaskList[last_d+1]
            except: next = "None"
            #print "ExistPlaceCouple( task,queue ):",ExistPlaceCouple( task,queue )
            if task[0] == "Pick" and task[5] not in res_tcp and ((d == next or next == "None")or len(res_tcp) == 0):
              
              #last_d = runner_i
              #queue.remove(tasks_data_q)
              fromcomp, prodid, taskname, eoat, tcp = task[1:]
              fromcomponent = app.findComponent(fromcomp)
              TransportInFlag = CheckWorksProcessTask(fromcomponent,"TransportIn")
              if(hasattr(fromcomponent,"Advanced::PickDynamicRandEnabled") and fromcomponent.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
                fromcont = fromcomponent.findBehaviour("Path__HIDE__")
              else:
                fromcont = fromcomponent.findBehaviour("Container__HIDE__")
              parts = [c for c in fromcont.Components if c.ProdID == prodid]
              #print"Pick size of parts," ,len(parts)
              if len([c for c in fromcont.Components if c.ProdID == prodid]) <= 0:
                break
              part = [c for c in fromcont.Components if c.ProdID == prodid][-1]
              part.update()
              res_tcp.append(tcp)
              prio = priority
              toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
              
              dynamicFlag = None
              if(hasattr(fromcomponent,"Advanced::PickDynamicRandEnabled") and fromcomponent.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
                dynamicFlag = True
              else:
                dynamicFlag = False
              #print "fromcomponent:",fromcomponent.Name
              activeSignalValue = getComponent().findBehaviour("Active").Value
              idSame = (part.Name == prodid)
              #print "idSame:",idSame
              if idSame or ExistPlaceCouple( task,queue ) or CheckactiveSignalValue(taskname):
                try:
                  last_d = runner_i
                  queue.remove(tasks_data_q)
                  doPick(part, fromcomponent, taskname, tcp, prodid,dynamicFlag)
                except:
                  res_tcp.remove(tcp)
              else:
                res_tcp.remove(tcp)
            if task[0] == "Place" and task[2] in [x.ProdID for x in robot_cont.Parent.ChildComponents if x.getProperty('ProdID')]:
              #print "place :",task
              last_d = runner_i
              tocomp, prodid, taskname, eoat, tcp, to_location_xyz = task[1:]
              if tcp in res_tcp:
                queue.remove(tasks_data_q)
                res_tcp.remove( tcp )
                to_location_xyz = [float(x) for x in to_location_xyz.split(',')]
                prio = priority
                tocomponent = app.findComponent(tocomp)
                tocont = tocomponent.findBehaviour("Container__HIDE__")
                part = [x for x in robot_cont.Parent.ChildComponents if x.getProperty('ProdID') and x.ProdID == prodid][-1]
                toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
                doPlace( part, tocomponent, taskname, tcp, prodid, to_location_xyz, tocont)
              
    elif task[1] in multiPickTaskList:
      ##
      ## multiPickTaskList
      ##
      taskname, fromcomp, tocomp, prodid, location_xyz, eoat, tcp = task[1:]
      multi_tasks = []
      for taskname in multiPickTaskList:
        for item2 in queue:
          priority2, task_creation_time2, task2 = item2
          if taskname == task2[1]:
            multi_tasks.append( item2 )
      if len(multi_tasks) >= len(multiPickTaskList):
        parts = []
        for item in multi_tasks:
          # loop multi picks
          queue.remove( item )
          priority, task_creation_time, task = item
          taskname, fromcomp, tocomp, prodid, to_location_xyz, eoat, tcp = task[1:]
          prio = 999
          fromcomponent = app.findComponent(fromcomp)
          fromcont = fromcomponent.findBehaviour("Container__HIDE__")
          part = [c for c in fromcont.Components if c.ProdID == prodid][0]
          part.update()
          toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
          doPick(part, fromcomponent, taskname, tcp, prodid)
          parts.append( part ) 
        for item, part in zip(multi_tasks, parts):
          # loop multi places
          priority, task_creation_time, task = item
          taskname, fromcomp, tocomp, prodid, to_location_xyz, eoat, tcp = task[1:]
          to_location_xyz = [float(x) for x in to_location_xyz.split(',')]
          prio = 999
          tocomponent = app.findComponent(tocomp)
          tocont = tocomponent.findBehaviour("Container__HIDE__")
          part.update()
          toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
          doPlace( part, tocomponent, taskname, tcp, prodid, to_location_xyz, tocont)
      else:
        delay(1.0)
        queue.remove(task_data)
        queue.append(task_data)
    
    elif task[0] == "Transport":
      ##
      ## Transport
      ##
      queue.remove(task_data)
      taskname, fromcomp, tocomp, prodid, to_location_xyz, eoat, tcp = task[1:]
      to_location_xyz = [float(x) for x in to_location_xyz.split(',')]
      prio = 999
      fromcomponent = app.findComponent(fromcomp)
      tocomponent = app.findComponent(tocomp)
      fromcont = fromcomponent.findBehaviour("Container__HIDE__")
      tocont = tocomponent.findBehaviour("Container__HIDE__")
      part = None
      part = [c for c in fromcont.Components if c.ProdID == prodid][0]
      part.update()
      toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
      #if not toolOK: tcp = ''
      doPick(part, fromcomponent, taskname, tcp, prodid)
      doPlace( part, tocomponent, taskname, tcp, prodid, to_location_xyz, tocont)

    elif task[0] == "RobotProcess":
      ##
      ## RobotProcess
      ##
      queue.remove(task_data)
      caller_name, routine, eoat, tcp = task[1:]
      caller = app.findComponent(caller_name)
      sig = caller.findBehaviour("TaskDone")
      toolOK = doEoatChange(robo, robot_tool_iface, eoat, tcp)
      routines = robot_executor.SubRoutines
      for r in routines:
        if r.Name == routine:
          statistics.State = 'Process'
          robot_executor.callRoutine(routine)
          break
      else:
        print "ERROR: No routine: " + routine + " found"
      sig.signal("&".join(task))

    else:
      delay(1.0)




do_TAT_stamps = True
def stampTAT(part, postfix, sim_time = None):
  if do_TAT_stamps:
    the_time = sim.SimTime
    id = part.ProdID
    stamp_name = 'TAT_' + id + '_' + postfix
    prop = part.createProperty(VC_REAL, stamp_name)
    if prop: 
      if sim_time:
        prop.Value += sim_time
      else:
        prop.Value = the_time
    for kid in part.ChildComponents:
      id = kid.ProdID
      stamp_name = 'TAT_' + id + '_' + comp.Name + '_' + postfix
      prop = kid.createProperty(VC_REAL, stamp_name)
      if prop: 
        if sim_time:
          prop.Value += sim_time
        else:
          prop.Value = the_time   


def runframes(dat,task,thecomp,robo,toolname):
  for i in range(20):
    fname = dat+task + "_"+str(i)
    frame = thecomp.findFeature(fname)
    if frame:
      thecomp.update()
      mtx = thecomp.WorldPositionMatrix * frame.NodePositionMatrix
      moveTrack( mtx ) 
      setTcp(robo, toolname)
      if config_prop.Value in robo.ConfigurationsList:
        robo.Configuration = config_prop.Value
      robo.JointSpeed = 100
      robo.jointMoveToMtx( mtx )
      
def countframes(dat,task,thecomp):
  count = 0
  for i in range(20):
    fname = dat+task + "_"+str(i)
    frame = thecomp.findFeature(fname)
    if frame: count += 1
  return count
      

def check_approach(ccomp,dir):
  if dir == "pick":
    dir = ccomp.getProperty("Advanced::PickDirection").Value
    d = ccomp.getProperty("Advanced::PickApproach").Value
    vec = vcVector.new(0,0,0)
    if dir == "Z": vec.Z = -d
    if dir == "-Z": vec.Z = d
    if dir == "X": vec.X = d
    if dir == "-X": vec.X = -d
    if dir == "Y": vec.Y = -d
    if dir == "-Y": vec.Y = d
    return vec
  if dir == "place":
    dir = ccomp.getProperty("Advanced::PlaceDirection").Value
    d = ccomp.getProperty("Advanced::PlaceApproach").Value
    vec = vcVector.new(0,0,0)
    if dir == "Z": vec.Z = -d
    if dir == "-Z": vec.Z = d
    if dir == "X": vec.X = d
    if dir == "-X": vec.X = -d
    if dir == "Y": vec.Y = -d
    if dir == "-Y": vec.Y = d
    return vec


def check_picklocation(ccomp,dir,part):
#  partHeight = part.BoundDiagonal.Z*2
  partBC = part.BoundCenter
  partBD = part.BoundDiagonal
  partTop = part.BoundDiagonal.Z+part.BoundCenter.Z  
  target = part.WorldPositionMatrix
  target.translateRel( partBC.X, partBC.Y, partBC.Z )
  
  if dir == "pick":
    dir = ccomp.getProperty("Advanced::PickDirection").Value
    vec = vcVector.new(0,0,0)
    if dir == "Z":
      target.rotateRelY(0)
      target.translateRel( 0,0, partBD.Z)
    if dir == "-Z":
      target.rotateRelY(180)
      target.translateRel( 0,0, partBD.Z)
    if dir == "X":
      target.rotateRelY(90)
      target.translateRel( 0,0, partBD.X)
    if dir == "-X":
      target.rotateRelY(-90)
      target.translateRel( 0,0, partBD.X)
    if dir == "Y":
      target.rotateRelX(-90)
      target.translateRel( 0,0, partBD.Y)
    if dir == "-Y":
      target.rotateRelX(90)
      target.translateRel( 0,0, partBD.Y)
    return target
  if dir == "place":
    dir = ccomp.getProperty("Advanced::PlaceDirection").Value
    d = ccomp.getProperty("Advanced::PlaceApproach").Value
    vec = vcVector.new(0,0,0)
    if dir == "Z": vec.Z = -d
    if dir == "-Z": vec.Z = d
    if dir == "X": vec.X = -d
    if dir == "-X": vec.X = d
    if dir == "Y": vec.Y = -d
    if dir == "-Y": vec.Y = d
    return vec
    
def moveTrack(target, given_state = ''):
  '''Move the track closer to the target'''
  global robot_controller, servo, statistics
  if track:
    state = statistics.State
    if given_state:                      statistics.State = given_state
    elif robot_cont.ComponentCount == 0: statistics.State = 'TransferWithoutPart'
    else:                                statistics.State = 'TransferWithPart'
    position_offset = comp.getProperty('Track::PositionOffset').Value
    ret = calcTargetTrackPos(track, target, servo, Y_PickOffset = position_offset)
    position_tolerance = comp.getProperty('Track::PositionTolerance').Value # track move is not done if the track is already close enough
    if abs(ret - servo.Joints[0].CurrentValue) > position_tolerance:
      #robo.driveJoints(0) # Only first joint give => other joints use current value
      joint_vals = readJointString()
      for v in joint_vals:
        robot_controller.setJointTarget(v[0], v[1])
      externals = [(yy,xx) for yy,xx in enumerate(robot_controller.Joints) if xx.Controller != robot_controller] # handle external axis
      for yy, xx in externals:                                                       # handle external axis
        robot_controller.setJointTarget(yy, xx.CurrentValue)                                       # handle external axis
      robot_controller.move()
      servo.move( ret )
    statistics.State = state

def readJointString():
  j_string = comp.getProperty('Track::RobotJointsOnMove').Value
  vals = map(gimme_joint, [(i,x.strip()) for i,x in enumerate( j_string.split(',') ) ] )
  return vals

def gimme_joint(f):
  i,x = f
  x = robot_controller.Joints[i].CurrentValue if x == '-' else float(x)
  return i,x
  
track_position_tolerance = 300
def calcTargetTrackPos(servotrack, target, servo, Y_PickOffset = 0):
  '''
  Calulates projected position of the target on the servo track
    returns the needed track joint value 
  '''
  params = {}
  for i in servotrack.Properties:
    params[i.Name] = i.Value
  trackInvPos = servotrack.InverseWorldPositionMatrix
  if type(target) == type(dummyMtx):
    # Target is given as Matrix
    targetPos = target
  else:
    # Target is given as node
    targetPos = target.WorldPositionMatrix 
  relPosMtx = trackInvPos * targetPos
  # Horizontal
  axis_dir = comp.getProperty('Track::TrackAxis').Value
  relPosH = getattr(relPosMtx.P, axis_dir) + Y_PickOffset
  if comp.getProperty('Track::FlippedJoint').Value:
    relPosH *= -1
  joint1Min, joint1Max = getJointMinMax( servo.Joints[0] )
  if relPosH < joint1Min:
    relPosH = joint1Min
  elif relPosH > joint1Max:
    relPosH = joint1Max
  return relPosH
  #return 400

def getJointMinMax( joint ):
  '''Read the maximum and minimum values of the joint'''
  controller = joint.Controller
  component = controller.Component
  prop = component.getProperty( joint.Name )
  minValue = prop.MinValue
  maxValue = prop.MaxValue
  return minValue, maxValue

dummyMtx = vcMatrix.new()


def calcStats():
  global statistics
  t = sim.SimTime/100.0
  statsProps['Utilization'].Value = statistics.BusyPercentage / 100.0
  statsProps['Idle'].Value = statistics.IdlePercentage * t
  statsProps['TransferWithoutPart'].Value = statistics.getPercentage('TransferWithoutPart') * t
  statsProps['TransferWithPart'].Value = statistics.getPercentage('TransferWithPart') * t
  statsProps['PickingTime'].Value = statistics.getPercentage('Picking') * t
  statsProps['PlacingTime'].Value = statistics.getPercentage('Placing') * t
  statsProps['ToolChangeTime'].Value = statistics.getPercentage('ToolChanging') * t
  statsProps['ProcessTime'].Value = statistics.getPercentage('Process') * t

def eoat_init_location(eoat):
  eoat.update()
  if not eoat.getProperty("InitialLoc"):
    prop = eoat.createProperty(VC_MATRIX, "InitialLoc")
    eoat.update()
    target_pos = eoat.WorldPositionMatrix
    prop.Value = target_pos
  else:
    target_pos = eoat.InitialLoc
  return target_pos


def returnEOAT(robo, robot_tool_iface):
  global statistics
  state = statistics.State
  statistics.State = 'ToolChanging'
  curtool = robot_tool_iface.ConnectedComponent
  tar_pos = eoat_init_location( curtool )
  moveTrack(tar_pos, 'ToolChanging')
  flange_tcp = flange_tcp_prop.Value
  setTcp(robo, flange_tcp)
  if config_prop.Value in robo.ConfigurationsList:
    robo.Configuration = config_prop.Value
  robo.JointSpeed = 100
  robo.jointMoveToMtx(tar_pos)
  robo.Component.update()
  t_ints = curtool.findBehavioursByType(VC_ONETOONEINTERFACE)
  t_int = t_ints[0]
  robot_tool_iface.unConnect(t_int)
  curtool.PositionMatrix = tar_pos
  statistics.State = state

def getEOAT(robo, robot_tool_iface, eoat_comp):
  global statistics
  state = statistics.State
  statistics.State = 'ToolChanging'
  tar_pos = eoat_init_location( eoat_comp )
  moveTrack(tar_pos, 'ToolChanging')
  flange_tcp = flange_tcp_prop.Value
  setTcp(robo, flange_tcp)
  if config_prop.Value in robo.ConfigurationsList:
    robo.Configuration = config_prop.Value
  robo.JointSpeed = 100
  robo.jointMoveToMtx(tar_pos)
  robo.Component.update()
  t_ints = eoat_comp.findBehavioursByType(VC_ONETOONEINTERFACE)
  t_int = t_ints[0]
  robot_tool_iface.connect(t_int)
  statistics.State = state

def doEoatChange(robo, robot_tool_iface, eoat, tcp):
  # eoat ~ End Of Arm Tool
  global statistics
  state = statistics.State
  statistics.State = 'ToolChanging'
  eoat = eoat.strip()
  sim_time = sim.SimTime
  if not eoat:
    ##
    ## task doesn't require an eoat
    if robot_tool_iface.ConnectedComponent:
      # robot is holding an eoat => return the eoat
      returnEOAT(robo, robot_tool_iface)
      statistics.State = state
      return True
    else:
      # robot is not holding an eoat and  => all fine
      statistics.State = state
      return True
  elif eoat:
    ##
    ## task requires an eoat
    if robot_tool_iface.ConnectedComponent and robot_tool_iface.ConnectedComponent.Name == eoat:
      # robot is holding the correct eoat => all fine
      statistics.State = state
      return True
    if robot_tool_iface.ConnectedComponent and robot_tool_iface.ConnectedComponent.Name != eoat:
      # robot is holding a wrong eoat => return the eoat
      returnEOAT(robo, robot_tool_iface)
    eoat_comp = app.findComponent(eoat)
    if eoat_comp:
      # correct eoat exists => get it
      getEOAT(robo, robot_tool_iface, eoat_comp)
      statistics.State = state
      return True
    else:
      # correct eoat doesn't exist in the layout => return false from this function
      warning( 'No EndOfArmTool -component named "%s" found in the layout.' % eoat )
      statistics.State = state
      return False
  statistics.State = state # just in case
 

testerINT = lambda x,y,tcp: x == tcp-1
testerSTR = lambda x,y,tcp: y.Name == tcp
def setTcp(robo, tcp):
  try:
    tcp = int(tcp)
    tester = testerINT
  except:
    tcp = tcp
    tester = testerSTR
  for i,t in enumerate(robo.Controller.Tools):
    if tester(i,t,tcp):
      break
  else:
    tcp = ''
  robo.ActiveTool = tcp


def getToolBaseFrame(robo, tcp):
  tool = None
  try: tcp = int(tcp)
  except: tcp = tcp
  if type(tcp) == int:
    tool = robo.Controller.Tools[tcp-1]
  else:
    for t in robo.Controller.Tools:
      if t.Name == tcp:
        tool = t
        break
  return tool


def getPartToTcpMtx( robo, tcp, part ):
  tool = getToolBaseFrame(robo, tcp)
  if tool:
    tool_offset = tool.PositionMatrix
    tool_node = tool.Node
    tool_node.update()
    tcp_pos = tool_node.WorldPositionMatrix * tool_offset
  else:
    flange = robo.Controller.FlangeNode
    flange.update()
    tcp_pos = flange.WorldPositionMatrix
  part.update()
  part_inv_mtx = part.InverseWorldPositionMatrix
  return part_inv_mtx * tcp_pos

def printP(m):
  v = m.P
  print v.X, v.Y, v.Z

def doPlace(part, tocomponent, taskname, tcp, prodid, to_location_xyz, tocont):
  global robo, robot_cont, statistics
  state = statistics.State
  statistics.State = 'Process'
  robo.Executor.createSubRoutine("preplace_" + taskname)
  robo.Executor.callRoutine("preplace_" + taskname)
  statistics.State = 'Placing'
  tocomponent.update()
  to_wpos = tocomponent.WorldPositionMatrix
  to_wpos.translateRel( *to_location_xyz )
  approach_mtx = vcMatrix.new(to_wpos)
  approach_value = tocomponent.getProperty("Advanced::PlaceApproach").Value
  approach_dir = tocomponent.getProperty("Advanced::PlaceDirection").Value
  

  
  if approach_dir == 'X': approach_mtx.translateRel(approach_value,0,0)
  elif approach_dir == '-X': approach_mtx.translateRel(-approach_value,0,0)
  elif approach_dir == 'Y': approach_mtx.translateRel(0,approach_value,0)
  elif approach_dir == '-Y': approach_mtx.translateRel(0,-approach_value,0) 
  elif approach_dir == 'Z': approach_mtx.translateRel(0,0,approach_value)
  elif approach_dir == '-Z': approach_mtx.translateRel(0,0,-approach_value) 

  
  if comp.getProperty("Advanced::PlaceBodyCoordinates").Value and tocomponent.Parent != tocomponent.World:
    local_target = tocomponent.PositionMatrix
    local_target.invert()
  else:
    posname = "MW_%s" % (prodid)
    if tocomponent.getFeature(posname) != None:
      local_target = tocomponent.getFeature(posname).NodePositionMatrix
    else:
      local_target = tocomponent.DefaultMatrix

  part_to_tcp_mtx = getPartToTcpMtx( robo, tcp, part )
  target = to_wpos * local_target * part_to_tcp_mtx
  approach_mtx = approach_mtx * local_target * part_to_tcp_mtx
  if len(to_location_xyz) >= 6:
    target.rotateRelZ( to_location_xyz[3] )
    target.rotateRelY( to_location_xyz[4] )
    target.rotateRelX( to_location_xyz[5] )
    approach_mtx.rotateRelZ( to_location_xyz[3] )
    approach_mtx.rotateRelY( to_location_xyz[4] )
    approach_mtx.rotateRelX( to_location_xyz[5] )
  moveTrack(target)
  sim.update() 

  totalMoves = 4 + countframes("preplace_",taskname,tocomponent) + countframes("postplace_",taskname,tocomponent) 
  dtime = tocomponent.getProperty("Advanced::PlaceDelay").Value
  try: ptime = (tocomponent.getProperty("Advanced::PlaceCycleTime").Value - dtime)/totalMoves
  except: ptime = 0.0
  if ptime > 0: robo.MotionTime = ptime

  runframes("preplace_",taskname,tocomponent,robo,tcp)
  setTcp(robo, tcp)
  if config_prop.Value in robo.ConfigurationsList:
    robo.Configuration = config_prop.Value
  if ptime > 0: robo.MotionTime = 2*ptime
  try:
    rots = tocomponent.getProperty("Advanced::PlaceRotation").Value.split(",")
  except: rots = None
  if rots != None:
    approach_mtx.rotateRelX(float(rots[0]))
    approach_mtx.rotateRelY(float(rots[1]))
    approach_mtx.rotateRelZ(float(rots[2]))  
  try:
    jointSpeed = tocomponent.getProperty("Advanced::PlaceApproachJointSpeed").Value
  except:
    jointSpeed = 100
  robo.JointSpeed = jointSpeed
  randTransX = None
  randTransY = None
  randTransZ = None
  randRotX = None
  randRotY = None
  randRotZ = None  
  if(hasattr(tocomponent,"Advanced::PlaceRandEnabled") and tocomponent.getProperty("Advanced::PlaceRandEnabled").Value == True):
    randTransX = tocomponent.getProperty("Advanced::PlaceTransRandX").Value
    randTransY = tocomponent.getProperty("Advanced::PlaceTransRandY").Value
    randTransZ = tocomponent.getProperty("Advanced::PlaceTransRandZ").Value
    randRotX = tocomponent.getProperty("Advanced::PlaceRotRandX").Value
    randRotY = tocomponent.getProperty("Advanced::PlaceRotRandY").Value
    randRotZ = tocomponent.getProperty("Advanced::PlaceRotRandZ").Value      
    approach_mtx.translateRel(randTransX,randTransY,randTransZ)
    approach_mtx.rotateRelX( randRotX )
    approach_mtx.rotateRelY( randRotY )
    approach_mtx.rotateRelZ( randRotZ )    
  else:
    pass
    
    
  robo.jointMoveToMtx( approach_mtx ) # Place 第一点
  if ptime > 0: robo.MotionTime = ptime
  if rots != None:
    target.rotateRelX(float(rots[0]))
    target.rotateRelY(float(rots[1]))
    target.rotateRelZ(float(rots[2]))
  
  if(hasattr(tocomponent,"Advanced::PlaceRandEnabled") and tocomponent.getProperty("Advanced::PlaceRandEnabled").Value == True):
    target.translateRel(randTransX,randTransY,randTransZ)
    target.rotateRelX( randRotX )
    target.rotateRelY( randRotY )
    target.rotateRelZ( randRotZ )    
  else:
    pass
  robo.linearMoveToMtx( target ) # Place 第二点
  if comp.getProperty("HandleClaw::Enabled").Value:
    HandleClawOpen()
  for tool in robo.Controller.Tools:
    if tcp == tool.Name:
      try:
        if tool.Node:
          toolComp = tool.Node.Component
          if toolComp != robo.Component:
            ioPort = tool.Node.Component.getProperty('IOPort')
            if ioPort:
              ioPort = tool.Node.Component.IOPort
              robo.Executor.setOutput( ioPort.Value, False )
              if dtime > 0: delay(dtime)
              while robo.Executor.DigitalMapIn.input( ioPort.Value ):
                delay(0.1)
              sim.update()
              tocont.grab(part)
              break
            else:
              compsig = toolComp.findBehaviour('ComponentSignal')
              if compsig: compsig.signal(tocont.Component)
              tasksig = toolComp.findBehaviour('Task')
              if tasksig:
                tasksig.signal('Release&'+tcp+'&'+comp.Name)
                condition( lambda: tasksignal.Value == tasksig.Value )
                queue.pop()
                break
      except:
        pass
  else:
    if dtime > 0: delay(dtime)
    sim.update()
    tocont.grab(part)
  
  if ptime > 0: robo.MotionTime = ptime
  if(hasattr(comp,"Advanced::PrePickTime")):
    robo.JointSpeed = comp.getProperty("Advanced::PrePickTime").Value
  else:
    robo.JointSpeed = 10
    print "Advanced::PrePlaceTime",robo.JointSpeed
  robo.linearMoveToMtx( approach_mtx )# Place 第三点
  runframes("postplace_",taskname,tocomponent,robo,tcp)
  robo.MotionTime = 0.0 

  robo.Executor.createSubRoutine("postplace_" + taskname)
  robo.Executor.callRoutine("postplace_" + taskname)
  statistics.State = state


def doPick(part, fromcomponent, taskname, tcp, prodid, dynamicFlag = False):
  global robo, robot_cont, statistics
  global robotLocal
  
  if dynamicFlag == True:
    if(hasattr(fromcomponent,"Advanced::PickApproach")):
      Approach = fromcomponent.getProperty( "Advanced::PickApproach" ).Value
    robotLocal.pickMovingPart(part, Approach,0,0,0,0,0,0,True,-1,-1,-1,fromcomponent)
    #robotLocal.pickMovingPart( part, Approach)
    return
  
  
  state = statistics.State
  statistics.State = 'Picking'
  
  if comp.getProperty("Advanced::PickBodyCoordinates").Value:
    for tool in robo.Controller.Tools:
      if tcp == tool.Name:
        target = part.WorldPositionMatrix * tool.PositionMatrix
        break
    else:
      warning( "Tool offset <%s> not found " % (tcp) )
  else:
    pick_frame = part.findFeature("pick")
    if pick_frame:
      target = part.WorldPositionMatrix * pick_frame.NodePositionMatrix
    else:
      target = check_picklocation(fromcomponent,"pick",part)
      target.rotateRelX(180) #flip position to grip in the desired orientation
    try:
      rots = fromcomponent.getProperty("Advanced::PickRotation").Value.split(",")
      target.rotateRelX(float(rots[0]))
      target.rotateRelY(float(rots[1]))
      target.rotateRelZ(float(rots[2]))
    except:
      warning( "Bad 'Advanced::PickRotation' param value in %s. Use 3 comma-separated values (e.g. 0,0,45.0)" % fromcomponent.Name )
  moveTrack(target) 
  approach_value = fromcomponent.getProperty("Advanced::PickApproach").Value
  approach_mtx = vcMatrix.new(target)
  approach_mtx.translateRel(0,0,-approach_value)
  robo.Executor.createSubRoutine("prepick_" + taskname)
  robo.Executor.callRoutine("prepick_" + taskname)
  totalMoves = 4 + countframes("prepick_",taskname,fromcomponent) + countframes("postpick_",taskname,fromcomponent) 
  dtime = fromcomponent.getProperty("Advanced::PickDelay").Value
  try: ptime = (fromcomponent.getProperty("Advanced::PickCycleTime").Value - dtime)/totalMoves
  except: ptime = 0.0
  if ptime > 0: robo.MotionTime = ptime
  runframes("prepick_",taskname,fromcomponent,robo,tcp)
  setTcp(robo, tcp)
  if config_prop.Value in robo.ConfigurationsList:
    robo.Configuration = config_prop.Value
  if ptime > 0: robo.MotionTime = 2*ptime
  
  if(hasattr(fromcomponent,"Advanced::PickApproachJointSpeed")):
    robo.JointSpeed = fromcomponent.getProperty("Advanced::PickApproachJointSpeed").Value
  else:
    robo.JointSpeed = 10
  robo.jointMoveToMtx( approach_mtx )# Pick 第一点
  
  if ptime > 0: robo.MotionTime = ptime

  if(hasattr(fromcomponent,"Advanced::PickTrans")):
    trans = fromcomponent.getProperty("Advanced::PickTrans").Value.split(",")
    transX = float(trans[0])
    transY = float(trans[1])
    transZ = float(trans[2])
    target.translateRel(transX,transY,transZ)
  else:
    pass
      
  robo.linearMoveToMtx( target ) # Pick 第二点
  if comp.getProperty("HandleClaw::Enabled").Value:
    #print("HandleClawClose()")
    HandleClawClose()
  dtime = fromcomponent.getProperty("Advanced::PickDelay").Value
  for tool in robo.Controller.Tools:
    if tcp == tool.Name:
      try:
        if tool.Node:
          toolComp = tool.Node.Component
          if toolComp != robo.Component:
            ioPort = tool.Node.Component.getProperty('IOPort')
            if ioPort:
              ioPort = tool.Node.Component.IOPort
              robo.Executor.setOutput( ioPort.Value, True )
              if dtime > 0: delay(dtime)
              while not robo.Executor.DigitalMapIn.input( ioPort.Value ):
                delay(0.1)
              sim.update()
              robot_cont.grab(part)
              break
            else:
              compsig = toolComp.findBehaviour('ComponentSignal')
              if compsig: compsig.signal(part)
              tasksig = toolComp.findBehaviour('Task')
              if tasksig: 
                tasksig.signal('Grab&'+tcp+'&'+comp.Name)
                condition( lambda: tasksignal.Value == tasksig.Value )
                queue.pop()
                break
      except:
        pass
  else:
    if dtime > 0: delay(dtime)
    sim.update()
    robot_cont.grab(part)
  if ptime > 0: robo.MotionTime = ptime
  if(hasattr(comp,"Advanced::PrePlaceTime")):
    robo.JointSpeed = comp.getProperty("Advanced::PrePlaceTime").Value
  else:
    robo.JointSpeed = 10
  robo.linearMoveToMtx( approach_mtx ) # Pick 第三点
  runframes("postpick_",taskname,fromcomponent,robo, tcp)
  robo.MotionTime = 0.0 
  statistics.State = 'Process'
  robo.Executor.createSubRoutine("postpick_" + taskname)
  robo.Executor.callRoutine("postpick_" + taskname)
  statistics.State = state

def HandleClawClose( part = None ):
  comp = getComponent()
  '''
  app = getApplication()
  HandleClawName = comp.getProperty("HandleClaw::Name").Value
  HandleClawComp = app.findComponent( HandleClawName )
  HandleControllerWL = None
  for component in HandleClawComp.ChildComponents:
    if component.Name.split("_")[0] == "HideHandleController":
      HandleControllerWL = component
  if HandleControllerWL == None:
    return
  '''
  closeSignal = comp.findBehaviour( "Close" )
  handleClawSignal = comp.findBehaviour( "handleclawprocess" )
  closeSignal.signal( True )
  condition( lambda: handleClawSignal.Value == "closeFinish" )
  
def HandleClawOpen():
  comp = getComponent()
  '''
  app = getApplication()
  HandleClawName = comp.getProperty("HandleClaw::Name").Value
  HandleClawComp = app.findComponent( HandleClawName )
  HandleControllerWL = None
  for component in HandleClawComp.ChildComponents:
    
    if component.Name.split("_")[0] == "HideHandleController":
      HandleControllerWL = component
  if HandleControllerWL == None:
    return
  '''
  OpenSignal = comp.findBehaviour( "Open" )
  handleClawSignal = comp.findBehaviour( "handleclawprocess" )
  OpenSignal.signal( True )
  condition( lambda: handleClawSignal.Value == "openFinish" )
  
def warning(stri):
  text = "Warning from '%s': %s" % (comp.Name, stri)
  print text


flange_tcp_prop = comp.getProperty('Advanced::FlangeTCP')
comp.createProperty(VC_BOOLEAN, 'Advanced::PickBodyCoordinates')
comp.createProperty(VC_BOOLEAN, 'Advanced::PlaceBodyCoordinates')
comp.createProperty(VC_REAL, 'Stats::Utilization')
comp.createProperty(VC_REAL, 'Stats::Idle')
comp.createProperty(VC_REAL, 'Stats::TransferWithoutPart')
comp.createProperty(VC_REAL, 'Stats::TransferWithPart')
comp.createProperty(VC_REAL, 'Stats::PickingTime')
comp.createProperty(VC_REAL, 'Stats::PlacingTime')
comp.createProperty(VC_REAL, 'Stats::ToolChangeTime')
comp.createProperty(VC_REAL, 'Stats::ProcessTime')

statsUtilization = comp.getProperty('Stats::Utilization')
statsIdle = comp.getProperty('Stats::Idle')
statsTransferWithoutPart = comp.getProperty('Stats::TransferWithoutPart')
statsTransferWithPart = comp.getProperty('Stats::TransferWithPart')
statsPickingTime = comp.getProperty('Stats::PickingTime')
statsPlacingTime = comp.getProperty('Stats::PlacingTime')
statsToolChangeTime = comp.getProperty('Stats::ToolChangeTime')
statsProcessTime = comp.getProperty('Stats::ProcessTime')
statsProps = {}
statsProps['Utilization'] = statsUtilization
statsProps['Idle'] = statsIdle
statsProps['TransferWithoutPart'] = statsTransferWithoutPart
statsProps['TransferWithPart'] = statsTransferWithPart
statsProps['PickingTime'] = statsPickingTime
statsProps['PlacingTime'] = statsPlacingTime
statsProps['ToolChangeTime'] = statsToolChangeTime
statsProps['ProcessTime'] = statsProcessTime

config_prop = comp.getProperty('Configuration')

robot_iface = comp.findBehaviour("RobotPnP")
def foo(other_iface, foobar):
  other_comp = other_iface.Component
  if other_comp.findBehavioursByType(VC_SERVOCONTROLLER) and not other_comp.findBehavioursByType(VC_ROBOTCONTROLLER):
    comp.PedestalHeight = 0
  config_prop.StepValues = ['Automatic']
  config_prop.Value = 'Automatic'
  if other_comp.findBehavioursByType(VC_ROBOTCONTROLLER):
    publish = other_comp.findBehaviour('PublishRSL')
    if not publish:
      publish = other_comp.createBehaviour(VC_ONETOMANYINTERFACE,'PublishRSL')
      publish.IsAbstract = True
      publish.ConnectionEditName = 'Interfaces::Publish RSL'
      s = publish.Sections[0]
      s.Name = 'Publish'
      f = s.createField(VC_RSLFIELD,'Publish')
      f.Publisher = other_comp.findBehavioursByType(VC_RSLEXECUTOR)[0]
      f.Publish = True
      
robot_iface.OnConnect = foo