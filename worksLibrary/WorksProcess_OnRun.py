# v3.6.1.3
from vcScript import *
import random, inspect
import vcMatrix
app = getApplication()
comp = getComponent()
sim = getSimulation()

global TransportInFlag
TransportInFlag = None

def OnStart():
  global userVariables, taskTimesList, compsInWorld
  global GlobalTimes, Global_ids, GlobalTimes2, Custom_patterns
  global taskcontrol, static_comps, task_signal, sync_signal, syncOK_signal
  userVariables = {}
  for p in comp.Properties:
    if p.IsVisible and 'UserVariables::' in p.Name and p.Group >= 100:
      userVariables[p.Name[len('UserVariables::'):]] = p
  #Adds all components in world to a dict
  compsInWorld = {}
  for x in app.Components:
    compsInWorld[x.Name] = x
  comp.Note = ''
  comp.SyncMessage = ''
  taskTimes.Note = ''
  taskTimesList = []
  static_comps = dict( [(x.Name, x) for x in app.Components] )
  #taskcontrol = compsInWorld.get("Works_TaskControl")
  taskcontrol = None
  for x in app.Components:
    if "Works_TaskControl" in x.Name:
      taskcontrol = x
      break
  if not taskcontrol:
    callerror("No task control component found!")
    suspendRun()
    return
  task_signal = taskcontrol.findBehaviour("Task")
  sync_signal = taskcontrol.findBehaviour("Sync")
  syncOK_signal = comp.findBehaviour("SyncOk")
  #
  # GlobalTimes
  #
  GlobalTimes = {"example":10.0}
  GlobalTimes2 = []
  timenote = taskcontrol.findBehaviour('GlobalTimes')
  if timenote:
    timenote.Note = timenote.Note.replace('\r\n','\n')
    content = timenote.Note
    for i in content.split('\n'):
      i = i.strip()
      if not i or ":" not in i or i[0] == '#':
        continue
      cells = [x.strip() for x in i.split(':')]
      if len(cells)>2:
        GlobalTimes2.append(cells)
      else:
        GlobalTimes[cells[0].strip()] = float(cells[1].strip())
  else:
    callerror( 'Old version of Task Control Component found in the layout. Please reload a new version of Works TaskControl from webcat.' ) 
  #
  # Global_ids
  #
  Global_ids = {"example":"foobar"}
  idnote = taskcontrol.findBehaviour('GlobalIDs')
  if idnote:
    global_id_comp = taskcontrol
    idnote = global_id_comp.findBehaviour('GlobalIDs')
    idnote.Note = idnote.Note.replace('\r\n','\n')
    content = idnote.Note
    for i in content.split('\n'):
      i = i.strip()
      if not i or ":" not in i or i[0] == '#':
        continue
      cells = [x.strip() for x in i.split(':')]
      Global_ids[cells[0].strip()] = cells[1].strip()
  else:
    callerror( 'Old version of Task Control Component found in the layout. Please reload a new version of Works TaskControl from webcat.' ) 
  #
  # Custom patterns
  #
  Custom_patterns = {}
  patterns_note = taskcontrol.findBehaviour('CustomPatterns')
  if patterns_note:
    patterns_note.Note = patterns_note.Note.replace('\r\n','\n')
    content = patterns_note.Note
    for_id = None
    for line in content.split('\n'):
      line = line.strip()
      if not line or line[0] == '#':
        continue
      if ('</' in line and '>' in line) or '<>' in line:
        # end block
        for_id = None
      elif '<' in line and '>' in line:
        # start block
        line = line.replace('<','')
        for_id = line.replace('>','')
        for_id = for_id.strip()
      elif for_id:
        # append to id based patterns
        if for_id in Custom_patterns:
          location_in_pattern = [x.strip() for x in line.split(',')]
          if len(location_in_pattern) == 7:
            Custom_patterns[for_id].append( location_in_pattern )
        else:
          location_in_pattern = [x.strip() for x in line.split(',')]
          if len(location_in_pattern) == 7:
            Custom_patterns[for_id] = [ location_in_pattern ]
  else:
    callerror( 'Old version of Task Control Component found in the layout. Please reload a new version of Works TaskControl from webcat.' ) 
  studyInputAndOutputConveyors()

def OnStop():
  s = ''
  taskTimes.Note = s.join(taskTimesList)
  
def OnReset():
  comp.MaterialInheritance = VC_MATERIAL_DISABLED 

def OnContinue():
  if sim.SimTime > 0.001: # error calls from onstart are not reset due to this delay
    comp.Sign = False

def CheckTask(taskList,task):
  for i in taskList:
    if task in i:
      return True
      break
  return False

def OnRun():
  global comp, task_signal, cont1, gripper, servo, in_connector, trigger, reserveme
  global othercont, outcont, taskdone, sim, starvedTime, taskcontrol, previous_failure_fixed_at
  global TransportInFlag
  reserveme = ""
  in_connector = cont1.getConnector(0)
  comp.Sign = False
  def_frame = comp.findFeature("DefaultFrame")
  comp.DefaultMatrix = def_frame.NodePositionMatrix
  # read tasks from note
  process1_list.Note = process1_list.Note.replace('\r\n','\n')
  list1 = [x for x in process1_list.Note.split("\n") if x.strip() and x.strip()[0] != "#"]
  raw_list = [x for x in list1 if x.strip() and x.strip()[0] != "#"]
  list1, id_based_tasks = parseInputNote(raw_list)# support for the conditional id based tasks
  TransportInFlag = CheckTask(list1,"TransportIn")
  previous_failure_fixed_at = 0.0
  delay(0.00001)
  while True:
    t = runTimesProp.Value
    done = comp.getProperty("Task::Done")
    done.Value = False
    while t > 0:
      delay(0.01)
      starttime = sim.SimTime
      stats.State = "Idle"
      
      for step in list1:
        delay(0)
        list1 = call_WT(step, id_based_tasks, list1)
        if not list1: # Exit task
          done.Value = True
          stats.State = "Off"
          return  

      t=t-1
      endtime = sim.SimTime
      if (endtime - starttime) < 1.0:
        callerror("Process is not spending time: %s"%comp.Name)
        delay(10.0)
    done.Value = True
    while done.Value:
      stats.State = "Off"
      delay(10.0)


def call_WT(step, id_based_tasks, list1):
  failure()
  items = [ x.strip() for x in step.split(":") ]
  try:
    task = items[0]
    data = items[1:]
  except:
    task = items[0]
    data = []
  ## TASKS
  newID = ''
  '''
  for i,d in enumerate(data):
    p = userVariables.get(d,None)
    if p and p.Type == VC_STRING:
      data[i] = p.Value'''
  if task == "WriteSignal":  WT__WriteSignal(*data)
  elif task == "If":  newID = WT__If(*data)
  elif task == "WaitSignal":  newID = WT__WaitSignal(*data)
  elif task == "Create": WT__create(*data)
  elif task == "WaitProperty":  newID = WT__waitProperty(*data)
  elif task == "WriteProperty":  WT__writeProperty(*data)
  elif task == "ChangeProductProperty":  WT__changeProductProperty(*data)
  elif task == "ChangeProductMaterial":  WT__changeProductMaterial(*data)
  elif task == "CreatePattern":   newID = WT__createpattern(*data)
  elif task == "TransportIn":   newID = WT__transportin(*data)
  elif task == "NeedPattern":   newID = WT__needpattern(*data)
  elif task == "RobotProcess":  WT__robotprocess(*data)
  elif task == "HumanProcess":  WT__humanprocess(*data)
  elif task == "MachineProcess":  WT__machineprocess(*data)
  elif task == "Merge":   WT__merge(*data)
  elif task == "Split":   WT__split(*data)
  elif task == "Delay":   WT__delay(*data)
  elif task == "ChangeID":  WT_ChangeID(*data)
  elif task == "Remove":  WT__remove(*data)
  elif task == "Feed":  WT__feed(*data)
  elif task == "TransportOut":  WT__transportout(*data)
  elif task == "Pick":  WT__pick(*data)
  elif task == "Place":   WT__place(*data)
  elif task == "PlacePattern":  WT__placepattern(*data)
  elif task == "DummyProcess":  WT__dummyprocess(*data)
  elif task == "SuspendRun":  WT__suspendRun(*data)
  elif task == "GlobalProcess":   WT__GlobalProcess(*data)
  elif task == "GlobalID":  WT__GlobalID(*data)
  elif task == "Reserve":   WT__Reserve(*data)
  elif task == "Release":   WT__Release(*data)
  elif task == "Sync":   WT__Sync(*data)
  elif task == "Need": newID = WT__need(*data)
  elif task == "NeedCustomPattern": WT__needcustompattern(*data)
  elif task == "Print": WT__print(*data)
  elif task == "Define": WT__define(*data)
  elif task == "Assign": WT__assign(*data)
  elif task == "WarmUp": list1 = list1[list1.index(step)+1:]
  elif task == "Exit": return WT__exit()
  else: callerror("Unknown task: %s"%task)
  if newID and newID in id_based_tasks:
    # conditional task calls: use <ID> <> -tags in tasks to create conditional tasks
    for step in id_based_tasks[newID]:
      if not call_WT(step, id_based_tasks, [""]): return None
  return list1

def failure():
  global previous_failure_fixed_at
  if mtbf.Value and mttr.Value:
    if sim.SimTime - previous_failure_fixed_at > mtbf.Value:
      repair_material = repair_material_prop.Value
      if repair_material:
        comp.MaterialInheritance = VC_MATERIAL_FORCE_INHERIT
        comp.NodeMaterial = repair_material
      repairtaskname = repair_task.Value.strip()
      stats.State = "Failure"
      if repairtaskname:
        # call resource to fix the machine
        task_call_data = "HumanProcess"+"&"+comp.Name+"&"+str(mttr.Value)+"&"+repairtaskname+"&"+''
        task_signal.signal(task_call_data)
        delay(0.0)
        triggerCondition(lambda: getTrigger() == taskdone and taskdone.Value == task_call_data)
      else:
        # auto healing
        delay(mttr.Value)
      comp.MaterialInheritance = VC_MATERIAL_DISABLED 
      stats.State = "Idle"
      previous_failure_fixed_at = sim.SimTime

mttr = comp.getProperty('Failure::MTTR')
mtbf = comp.getProperty('Failure::MTBF')
repair_task = comp.getProperty('Failure::RepairTask')
repair_material_prop = comp.getProperty('Failure::FailureVisualization')


##
## WT__ -functions   (WT stands for Works Task)
## 
def WT__Sync(List_Of_Comps, Message):
  global in_connector, Reserveme, sync_signal, syncOK_signal
  msg = '%s<>%s<>%s' % ( comp.Name, List_Of_Comps, Message)
  sync_signal.signal( msg )
  comp.SyncMessage = Message.strip().lower()
  startTime = sim.SimTime
  triggerCondition(lambda: getTrigger() == syncOK_signal )
  if sim.SimTime > startTime:
    printTimes( 'Sync', startTime, Message )
      

def WT__Reserve(reserved_comp_name):
  global in_connector, Reserveme
  robot = compsInWorld.get( reserved_comp_name )
  robot.reserver = reserved_comp_name

def WT__Release(released_comp_name):
  global in_connector, Reserveme
  robot = compsInWorld.get( released_comp_name )
  robot.reserver = ""
  reserveme = ""

def WT__GlobalID(all): #modify this to support all or specific id
  global in_connector, Global_ids
  for ccomp in cont1.Components:
    n = ccomp.Name
    if ccomp.getProperty("ProdID") != None:
      n = ccomp.ProdID
    ccomp.ProdID = Global_ids.get(n, ccomp.ProdID)


def WT__GlobalProcess(prodid):
  global in_connector, GlobalTimes, GlobalTimes2
  if prodid:
    process_id = prodid.strip()
  else:
    if not cont1.Components:
      callerror('not components in the process. Give prodID in the GlobalProcess-call')
    part = cont1.Components[0]
    if prodid:
      process_id = prodid
    else:
      process_id = part.Name
      if part.getProperty("ProdID"):
        process_id = part.ProdID
  if process_id in GlobalTimes.keys():
    dtime = GlobalTimes.get(process_id, None)
  else:
    for item in GlobalTimes2:
      if item[2].strip() in part.PN and item[0].strip() == process_id:
        dtime = float(item[1].strip())
        break
    else:
      dtime = None 
  if dtime != None:
    stats.State = "Busy"
    delay(dtime)
    stats.State = "Idle"
  else:
    callerror( "No process found in GlobalProcess for ProdID: " + str(process_id) )
  

def WT__create(prodnames, new_prodid):
  global in_connector, static_comps
  '''Added to support user variables as a list of products to create'''
  userVar = userVariables.get(prodnames)
  if userVar and userVar.Type == VC_STRING:
    prodnames = userVar.Value
  '''end'''
  prodname_list = [ x.strip() for x in prodnames.split(",") ]
  newparts = []
  for prodname in prodname_list:
    if new_prodid:
      prodid = new_prodid
    else:
      prodid = prodname
    ccomp = static_comps.get( prodname, None )
    if not ccomp:
      callerror("No component witn name %s found in Create" % prodname)
    else:
      posname = "MW_%s"%( prodid )
      target_frame = comp.findFeature(posname)
      if target_frame:
        target = target_frame.NodePositionMatrix
      else:
        target = comp.DefaultMatrix #default location
      newpart = app.cloneComponent(ccomp)
      delay(0.0)
      cont1.grab(newpart)
      newparts.append( newpart )
      newpart.PositionMatrix = target
      if not newpart.getProperty("ProdID"):
        prop = newpart.createProperty(VC_STRING,"ProdID")
      newpart.ProdID = prodid
      stampTAT(newpart, 'CREATED')
  return newparts

def WT__createpattern(prodname, x_amount, y_amount, z_amount, x_step, y_step, z_step, start_range, end_range):
  global in_connector
  ccomp = compsInWorld.get( prodname )
  if ccomp == None:
    callerror("No component with name %s found in CreatePattern" % vals[0])
  else:
    xa = ivaluate(x_amount)
    ya = ivaluate(y_amount)
    za = ivaluate(z_amount)
    tx = evaluate(x_step)
    ty = evaluate(y_step)
    tz = evaluate(z_step)
    from_range = ivaluate(start_range)
    amount = ivaluate(end_range)
    start = from_range-1
    if xa*ya*za<amount:
      amount = xa*ya*za
    matrix = vcMatrix.new()
    prodid = ccomp.getProperty("ProdID")
    if not prodid:
        prodid = ccomp.createProperty(VC_STRING,"ProdID")
    ccomp.ProdID = prodname
    posname = "MW_%s"%(prodid.Value)
    target_frame = comp.findFeature(posname)
    if target_frame:
      target = target_frame.NodePositionMatrix
    else:
      target = comp.DefaultMatrix #default location
    for i in range(start,amount):    
      newpart = app.cloneComponent(ccomp)
      delay(0.0)
      cont1.grab(newpart)
      delay(0.0)
      loc = vcMatrix.new(target)
      Pos = loc.P
      moveZ=0
      level = ya * xa
      moveZ = i / level
      index2 = i-level*moveZ
      ret = divmod(index2, ya)
      Pos.X += tx* ret[0]
      Pos.Y += ty* ret[1]
      Pos.Z += tz* moveZ
      loc.P = Pos
      newpart.PositionMatrix = loc
      stampTAT(newpart, 'CREATED')
    if prodid:
      ccomp.deleteProperty(prodid)

def WT__transportin(prodid, all_boolean):#["SingleProdID","Any"]
  props = comp.Properties
  loop_on = True
  while loop_on:
    stats.State = "Starved"
    path.Enabled = True
    
    triggerCondition(lambda: getTrigger() == trans and trans.Value)
    
    newpart = trans.Value
    prodids = [x.strip() for x in prodid.split('?')]
    if all_boolean == "True" or ( newpart.getProperty("ProdID") and newpart.ProdID in prodids ):
      if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True):
        pass
      else:
        cont1.grab(newpart)
      stampTAT(newpart, 'IN')
      delay(0.0)
      if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True):
        path.Enabled = True
      else:
        path.Enabled = False      
      
      if newpart.getProperty("ProdID"):
        n = newpart.ProdID
      else:
        n = newpart.Name
      posname = "MW_%s"%(n)
      target_frame = comp.findFeature(posname)
      if target_frame:
        target = target_frame.NodePositionMatrix
        newpart.PositionMatrix = target
      else:
        target = comp.DefaultMatrix #default location
      if not newpart.getProperty("ProdID"):
        prop = newpart.createProperty(VC_STRING,"ProdID")
        prop.Value = n
      delay(0.0001)
      loop_on = False
  stats.State = "Idle"
  return newpart.ProdID

def WT__need(prodnames): #needing components
  global reserveme, task_signal
  needlist = []
  id_list = [ x.strip() for x in prodnames.split(",")] #list of needed components
  for prodid in id_list:
    xyz = "0,0,0" #offsets... reserved for pattern
    task_call_data = "Need"+"&"+prodid+"&"+comp.Name+"&"+xyz
    task_signal.signal(task_call_data)
    delay(0.0)
    needlist.append(prodid)
  for n in needlist:
    stats.State = "Starved"
    current_comps1 = cont1.Components
    startTime = sim.SimTime
    triggerCondition(lambda: getTrigger() == trigger)
    printTimes( 'Need', startTime, n )
    current_comps2 = cont1.Components
    newcomp = [x for x in current_comps2 if x not in current_comps1][0]
    stampTAT(newcomp, 'IN')
    delay(0.0)
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  stats.State = "Idle"
  return newcomp.ProdID



def WT__needpattern(SingleProdID, AmountX, AmountY, AmountZ, StepX, StepY, StepZ, StartRange, EndRange):
  global reserveme, task_signal
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  l = len(cont1.Components) #current components in the container
  xa = ivaluate(AmountX)
  ya = ivaluate(AmountY)
  za = ivaluate(AmountZ)
  tx = evaluate(StepX)
  ty = evaluate(StepY)
  tz = evaluate(StepZ)
  from_range = ivaluate(StartRange)
  amount = ivaluate(EndRange)
  start = from_range-1
  if xa*ya*za<amount:
    amount = xa*ya*za
    start = 0
  for i in range(start,amount):  
    moveZ=0
    level = ya * xa
    moveZ = i / level
    index2 = i-level*moveZ
    ret = divmod(index2, ya)
    xx = str(tx* ret[0])
    yy = str(ty* ret[1])
    zz = str(tz* moveZ)
    task_call_data = "Need"+"&"+SingleProdID+"&"+comp.Name+"&"+xx+","+yy+","+zz
    task_signal.signal( task_call_data )
    delay(0.0)
    stats.State = "Starved"
    current_comps1 = cont1.Components
    triggerCondition(lambda: getTrigger() == trigger)
    current_comps2 = cont1.Components
    newcomp = [x for x in current_comps2 if x not in current_comps1][0]
    stampTAT(newcomp, 'IN')
    delay(0.0)
  stats.State = "Idle"
  return newcomp.ProdID


def WT__needcustompattern(PatternName, StartRange, EndRange):
  global reserveme, task_signal, Custom_patterns
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  pattern = Custom_patterns.get( PatternName.strip(), [] )
  if not pattern:
    callerror( 'Pattern "" not defined in the custom patterns note in Task Control Component.' )
    return
  from_range = ivaluate(StartRange)
  amount = ivaluate(EndRange)
  start = from_range-1
  if len(pattern) < amount:
    amount = len(pattern)
    #start = 0
  for i in range(start,amount):  
    id, xx, yy, zz, rz, ry, rx = pattern[i]
    task_call_data = "Need"+"&"+id+"&"+comp.Name+"&"+xx+","+yy+","+zz+","+rz+","+ry+","+rx
    task_signal.signal( task_call_data )
    delay(0.0)
    stats.State = "Starved"
    current_comps1 = cont1.Components
    triggerCondition(lambda: getTrigger() == trigger)
    current_comps2 = cont1.Components
    newcomp = [x for x in current_comps2 if x not in current_comps1][0]
    stampTAT(newcomp, 'IN')
    delay(0.0)
  stats.State = "Idle"
  return newcomp.ProdID




def WT__robotprocess(TaskName, ToolName, TCPName):
  global gripper, reserveme, task_signal
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  stats.State = "Busy"
  task_call_data = "RobotProcess"+"&"+comp.Name+"&"+TaskName+"&"+ToolName+"&"+TCPName
  task_signal.signal(task_call_data)
  delay(0.0)
  startTime = sim.SimTime
  triggerCondition(lambda: getTrigger() == taskdone and taskdone.Value == task_call_data)
  printTimes( 'RobotProcess', startTime, TaskName)
  stats.State = "Idle"

def WT__humanprocess(ProcessTime, TaskName, ToolName, State = 'Busy' ): # new
  global gripper, reserveme, task_signal
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  stats.State = State
  ptime = str(evaluate( ProcessTime ))
  task_call_data = "HumanProcess"+"&"+comp.Name+"&"+ptime+"&"+TaskName+"&"+ToolName
  task_signal.signal(task_call_data)
  delay(0.0)
  startTime = sim.SimTime
  triggerCondition(lambda: getTrigger() == taskdone and taskdone.Value == task_call_data)
  if ptime != ProcessTime: ptime = "%s=%s" % (ProcessTime,ptime)
  printTimes( 'HumanProcess', startTime, TaskName+" "+ptime)
  stats.State = "Idle"
  
def WT__machineprocess(SingleCompName, ProcessTime, mcCommand):
  global gripper, reserveme
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  stats.State = "Busy"
  machine = compsInWorld.get( SingleCompName )
  machinesignal = machine.findBehaviour("task")
  #machineCommand = machine.findBehaviour("MachineCommand")
  ptime = str(evaluate( ProcessTime ))
  task_call_data = ptime+","+comp.Name+","+mcCommand
  machinesignal.signal( task_call_data )
  #machineCommand.signal( mcCommand )
  delay(0.0)
  startTime = sim.SimTime
  triggerCondition(lambda: getTrigger() == taskdone and taskdone.Value == task_call_data )
  if ptime != ProcessTime: ptime = "%s=%s" % (ProcessTime,ptime)
  printTimes( 'MachineProcess', startTime, SingleCompName+" "+ptime)
  stats.State = "Idle"


def WT__merge(ParentProdID, ListOfProdID, All):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
    
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  #ccomps = cont1.Components
  parent = None
  childs = []
  child_ids = [x for x in ListOfProdID.split(",")]
  for c in ccomps:
    if parent == None and c.getProperty("ProdID") != None and c.ProdID == ParentProdID:
      parent = c
    elif All == "True" or (c.getProperty("ProdID") != None and c.ProdID in child_ids):
      childs.append(c)
  if parent != None:
    ccont = parent.findBehaviour("ComponentContainer")
    if not ccont:
      ccont = parent.createBehaviour(VC_COMPONENTCONTAINER,"ComponentContainer")
      delay(0.0)
    for c in childs:
      ccont.grab(c)
  else:
    callerror("No component with ProdID %s found in Merge" % ParentProdID)


def WT__split(ListOfProdID):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  prodids = [ x.strip() for x in ListOfProdID.split(",")]
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  #ccomps = cont1.Components
  traversecomps = ccomps[:]
  allcomps = []
  while traversecomps:
    c = traversecomps.pop(0)
    allcomps.append(c)
    traversecomps.extend(c.ChildComponents)
  for s in prodids:
    for c in allcomps:
      if c.getProperty("ProdID") and c.ProdID == s:
        cont1.grab(c)
        delay(0.0)   


def WT__delay(DelayTime, State = 'Break' ):
  global gripper, reserveme
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
    
  dtime = evaluate( DelayTime )
    
  stats.State = State
  startTime = sim.SimTime
  delay( max(0, dtime) )
  if ("%g"%dtime) != DelayTime: DelayTime = "%s=%g" % (DelayTime,dtime)
  printTimes( 'Delay', startTime, DelayTime )
  stats.State = "Idle"


def WT_ChangeID(SingleProdID, NewProdID):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
    
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  #print "ccomps:",ccomps
  #ccomps = cont1.Components
  for c in ccomps:
    try:
      if c.ProdID == SingleProdID:
        c.ProdID = NewProdID
    except: pass


def WT__changeProductProperty(SingleProdID, PropertyName, PropertyValue):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  propname = PropertyName.replace(';',':').strip()
  propval  = PropertyValue
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components  
  #ccomps = cont1.Components
  for c in ccomps:
    try:
      if c.ProdID == SingleProdID.strip() or not SingleProdID.strip():
        prop = c.getProperty(propname)
        if prop:
          _type_map = { VC_STRING:str, VC_INTEGER:int, VC_BOOLEAN:boolean_cast, VC_REAL:float }
          _type_func = _type_map.get( prop.Type, str )
          _Value = _type_func(propval)
          prop.Value = _Value
    except: pass


def WT__changeProductMaterial(SingleProdID, MaterialName):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  __material = app.findMaterial( MaterialName.strip() )
  if not __material:
    return
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components    
  #ccomps = cont1.Components
  for c in ccomps:
    try:
      if c.ProdID == SingleProdID.strip() or not SingleProdID.strip():
        c.MaterialInheritance = VC_MATERIAL_FORCE_INHERIT
        c.NodeMaterial = __material
    except: pass



def WT__remove(ListOfProdID, All):
  global in_connector, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  prodids = [ x.strip() for x in ListOfProdID.split(",")]
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components 
  #ccomps = cont1.Components
  for r in prodids:
    for c in ccomps:
      if All == "True":
        c.delete()
      else:
        try:
          if c.ProdID == r:
            c.delete()
        except:
          pass


def WT__feed(ListOfProdID, TaskName, ToolName, TCPName, All):
  global gripper, reserveme, task_signal
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    # path Length = 50 时 获取组件为空  Length = 200 时 获取组件不为空   测试150 获取组件不为空
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  #ccomps = cont1.Components
  complist = [x.strip() for x in ListOfProdID.split(",")]
  count = 0
  for c in ccomps:
    if (c.getProperty("ProdID") != None and c.ProdID in complist) or All == "True":
      task_call_data = "Feed"+"&"+comp.Name+"&"+c.ProdID+"&"+TaskName+"&"+ToolName+"&"+TCPName
      task_signal.signal(task_call_data)
      delay(0.0)
      count = count +1
  for i in range(count):
    # wait parts to be picked out
    stats.State = "Blocked"
    delay(0.0001)
    current_comps1 = cont1.Components
    startTime = sim.SimTime
    triggerCondition(lambda: getTrigger() == trigger)
    current_comps2 = cont1.Components
    outcomp = [x for x in current_comps1 if x not in current_comps2][0]
    stampTAT(outcomp, 'OUT')
    printTimes( 'Feed', startTime, outcomp.ProdID )
  stats.State = "Idle"


def WT__pick(SingleProdID, TaskName, ToolName, TCPName, All):
  global gripper, reserveme, task_signal
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  ccomps = cont1.Components
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  complist = [x.strip() for x in SingleProdID.split(",")]
  count = 0
  for c in ccomps:
    if (c.getProperty("ProdID") != None and c.ProdID in complist):
      task_call_data = "Pick"+"&"+comp.Name+"&"+c.ProdID+"&"+TaskName+"&"+ToolName+"&"+TCPName
      task_signal.signal(task_call_data)
      delay(0.0)
      count = count +1
      if All == "False":
        break
  for i in range(count):
    # wait the parts to be picked out
    stats.State = "Blocked"
    delay(0.0001)
    current_comps1 = cont1.Components
    startTime = sim.SimTime
    if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
      pass
    else:
      triggerCondition(lambda: getTrigger() == trigger)       
      current_comps2 = cont1.Components
      outcomp = [x for x in current_comps1 if x not in current_comps2][0]
      stampTAT(outcomp, 'OUT')
      printTimes( 'Pick', startTime, outcomp.ProdID )
  stats.State = "Idle"


def WT__placepattern(SingleProdID, AmountX, AmountY, AmountZ, StepX, StepY, StepZ, StartRange, EndRange, TaskName, ToolName, TCPName):
  global gripper, reserveme, task_signal
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  needlist = []
  xa = ivaluate(AmountX)
  ya = ivaluate(AmountY)
  za = ivaluate(AmountZ)
  tx = evaluate(StepX)
  ty = evaluate(StepY)
  tz = evaluate(StepZ)
  from_range = ivaluate(StartRange)
  amount = ivaluate(EndRange)
  start = from_range-1
  if xa*ya*za<amount:
    amount = xa*ya*za
    start = 0
  for i in range(start,amount):  
    moveZ=0
    level = ya * xa
    moveZ = i / level
    index2 = i-level*moveZ
    ret = divmod(index2, ya)
    xx = str(tx* ret[0])
    yy = str(ty* ret[1])
    zz = str(tz* moveZ)
    task_call_data = "Place"+"&"+comp.Name+"&"+SingleProdID+"&"+TaskName+"&"+ToolName+"&"+TCPName+"&"+xx+","+yy+","+zz#xx+","+yy+","+zz
    task_signal.signal(task_call_data)
    delay(0.0)
    needlist.append(SingleProdID)
  for n in needlist:
    stats.State = "Starved"
    current_comps1 = cont1.Components
    triggerCondition(lambda: getTrigger() == trigger)
    current_comps2 = cont1.Components
    newcomp = [x for x in current_comps2 if x not in current_comps1][0]
    stampTAT(newcomp, 'IN')
    delay(0.0)
  stats.State = "Idle"


def WT__place(SingleProdID, TaskName, ToolName, TCPName):
  global gripper, reserveme, task_signal
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  task_call_data = "Place"+"&"+comp.Name+"&"+SingleProdID+"&"+TaskName+"&"+ToolName+"&"+TCPName+"&"+"0,0,0"
  task_signal.signal(task_call_data)
  delay(0.0)
  stats.State = "Idle"
  stats.State = "Starved"
  current_comps1 = cont1.Components
  startTime = sim.SimTime
  triggerCondition(lambda: getTrigger() == trigger)
  printTimes( 'Place', startTime, SingleProdID )
  current_comps2 = cont1.Components
  newcomp = [x for x in current_comps2 if x not in current_comps1][0]
  stampTAT(newcomp, 'IN')
  delay(0.0)
  stats.State = "Idle"


def WT__transportout(ListOfProdID, All):
  global gripper, outcont, reserveme
  global TransportInFlag
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  if(hasattr(comp,"Advanced::PickDynamicRandEnabled") and comp.getProperty("Advanced::PickDynamicRandEnabled").Value == True) and TransportInFlag:
    ccomps = path.Components
  else:
    ccomps = cont1.Components
  #ccomps = cont1.Components
  ccomps.reverse()
  complist = [x.strip() for x in ListOfProdID.split(",")]
  if not outcont:
    callerror("Component %s not connected"%comp.Name)
  else:
    if All == "True":
      for c in ccomps:
        while outcont.Type != VC_COMPONENTFLOWPROXY and  not outcont.checkCapacity(c,0):
          stats.State = "Blocked"
          delay(1.0)
          break

        
        path.grab(c)
        stampTAT(c, 'OUT')
        delay(0.1)
        path.Enabled = True        
        while c in path.Components:
          stats.State = "Blocked"
          delay(0.1)
        path.Enabled = False
              
    else:
      for c in ccomps:
        if (c.getProperty("ProdID") != None and c.ProdID in complist):
          while outcont.Type != VC_COMPONENTFLOWPROXY and  not outcont.checkCapacity(c,0):
            stats.State = "Blocked"
            delay(1.0)
            break
          path.grab(c)
          stampTAT(c, 'OUT')
          delay(0.1)        
          path.Enabled = True
          while c in path.Components:
            stats.State = "Blocked"
            delay(0.1)
          path.Enabled = False 
  stats.State = "Idle"


def WT__dummyprocess(ProcessTime, State = "Busy"):
  global gripper, reserveme
  if reserveme != "":
    robot = compsInWorld.get( reserveme )
    robot.reserver = comp.Name
  stats.State = State
  startTime = sim.SimTime
  ptime = evaluate( ProcessTime )
  delay( max(ptime,0) )
  if ("%g"%ptime) != ProcessTime: ProcessTime = "%s=%g" % (ProcessTime,ptime)
  printTimes( 'DummyProcess', startTime, ProcessTime )
  stats.State = "Idle"
  
def WT__exit():
  printTimes( 'Exit', sim.SimTime, 0.0 )
  stats.State = "Idle"
  return None
  
def WT__suspendRun(*no_valid_args):
  suspendRun()


def WT__waitProperty(CompName, PropName, PropValue):
  propname = PropName.replace(';',':')
  propval  = PropValue
  thecomp = compsInWorld.get( CompName )
  if thecomp:
    prop = thecomp.getProperty(propname)
    if prop:
      _type_map = { VC_STRING:str, VC_INTEGER:int, VC_BOOLEAN:boolean_cast, VC_REAL:float }
      _type_func = _type_map.get( prop.Type, str )
      vals = [x.strip() for x in propval.split('?') if x.strip()]
      _Values = map( _type_func, vals )
      stats.State = "Blocked"
      while prop.Value not in _Values:
        delay(0.1)
      stats.State = "Idle"
      return str(prop.Value)
      

def WT__writeProperty(CompName, PropName, PropValue):

  propname = PropName.replace(';',':')
  propval  = PropValue
  thecomp = compsInWorld.get( CompName )
  if thecomp:
    prop = thecomp.getProperty(propname)
    if prop:
      _type_map = { VC_STRING:str, VC_INTEGER:int, VC_BOOLEAN:boolean_cast, VC_REAL:float }
      _type_func = _type_map.get( prop.Type, str )
      _Value = _type_func(propval)
      prop.Value = _Value


def WT__changeProductProp(SingleProdID, PropName, PropValue, All):
  thecomps = cont1.Components
  if not thecomps:
    print 'Warning from "%s" in task "ChangeProductProp": no part contained' % comp.Name
    return
  if All == 'False':
    thecomps = [x for x in ccomps if x.getProperty('ProdID') and x.getProperty('ProdID').Value == SingleProdID]
  for thecomp in thecomps:
    prop = thecomp.getProperty(PropName)
    if prop:
      _type_map = { VC_STRING:str, VC_INTEGER:int, VC_BOOLEAN:boolean_cast, VC_REAL:float }
      _type_func = _type_map.get( prop.Type, str )
      _Value = _type_func(PropValue)
      prop.Value = _Value


def WT__WriteSignal(ComponentName, SignalName, Value):
  _Signal = compsInWorld.get( ComponentName ).findBehaviour(SignalName)
  if _Signal:
    _type_map = {VC_STRINGSIGNAL:str, VC_INTEGERSIGNAL:int, VC_BOOLEANSIGNAL:boolean_cast, VC_REALSIGNAL:float}
    _type_func = _type_map.get( _Signal.Type )
    if not (_type_func):
      _Value = compsInWorld.get( Value )
      if not _Value: 
        callerror( 'component name provided in WriteSignal, does not exist' )
        return
    else:
      try:
        _Value = _type_func(Value)
      except:
        callerror( 'Correct value type was not provided for signal: %s of type: %s'%(SignalName,str(_Signal.Type)) )
        return
    _Signal.signal( _Value )
    delay(0)


def WT__WaitSignal(ComponentName, SignalName, Value):
  _Signal = compsInWorld.get( ComponentName ).findBehaviour(SignalName)
  if _Signal:
    _type_map = {VC_STRINGSIGNAL:str, VC_INTEGERSIGNAL:int, VC_BOOLEANSIGNAL:boolean_cast, VC_REALSIGNAL:float}
    _type_func = _type_map.get( _Signal.Type, str )
    vals = [x.strip() for x in Value.split('?') if x.strip()]
    _Values = map( _type_func, vals )
    _connections = _Signal.Connections
    this_script = comp.findBehaviour('OnRun')
    if not this_script in _connections:
      _connections.append(this_script)
      _Signal.Connections = _connections
    stats.State = "Blocked"
    triggerCondition(lambda: getTrigger() == _Signal and _Signal.Value in _Values)
    delay(0)
    stats.State = "Idle"
    return str( _Signal.Value )


def WT__print(Text, ComponentName, Time):
  s = ''
  if ComponentName == "True": s += comp.Name + " > "
  if Time == "True":          s += hours_mins_secs(sim.SimTime) + " > "
  s += Text
  print s


def WT__define(Variable, Default, Min, Max, Units ):
  varName = 'UserVariables::'+Variable
  if not comp.getProperty( varName ): 
    if Min and Max:
      prop = comp.createProperty( VC_REAL, varName, VC_PROPERTY_LIMIT )
      prop.MinValue = eval( Min )
      prop.MaxValue = eval( Max )
    else:
      prop = comp.createProperty( VC_REAL, varName )
    prop.Value = eval(Default)
    if Units:
      prop.Quantity = Units
    prop.Group = 100
      
def WT__assign(Variable, Expression):
  variables = dict( [(x, userVariables[x].Value) for x in userVariables] )
  prop = userVariables.get(Variable,None)
  if not prop:
    try:
      v = eval( Expression, variables )
      if type(v) == float:
        vcType = VC_REAL
      elif type(v) == int:
        vcType = VC_INTEGER
      elif type(v) == bool:
        vcType = VC_BOOLEAN
      else:
        vcType = VC_STRING
      prop = comp.createProperty( vcType, 'UserVariables::'+Variable )
      prop.Group = 100
      userVariables[Variable] = prop
      variables[Variable] = prop.Value
    except:
      try:
        prop = comp.createProperty( VC_STRING, 'UserVariables::'+Variable )
        prop.Group = 100
        userVariables[Variable] = prop
        variables[Variable] = prop.Value
      except:
        prop = None
  if prop:
    if prop.Type == VC_STRING:
      prop.Value = Expression
      return
    try:
      prop.Value = eval(Expression,variables)
    except Exception, error:
      callerror( 'Assign:%s:%s syntax error in expression [%s]' % (Variable,Expression,error))
  else:
    callerror( 'Assign:%s:%s Variable <%s> undefined' % (Variable,Expression,Variable))
    
def WT__If(Expression, Then, Else):
  
  variables = dict( [(x, userVariables[x].Value) for x in userVariables] )
  try:
    if eval(Expression,variables):
      return Then
    else:
      return Else
  except Exception, error:
    callerror( 'If:%s:%s:%s syntax error in expression [%s]' % (Expression,Then,Else,error))
  return ''
  
##
## SOME OTHER HELPER FUNCTIONS
##

def ivaluate( Expression ):
  return int(evaluate( Expression ) )
  
def evaluate( Expression ):
  try:
    value = float(Expression)
  except:
    prop = userVariables.get(Expression,None)
    if prop:
      value = prop.Value
    elif Expression in GlobalTimes.keys():
      value = GlobalTimes.get(Expression, 0.0)
    else:
      variables = dict( [(x, userVariables[x].Value) for x in userVariables] )
      try:
        value = eval( Expression,variables )
      except Exception, error:
        callerror( '[%s] Syntax error in expression [%s]' % (Expression,error))
        value = 0

  return value
  
def hours_mins_secs(t):
  hours, secs = divmod(t,3600)
  mins, secs = divmod(secs,60)
  return '%i : %02i : %05.2f' % (hours,mins,secs)

def boolean_cast(arg):
  arg = arg.lower()
  val_dict = {'0':False, 'false':False, '':False, '0.0':False}
  return val_dict.get(arg, True)


def callerror(stri):
  comp.Sign = True
  comp.Note = stri
  text = "Error found in component %s: "%comp.Name
  text = text+stri
  print text


def parseInputNote(raw_list):
  list1 = []
  id_based_tasks = {}
  need_ids = []
  for_id = None
  for line in raw_list:
    line = line.strip()
    if ('</' in line and '>' in line) or '<>' in line:
      # end block
      for_id = None
    elif '<' in line and '>' in line:
      # start block
      line = line.replace('<','')
      for_id = line.replace('>','')
    elif for_id:
      # append to id based tasks
      if for_id in id_based_tasks:
        id_based_tasks[for_id].append( line )
      else:
        id_based_tasks[for_id] = [ line ]
    else:
      # normal line to list1
      list1.append( line )
  return list1, id_based_tasks


do_TAT_stamps = True
def stampTAT(part, postfix):
  if do_TAT_stamps:
    parts = [part]
    parts.extend( part.ChildComponents )
    the_time = sim.SimTime
    for part in parts:
      id = part.getProperty('ProdID')
      if id:
        id = id.Value
      else:
        id = part.Name
      stamp_name = 'TAT_' + id + '_' + comp.Name + '_' + postfix
      prop = part.createProperty(VC_REAL, stamp_name)
      if prop: 
        prop.Value = the_time

def printTimes( task, startTime, taskList ):
  if doPrintTimes.Value:
    if timeFormat.Value == 'Seconds':
      simtime = '%8.2f' % startTime
    else:
      simtime = hours_mins_secs(startTime)
    #taskTimes.Note += '%s,%6.2f, %s %s, %s\n' % (simtime, sim.SimTime-startTime, task, taskList, stats.State )
    taskTimesList.append( '%s,%6.2f, %s %s, %s\n' % (simtime, sim.SimTime-startTime, task, taskList, stats.State ) )
   
def studyInputAndOutputConveyors():
  global othercont, outcont
  # study input and output conveyor connections
  try:
    sec = intFace.ConnectedToSections[0]
    fields = sec.Fields
    othercont = None
    for f in fields:
      if f.Type == VC_FLOWFIELD:
        othercont = f.Container
        break
  except:
    othercont = None
  try:
    sec = outFace.ConnectedToSections[0]
    fields = sec.Fields
    outcont = None
    for f in fields:
      if f.Type == VC_FLOWFIELD:
        outcont = f.Container
        if outcont.Type == VC_COMPONENTFLOWPROXY:
          callerror( 'Connected directly to a flowproxy. Blocking-state in statistics cant be calculated accurately in transport-out task.' )
        break
  except:
    outcont = None


def dogui():
  '''not complete - not in use'''
  callables = filter(callable, globals().values())
  funcs_in_this_script = filter(inspect.isfunction, callables)
  for f in local_funcs:
    print f.__name__, f.func_code.co_varnames  

path = comp.findBehaviour("Path__HIDE__")
trans = comp.findBehaviour("transition")
servo = comp.findBehaviour("ServoController__HIDE__")
cont1 = comp.findBehaviour("Container__HIDE__")
stats = comp.findBehaviour("Statistics")
taskdone = comp.findBehaviour("TaskDone")
process1_list = comp.findBehaviour("Task::Task")
trigger = comp.findBehaviour("trigger")
intFace = comp.findBehaviour("InInterface")
outFace = comp.findBehaviour("OutInterface")
taskTimes = comp.findBehaviour("Task::TaskTimes")
doPrintTimes = comp.getProperty("Task::PrintTaskTimes")
timeFormat = comp.getProperty("Task::TaskTimesFormat")
runTimesProp = comp.getProperty("Task::RunTaskTimes")
comp.Note = ''
