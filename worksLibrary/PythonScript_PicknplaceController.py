# v 0.1
# v 0.2 ToDo: Evere robot Pick Place Num upper limit
#       19:05 2016/7/30: 1. StopPallets = AtGreenSensor or AtRedSensor 停顿
#                        2. StopPallets = Never 
#                        3. StopPallets: AtRedSensor 第二个工件直接Move 不停止
#       20:45 2016/7/31  4. pallet 内部定义字典记录抓取释放的个数 palletCapacityRecord = {}
from vcScript import *
from vcHelpers.Robot import *
import vcMatrix


sim = getSimulation()
# 信号说明
'''
  # 六个信号 与 当前脚本绑定
    六个信号：
              ArriveComponentSignal 通过 InConvInterface  与传送带 ArriveComponentSignal  相连
              ExitComponentSignal   通过 InConvInterface  与传送带 ExitComponentSignal    相连
              
              ArrivePalletSignal    通过 OutConvInterface 与传送带 ArrivePalletSignal     相连
              ExitPalletSignal      通过 OutConvInterface 与传送带 ExitPalletSignal       相连
              
              PickPalletSignal      通过 PickPalletInterface 与对应组件(暂时没有例子)     相连
              ConvTransSignal       OnSignal 未做处理 其他处相关操作被注释掉 目前没有用上
'''
def OnSignal( signal ):
  global parts, pallets, pendingPallets, comp
  global palletPlaceCapacityFull
  if signal.Value:
    #########
    ## PARTS ##
    #########
    # # Green Sensor # #
    if signal.Name == 'ArriveComponentSignal':
      part = signal.Value
      if comp.StopParts == 'AtGreenSensor':
        part.stopMovement()
      parts.append(part)
    # # Red Sensor # #
    elif signal.Name == 'ExitComponentSignal':
      if signal.Value in parts:
        if comp.StopParts == 'AtRedSensor':
          signal.Value.stopMovement()
        else:
          parts.pop(0)
    ##########
    ## PALLETS ##
    ##########
    # # Green Sensor # #
    elif signal.Name == 'ArrivePalletSignal' and placeParts.Value == 'OnPallets':
      pallet = signal.Value
      x, y, z,  = getPatternSizes(pallet)
      if len(pallet.ChildComponents) > palletChildsOnArrive.Value:
        if patternPerProdID.Value:
          if pallet.ChildComponents:
            id = pallet.ChildComponents[palletChildsOnArrive.Value+1].ProdID
            isNotFull = patternNotFull(pallet,x,y,z,id)
          else:
            isNotFull = True
        else:
          isNotFull = patternNotFull(pallet,x,y,z)
      else:
        isNotFull = True
      if isNotFull:
        if comp.StopPallets == 'AtGreenSensor':
          
          pallet.stopMovement()
        if not pallet in pallets:
          pallets.append(pallet)
    # # Red Sensor # #
    elif signal.Name == 'ExitPalletSignal' and placeParts.Value == 'OnPallets':
      pallet = signal.Value
      if pallet in pallets:
        x, y, z,  = getPatternSizes(pallet)
        if len(pallet.ChildComponents) > palletChildsOnArrive.Value:
          if patternPerProdID:
            id = pallet.ChildComponents[palletChildsOnArrive.Value].ProdID
            isNotFull = patternNotFull(pallet,x,y,z,id)
          else:
            isNotFull = patternNotFull(pallet,x,y,z)
        else:
          isNotFull = True
        
        #if palletPlaceCapacityFull == True:
          #isNotFull = False
        if isNotFull:
          if comp.StopPallets == 'AtRedSensor':
            pallet.stopMovement()
          else:
            pass
            #pallets.remove(pallet) #不注释Never第二次不停止
        
          
    #######################
    ## PALLET FROM PALLET_INLET ##
    ######################
    elif signal.Name == 'PickPalletSignal' and placeParts.Value == 'OnPallets':
      pendingPallet = signal.Value
      pendingPallet.stopMovement()
      pendingPallets.append(pendingPallet)
      


# 判断 pallet是否装满 返回True 或者 False
def patternNotFull(pallet,x,y,z,id= None):
  global customPos
  if patternType.Value == 'XYZ':
    ret = len(pallet.Children) - palletChildsOnArrive.Value < x*y*z * cloneCount.Value
  else:
    if id:
      ret = len(pallet.Children) - palletChildsOnArrive.Value < len(customPos[id]) * cloneCount.Value
    else:
      ret = len(pallet.Children) - palletChildsOnArrive.Value < len(customPos['GeneralPattern']) * cloneCount.Value
  return ret

def getFullPatternSize(x=0,y=0,z=0, id = None):
  global customPos
  if patternType.Value == 'XYZ':
    ret = x*y*z
  else:
    if id:
      ret = len(customPos[id])
    else:
      ret = len(customPos['GeneralPattern'])
  return ret  

def contTrans(part,arrive):
  pass
  '''
  global convTransSignal
  convTransSignal.signal(True)
  '''
  
def OnRun():
  global parts, pallets, comp, customPos, pendingPallets, convTransSignal, trigConts
  global palletPlaceCapacityFull
  global palletProId
  global palletCapacityRecord
  palletPlaceCapacityFull = None
  palletProId = 0
  palletCapacityRecord = {}
  try:
    robo = getRobot('RobotInterface')
    #print robo.__doc__
  except:
    warning('No robot attached on the controller','Place robot on component:'+comp.Name)
    suspendRun()
  comp = getComponent()
  setRoboSpeeds(robo, comp)
  inConvIface = comp.findBehaviour('InConvInterface')
  outConvIface = comp.findBehaviour('OutConvInterface')
  partInSignal = comp.findBehaviour('ArriveComponentSignal')
  palletInSignal = comp.findBehaviour('ArrivePalletSignal')
  convTransSignal = comp.findBehaviour('ConvTransSignal')
  inConvs = []
  outConvs = []
  for sec in inConvIface.ConnectedToSections:
    inConvs.append(sec.Interface.Component)
  for sec in outConvIface.ConnectedToSections:
    outConvs.append(sec.Interface.Component)
  if not inConvs:
    warning('No input conveyor connected','Connect in conveyor on param tab.')
    suspendRun()
  if not outConvs:
    warning('No output conveyor connected','Connect out conveyor on param tab.')
    suspendRun()
  trigConts = []
  #for oConv in outConvs:
  #  for cont in oConv.findBehavioursByType(VC_CONTAINER):
  #    trigConts.append(cont)
  #    trigConts[-1].OnTransition = contTrans
  parts = []
  prevparts = []
  pallets = []
  pendingPallets = []
  again = False
  robo.ActiveTool = comp.getProperty('Advanced::ToolIndex').Value
  '''
  robo.Speed = 3000
  robo.AngularSpeed = 3000
  robo.Acc = 6000
  robo.AngularAcc = 6000
  robo.RecordRSL = True
  '''
  runner = 0
  if patternType.Value == 'Custom':
    patternNote = comp.findBehaviour('Pattern::CustomPattern')
    customPos = getCustomPositions(patternNote.Note)
  
  
  
  
  #
  #
  #
  #   WORK LOOP STARTS
  #
  while True:
    if not again:
      triggerCondition(lambda: True)
      again = True # Force logic to loop again, coz there's a change in environment
    else:
      again = False
      
    partid = '' 
    # ###############################################
    # GET PENDING PALLET FROM PALLET INLET
    # ###############################################
    if placeParts.Value == 'OnPallets' and pendingPallets:
      for outConv in outConvs:
        picked = False
        if not outConv.ChildComponents:
          ifaces = outConv.findBehavioursByType(VC_ONETOONEINTERFACE)
          inputIface = None
          for iface in ifaces:
            for sec in iface.Sections:
              for field in sec.Fields:
                if field.Type == VC_FLOWFIELD:
                  if field.Container.Connectors[field.Port].Type == VC_CONNECTOR_INPUT or field.Container.Connectors[field.Port].Type == VC_CONNECTOR_INPUT_OUTPUT:
                    inputIface = iface
          if not inputIface.IsConnected:
            #condition(lambda: pendingPallets)
            if comp.getProperty('PalletID Filtering').Value:
              inPallet = pendingPallets[0]
              if readPalletID.Value == 'From Pallet':
                inPalletProdIDs = inPallet.ProdID
              else:
                inPalletProdIDs = inPallet.Parent.ProdID_Filter
              prodIDS = outConv.ProdID_Filter
              prodIDS = prodIDS.replace(';',',')
              inPalletProdIDs = inPalletProdIDs.replace(';',',')
              inPalletProdIDs = inPalletProdIDs.split(',')
              ids = prodIDS.split(',')
              for inPalletProdID in inPalletProdIDs:
                if inPalletProdID in ids:
                  pendingPallets.pop(0)
                  robo.pick(inPallet)
                  robo.place(outConv)
                  picked = True
                  if not inPallet in pallets:
                    pallets.append(inPallet)
            else:
              inPallet = pendingPallets.pop(0)
              robo.pick(inPallet)
              robo.place(outConv)
              picked = True
              if not inPallet in pallets:
                pallets.append(inPallet)
        if picked == True:
          break


    targetPart = None
    targetPallet = None
    targetConv = None
    '''
    if not parts:
      idlePos = comp.getProperty('Advanced::RobotIdlePos Z').Value
      robo.jointMoveToComponent(inConvs[0], Tz = idlePos, OnFace = 'top')
    '''
    # ###################################################################
    #  SEARCH TARGETPART AND TARGETPALLET/-CONVEYOR
    # ###################################################################
    
    ############################################
    ## PARTS GOES ON PALLETS AND PRODID FILTERING IS ENABLED
    ############################################
    if placeParts.Value == 'OnPallets' and prodIDFiltering.Value:
      #condition(lambda: parts != prevparts and pallets)
      prevparts = parts[:]
      delay(0.00001)
      breakk = False
      for ipallet in pallets:
        for ipart in parts:
          if readPalletID.Value == 'From Pallet':
            ipallet.ProdID = ipallet.ProdID.replace(';',',')
            palletIDS = ipallet.ProdID.split(',')
          else:
            ipallet.Parent.ProdID_Filter = ipallet.Parent.ProdID_Filter.replace(';',',')
            palletIDS = ipallet.Parent.ProdID_Filter.split(',')
          ##
          ## SUBCASE PARTS ARRIVES ON PALLETS
          ##
          if partsOnPallet.Value:
            kids = ipart.ChildComponents
            if inversedPickOrder.Value:
              kids.reverse()
            for jpart in kids:
              if readPartID.Value == 'FromPart':
                partid = jpart.ProdID
              elif readPartID.Value == 'FromConveyor':
                partid = ipart.Parent.ProdID_Filter
              elif readPartID.Value == 'FromPallet':
                partid = ipart.ProdID
              if partid in palletIDS:
                targetPart = jpart
                targetPallet = ipallet
                fromPallet = ipart
                prevparts = ['CHANGED']
                breakk = True
                break
            if breakk:
              break
          ##
          ## SUBCASE PARTS ARRIVE AS INDIVIDUAL PARTS
          ##
          else:
            if readPartID.Value == 'FromPart':
              partids = [ipart.ProdID]
            elif readPartID.Value == 'FromConveyor':
              ipart.Parent.ProdID_Filter = ipart.Parent.ProdID_Filter.replace(';',',')
              partids = ipart.Parent.ProdID_Filter.split(',')
            elif readPartID.Value == 'FromPallet':
              partids = [ipart.ProdID]
            for partid in partids:
              if partid in palletIDS:
                prevparts = ['CHANGED']
                targetPart = ipart
                targetPallet = ipallet
                parts.remove(targetPart)
                breakk = True
                break
            if breakk:
              break
        if breakk:
          break
    ###################################################
    ## PARTS GOES DIRECTLY ON CONVEYORS AND PRODID FILTERING IS ENABLED
    ###################################################
    elif placeParts.Value == 'OnConveyor' and prodIDFiltering.Value:
      #condition(lambda: parts != prevparts)
      prevparts = parts[:]
      breakk = False
      for ipart in parts:
        for iconv in outConvs:
          iconv.ProdID_Filter = iconv.ProdID_Filter.replace(';',',')
          convIDS = iconv.ProdID_Filter.split(',')
          ##
          ## SUBCASE PARTS ARRIVES ON PALLETS
          ##
          if partsOnPallet.Value:
            kids = ipart.ChildComponents
            if inversedPickOrder.Value:
              kids.reverse()
            for jpart in kids:
              if readPartID.Value == 'FromPart':
                partid = jpart.ProdID
              elif readPartID.Value == 'FromConveyor':
                partid = ipart.Parent.ProdID_Filter
              elif readPartID.Value == 'FromPallet':
                partid = ipart.ProdID
              if partid in convIDS:
                targetPart = jpart
                fromPallet = ipart
                targetConv = iconv
                prevparts = ['CHANGED']
                breakk = True
                break
            if breakk:
              break
          ##
          ## SUBCASE PARTS ARRIVE AS INDIVIDUAL PARTS
          ##
          else:
            if readPartID.Value == 'FromPart':
              partid = ipart.ProdID
            elif readPartID.Value == 'FromConveyor':
              partid = ipart.Parent.ProdID_Filter
            if partid in convIDS:
              targetPart = ipart
              targetConv = iconv
              parts.remove(targetPart)
              prevparts = ['CHANGED']
              breakk = True
              break
        if breakk:
          break
          
          
    #########################
    ## PRODID FILTERING IS DISABLED
    #########################
    elif not prodIDFiltering.Value:
      #condition(lambda: parts != prevparts)
      prevparts = parts[:]
      ##
      ## SUBCASE PARTS ARRIVES ON PALLETS
      ##
      if partsOnPallet.Value:
        kids = None
        if parts:
          ipart = parts[0]
          fromPallet = ipart
          kids = ipart.ChildComponents
        if kids:
          prevparts = ['CHANGED']
          if inversedPickOrder.Value:
            targetPart = kids[-1]
          else:
            targetPart = kids[0]
      ##
      ## SUBCASE PARTS ARRIVE AS INDIVIDUAL PARTS
      ##
      else:
        if placeParts.Value == 'OnPallets':
          if parts and pallets:
            targetPart = parts.pop(0)
            targetPallet = pallets[0]
        else:
          if parts:
            targetPart = parts.pop(0)
          
      ## Choose simply the oldest pallet as a target pallet (FIFO)
      if placeParts.Value == 'OnPallets' and pallets:
        targetPallet = pallets[0]

    # ###################################################################
    # ###################################################################
    #  TARGETPART AND TARGETPALLET/-CONVEYOR ARE ALREADY SEARCHED
    #  LET'S TEST IF SOME OF THE FOLLOWING CASES ARE TRUE 
    # ###################################################################
    # ###################################################################
    
    caseA = placeParts.Value == 'OnPallets' and prodIDFiltering.Value and targetPart and targetPallet
    caseB = placeParts.Value == 'OnPallets' and not prodIDFiltering.Value and targetPart and targetPallet
    caseC = placeParts.Value == 'OnConveyor' and prodIDFiltering.Value and targetPart and targetConv
    caseD = placeParts.Value == 'OnConveyor' and not prodIDFiltering.Value and targetPart
    
    if caseA or caseB or caseC or caseD:
      ##
      ## Pick part
      ##
        
      # Part sizes
      partSize = targetPart.Geometry.BoundDiagonal*2.0
      partSizeX = partSize.X
      partSizeY = partSize.Y
      partSizeZ = partSize.Z
      aX,aY,aZ = getApproach('Advanced::ApproachOnPart')
      GraspContainerBehaviour = robo.Component.findBehaviour("GraspContainer")
      if GraspContainerBehaviour.ComponentCount == 0:
          #print "id(targetPallet):",id(targetPallet)
          #print "palletProId:",palletProId
          if id(targetPallet) != palletProId or palletPlaceCapacityFull != True:
              robo.jointMoveToComponent(targetPart, Tx = aX, Ty = aY, Tz = aZ+partSizeZ)
              robo.pickMovingPart(targetPart,Approach = aZ)
      again = True # Force logic to loop again, coz there's a change in environment
      # if part was on pallet try to start its pallet if it's now empty
      if partsOnPallet.Value:
        if fromPallet and len(fromPallet.ChildComponents) -  comp.getProperty('Pattern::PalletChildsOnArrive').Value <= 0:
          fromPallet.startMovement()
          if fromPallet in parts:
            parts.remove(fromPallet)
            
          
      
      ##
      ## Place part
      ## 将抓取的物件放置托盘中
      # Part placing on pallet
      if placeParts.Value == 'OnPallets':
        # move on top of conveyor if idle
        if not prodIDFiltering.Value:
          if not pallets: # Go on top of out conv to wait next pallet
            idlePos = comp.getProperty('Advanced::RobotIdlePos Z').Value
            robo.jointMoveToComponent(outConvs[0], Tz = idlePos, OnFace = 'top')
          
        #condition(lambda: pallets) # wait pallet
          
        childCount = len(targetPallet.Children) - comp.getProperty('Pattern::PalletChildsOnArrive').Value # Count pallet childs
        '''
        palletProId = id(targetPallet)
        palletPlaceCapacity = getComponent().getProperty("Pattern::PalletPlaceCapacity").Value
        if palletPlaceCapacity > 0:
            if childCount > palletPlaceCapacity - 1:
              palletPlaceCapacityFull = True
              targetPallet.startMovement()
              if targetPallet in pallets:
                pallets.remove(targetPallet)
              continue
            else:
              palletPlaceCapacityFull = None
        '''
        if patternType.Value == 'XYZ':
          x, y, z  = getPatternSizes(targetPallet)
          cloneIndex, patternIndex = divmod( childCount, getFullPatternSize(x,y,z,partid) )
        else:
          cloneIndex, patternIndex = divmod( childCount, getFullPatternSize(id = partid) )
        
        
        # Pallet sizes
        palletSize = targetPallet.Geometry.BoundDiagonal*2.0
        palletSizeX = palletSize.X
        palletSizeY = palletSize.Y
        palletSizeZ = palletSize.Z
        
        if patternType.Value == 'XYZ':
          xStep, yStep, zStep = getPatternSteps(targetPallet,targetPart)
          k, ij = divmod(patternIndex,y*x)
          i, j = divmod(ij,y)
          placeX = -( (x-1) * xStep ) /2 + i*xStep #+ xx
          placeY = -( (y-1) * yStep ) /2 + j*yStep #+ yy
          placeZ = k * (2*zStep) + palletSizeZ #+ zz
          a,b,c = 0.0, 0.0, 0.0
        else: # PatternType == 'Custom'
          x,y,z = 0,0,0
          if patternPerProdID.Value:
            if not partid:
              warning('Use "PatternPerProdID" with "ProdID_Filtering" enabled', 'Enable "ProdID_Filtering" on General-tab')
              partid = targetPart.ProdID
            placeX = customPos[partid][patternIndex][0]
            placeY = customPos[partid][patternIndex][1]
            placeZ = customPos[partid][patternIndex][2]
            a = customPos[partid][patternIndex][3]
            b = customPos[partid][patternIndex][4]
            c = customPos[partid][patternIndex][5]
          else:
            placeX = customPos['GeneralPattern'][patternIndex][0]
            placeY = customPos['GeneralPattern'][patternIndex][1]
            placeZ = customPos['GeneralPattern'][patternIndex][2]
            a = customPos['GeneralPattern'][patternIndex][3]
            b = customPos['GeneralPattern'][patternIndex][4]
            c = customPos['GeneralPattern'][patternIndex][5]
          
        clonePos = vcMatrix.new()
        clonePos.translateRel(0,0,cloneIndex * cloneStep.Value)
        if cloneRotationType.Value == 'Increasing':
          cloneRot = cloneIndex * cloneRotation.Value
        else:
          rr,ee = divmod(cloneIndex,2)
          if ee:
            cloneRot = cloneRotation.Value
          else:
            cloneRot = 0.0
        clonePos.rotateRelZ(cloneRot)
        offsetmtx = comp.getProperty('PatternPosition::Position').Value
        offsetmtx = offsetmtx * clonePos
        v = offsetmtx.getWPR()
        xx =offsetmtx.P.X
        yy = offsetmtx.P.Y
        zz = offsetmtx.P.Z
        mtx = vcMatrix.new()
        aX,aY,aZ = getApproach('Advanced::ApproachOnPallet')
        targetPallet.update()
        w = targetPallet.WorldPositionMatrix
        targetMtx = w*offsetmtx
        targetMtx.translateRel(placeX,placeY,placeZ+partSizeZ)
        targetMtx.rotateRelZ(c)
        targetMtx.rotateRelY(b)
        targetMtx.rotateRelX(a)
        #robo.jointMoveToMtx(targetMtx, Tx = aX, Ty = aY, Tz = aZ, Rx = 180 + a, Ry = b, Rz = c)
        # Move close
        robo.jointMoveToMtx(targetMtx, Tx = aX, Ty = aY, Tz = aZ, Rx = 180.0)
        targetMtx = offsetmtx#创建对象的引用 引用的改变直接影响对象本身
        #targetMtx.translateRel(placeX,placeY,placeZ+partSizeZ)
        targetMtx.translateRel(placeX,placeY,placeZ)
        targetMtx.rotateRelZ(c)
        targetMtx.rotateRelY(b)
        targetMtx.rotateRelX(a) # Now the matrix is in the exact position of the part origin in target
        exactMtx = vcMatrix.new(targetMtx)
        targetMtx.translateRel(0,0,partSizeZ) # Let's retract TCP with the part height amount
        # Move on part
        robo.linearMoveToMtx_ExternalBase(targetPallet,offsetmtx, Rx = 180.0)# 线性追踪底盘
        robo.releaseComponent(targetPallet) # 将法兰组件放置底盘中
        palletPlaceCapacity = getComponent().getProperty("Pattern::PalletPlaceCapacity").Value
        if id(targetPallet) in palletCapacityRecord:
          if palletCapacityRecord[id(targetPallet)] < palletPlaceCapacity:
            palletCapacityRecord[id(targetPallet)] += 1
          else:
            palletCapacityRecord[id(targetPallet)] = palletCapacityRecord[id(targetPallet)] - palletPlaceCapacity
            palletCapacityRecord[id(targetPallet)] += 1
        else:
          palletCapacityRecord[id(targetPallet)] = 1
       
        #exactPos(targetPart, placeX, placeY, placeZ, xx,yy,zz,v.Z+c, v.Y+b, v.X+a)
        exactPos2(targetPart,exactMtx)
        robo.linearMoveRel(Tx= -aX,Ty= -aY, Tz= -aZ)
        # 判空操作
        if comp.StopPallets:
          if patternPerProdID.Value:
            notFull = patternNotFull(targetPallet,x,y,z, partid) # PattrenType::Custom  Pattern Per ProdID 默认为False
          else:
            notFull = patternNotFull(targetPallet,x,y,z, id = None)# PattrenType::XYZ 默认执行
          if not notFull: # 托盘装满 托盘由静止开始运动
            targetPallet.startMovement()
            # print("pallet start move!")
            if targetPallet in pallets:
              pallets.remove(targetPallet)
            continue
        childCount = len(targetPallet.Children) - comp.getProperty('Pattern::PalletChildsOnArrive').Value # Count pallet childs
        palletProId = id(targetPallet)
        palletPlaceCapacity = getComponent().getProperty("Pattern::PalletPlaceCapacity").Value
        
        if palletPlaceCapacity > 0:
            #if childCount > palletPlaceCapacity - 1:
            if palletCapacityRecord[id(targetPallet)] == palletPlaceCapacity:
              palletPlaceCapacityFull = True
              targetPallet.startMovement()
              if targetPallet in pallets:
                pallets.remove(targetPallet)
              continue
            else:
              palletPlaceCapacityFull = None       
      
      
      # Part Placing On Conveyor
      elif placeParts.Value == 'OnConveyor':
        if not prodIDFiltering.Value:
          offsetmtx = comp.getProperty('PatternPosition::Position').Value
          v = offsetmtx.getWPR()
          xx = offsetmtx.P.X
          yy = offsetmtx.P.Y
          zz = offsetmtx.P.Z
          aX,aY,aZ = getApproach('Advanced::ApproachOnPallet')
          robo.place(outConvs[0], Tx = xx, Ty = yy, Tz = zz, Rx = v.X, Ry = v.Y, Rz = v.Z, Approach = aZ)
        else:
          offsetmtx = comp.getProperty('PatternPosition::Position').Value
          v = offsetmtx.getWPR()
          xx = offsetmtx.P.X
          yy = offsetmtx.P.Y
          zz = offsetmtx.P.Z
          aX,aY,aZ = getApproach('Advanced::ApproachOnPallet')
          robo.place(targetConv, Tx = xx, Ty = yy, Tz = zz, Rx = v.X, Ry = v.Y, Rz = v.Z, Approach = aZ)
          
          
          
        
      
    #robo.RecordRSL = False
    

def exactPos(part, x, y, z, xx,yy,zz, c, b, a):
  # Deprecated (exactPos2 replaced this one)
  mtx = vcMatrix.new()
  mtx.translateRel(xx,yy,zz)
  mtx.translateRel(x,y,z)
  mtx.rotateRelZ(c)
  mtx.rotateRelY(b)
  mtx.rotateRelX(a)
  part.PositionMatrix = mtx
  

def exactPos2(part, mtx):
  part.PositionMatrix = mtx


def printMatrix(mat):
  """Usefull utility function for printing matrix value"""
  for Vec in [mat.N,mat.O,mat.A,mat.P]:
    print ("%3.3g\t%3.3g\t%3.3g\t%3.3g\n"%(Vec.X,Vec.Y,Vec.Z,Vec.W))



def getApproach(apprName):
  global comp
  # Advancement on part
  aX, aY, aZ = 0.0, 0.0, 0.0
  appr = comp.getProperty(apprName).Value
  if appr:
      aX, aY, aZ = float(appr.X), float(appr.Y), float(appr.Z)
  return aX, aY, aZ
  

def getCustomPositions(note):
  customPosDict = {}
  note.replace(';',',')
  lines = note.split('\n')
  removals = []
  generalPatternFound = False
  # Study custom pattern and remove irrelevants
  for line in lines:
    if line.find('GeneralPattern') != -1:
      generalPatternFound = True
    line.lstrip()
    if len(line) < 10 or line[0] == '#':
      if line.find('<') == -1 and line.find('>') == -1:
        removals.append(line)
  for removal in removals:
    lines.remove(removal)
  
  if generalPatternFound:
    listToAppend = -1
    for line in lines:
      if line.find('<') != -1 and line.find('>') != -1:
        id = line[line.find('<')+1 : line.find('>')]
        listToAppend = id
        continue
      cells = line.split(',')
      if len(cells) < 6:
        warning('Custom pattern defined wrong','See the Custom Pattern note on parameters and make sure each line is defined correctly')
      elif listToAppend:
        if listToAppend in customPosDict:
          customPosDict[listToAppend].append( [float(cells[0]), float(cells[1]), float(cells[2]), float(cells[3]), float(cells[4]), float(cells[5]) ] ) # append x,y,z, a,b,c
        else:
          customPosDict[listToAppend] = []
          customPosDict[listToAppend].append( [float(cells[0]), float(cells[1]), float(cells[2]), float(cells[3]), float(cells[4]), float(cells[5]) ] ) # append x,y,z, a,b,c
        
  else:
    customPosDict['GeneralPattern'] = []
    for line in lines:
      cells = line.split(',')
      if len(cells) < 6:
        warning('Custom pattern defined wrong','See the Custom Pattern note on parameters and make sure each line is defined correctly')
      else:
        customPosDict['GeneralPattern'].append( [float(cells[0]), float(cells[1]), float(cells[2]), float(cells[3]), float(cells[4]), float(cells[5]) ] ) # append x,y,z, a,b,c
      
  return customPosDict

def getPatternSizes(pallet = None):
  global comp
  x, y, z = 0,0,0
  if patternDefinition.Value == 'ReadControllerParams':
    x = patternSizeX.Value # ####
    y = patternSizeY.Value # ####
    z = patternSizeZ.Value # ####
  elif patternDefinition.Value == 'ReadPalletParams':
    xParamName = patternSizeXName.Value
    yParamName = patternSizeYName.Value
    zParamName = patternSizeZName.Value
    if pallet:
      if xParamName and yParamName and zParamName:
        x = pallet.getProperty(xParamName).Value # ####
        y = pallet.getProperty(yParamName).Value # ####
        z = pallet.getProperty(zParamName).Value # ####
      else:
        warning('Cant find pattern parameters in pallet.','Please re-check pattern parameter names on Pattern - tab.')
    else:
      warning('No pallet component found.','Please make sure you receive pallet component on target conveyor.')
  elif patternDefinition.Value == 'ReadConveyorParams':
    xParamName = patternSizeXName.Value
    yParamName = patternSizeYName.Value
    zParamName = patternSizeZName.Value
    if pallet:
      palletconv = pallet.Parent
      if xParamName and yParamName and zParamName:
        x = palletconv.getProperty(xParamName).Value # ####
        y = palletconv.getProperty(yParamName).Value # ####
        z = palletconv.getProperty(zParamName).Value # ####
      else:
        warning('Cant find pattern parameters in pallet target conveyor.','Please re-check pattern parameter names on Pattern - tab.')
    else:
      warning('No pallet component found.','Please make sure you receive pallet component on target conveyor.')
  return x, y, z
  
  
  
def getPatternSteps(pallet, part):
  global comp
  xStep, yStep, zStep = 0,0,0
  if patternDefinition.Value == 'ReadControllerParams':
    if componentSpacing.Value == 'DetectPartSize':
      spaceX = partSpacingX.Value
      spaceY = partSpacingY.Value
      spaceZ = partSpacingZ.Value
      partSize = part.Geometry.BoundDiagonal*2.0
      partSizeX = partSize.X
      partSizeY = partSize.Y
      partSizeZ = partSize.Z
      xStep = partSizeX + spaceX # ####
      yStep = partSizeY + spaceY # ####
      zStep = partSizeZ + spaceZ # ####
    else:
      xStep = stepSizeX.Value
      yStep = stepSizeY.Value
      zStep = stepSizeZ.Value
  elif patternDefinition.Value == 'ReadPalletParams':
    if componentSpacing.Value == 'DetectPartSize':
      spaceX = partSpacingX.Value
      spaceY = partSpacingY.Value
      spaceZ = partSpacingZ.Value
      partSize = part.Geometry.BoundDiagonal*2.0
      partSizeX = partSize.X
      partSizeY = partSize.Y
      partSizeZ = partSize.Z
      xStep = partSizeX + spaceX # ####
      yStep = partSizeY + spaceY# ####
      zStep = partSizeZ + spaceZ# ####
    elif componentSpacing.Value == 'FixedSteps':
      stepXParamName = stepSixeXName.Value
      stepYParamName = stepSixeYName.Value
      stepZParamName = stepSixeZName.Value
      if pallet:
        if stepXParamName and stepYParamName and stepZParamName:
          try:
            xStep = pallet.getProperty(stepXParamName).Value
            yStep = pallet.getProperty(stepYParamName).Value
            zStep = pallet.getProperty(stepZParamName).Value
          except:
            warning('Defined parameter not found in pallet','One of these not found: '+stepXParamName+', '+stepYParamName+', '+stepZParamName+', '+'Check params on Pattern-tab and match them with pallet params.')
            suspendRun()
  elif patternDefinition.Value == 'ReadConveyorParams':
    if componentSpacing.Value == 'DetectPartSize':
      spaceX = partSpacingX.Value
      spaceY = partSpacingY.Value
      spaceZ = partSpacingZ.Value
      partSize = part.Geometry.BoundDiagonal*2.0
      partSizeX = partSize.X
      partSizeY = partSize.Y
      partSizeZ = partSize.Z
      xStep = partSizeX + spaceX # ####
      yStep = partSizeY + spaceY# ####
      zStep = partSizeZ + spaceZ# ####
    elif componentSpacing.Value == 'FixedSteps':
      stepXParamName = stepSixeXName.Value
      stepYParamName = stepSixeYName.Value
      stepZParamName = stepSixeZName.Value
      if pallet:
        palletconv = pallet.Parent
        if stepXParamName and stepYParamName and stepZParamName:
          try:
            xStep = palletconv.getProperty(stepXParamName).Value
            yStep = palletconv.getProperty(stepYParamName).Value
            zStep = palletconv.getProperty(stepZParamName).Value
          except:
            warning('Defined parameter not found in pallet target conveyor','One of these not found: '+stepXParamName+', '+stepYParamName+', '+stepZParamName+', '+'Check params on Pattern-tab and match them with pallet params.')
            suspendRun()
  return xStep, yStep, zStep
    


#
# Update GUI by hiding and showing different params
#
def updGUI(arg):
  if patternType.Value == 'XYZ':
    patternPerProdID.IsVisible = False
    componentSpacing.IsVisible = True
    patternDefinition.IsVisible = True
    if patternDefinition.Value == 'ReadControllerParams':
      if componentSpacing.Value == 'FixedSteps':
        stepSixeXName.IsVisible = False
        stepSixeYName.IsVisible = False
        stepSixeZName.IsVisible = False
        partSpacingX.IsVisible = False
        partSpacingY.IsVisible = False
        partSpacingZ.IsVisible = False
        stepSizeX.IsVisible = True
        stepSizeY.IsVisible = True
        stepSizeZ.IsVisible = True
        patternSizeX.IsVisible = True
        patternSizeY.IsVisible = True
        patternSizeZ.IsVisible = True
        patternSizeXName.IsVisible = False
        patternSizeYName.IsVisible = False
        patternSizeZName.IsVisible = False
      elif componentSpacing.Value == 'DetectPartSize':
        stepSixeXName.IsVisible = False
        stepSixeYName.IsVisible = False
        stepSixeZName.IsVisible = False
        partSpacingX.IsVisible = True
        partSpacingY.IsVisible = True
        partSpacingZ.IsVisible = True
        stepSizeX.IsVisible = False
        stepSizeY.IsVisible = False
        stepSizeZ.IsVisible = False
        patternSizeX.IsVisible = True
        patternSizeY.IsVisible = True
        patternSizeZ.IsVisible = True
        patternSizeXName.IsVisible = False
        patternSizeYName.IsVisible = False
        patternSizeZName.IsVisible = False
    elif patternDefinition.Value == 'ReadPalletParams' or patternDefinition.Value == 'ReadConveyorParams':
      if componentSpacing.Value == 'FixedSteps':
        stepSixeXName.IsVisible = True
        stepSixeYName.IsVisible = True
        stepSixeZName.IsVisible = True
        partSpacingX.IsVisible = False
        partSpacingY.IsVisible = False
        partSpacingZ.IsVisible = False
        stepSizeX.IsVisible = False
        stepSizeY.IsVisible = False
        stepSizeZ.IsVisible = False
        patternSizeX.IsVisible = False
        patternSizeY.IsVisible = False
        patternSizeZ.IsVisible = False
        patternSizeXName.IsVisible = True
        patternSizeYName.IsVisible = True
        patternSizeZName.IsVisible = True
      elif componentSpacing.Value == 'DetectPartSize':
        stepSixeXName.IsVisible = False
        stepSixeYName.IsVisible = False
        stepSixeZName.IsVisible = False
        partSpacingX.IsVisible = True
        partSpacingY.IsVisible = True
        partSpacingZ.IsVisible = True
        stepSizeX.IsVisible = False
        stepSizeY.IsVisible = False
        stepSizeZ.IsVisible = False
        patternSizeX.IsVisible = False
        patternSizeY.IsVisible = False
        patternSizeZ.IsVisible = False
        patternSizeXName.IsVisible = True
        patternSizeYName.IsVisible = True
        patternSizeZName.IsVisible = True
    if partsOnPallet.Value:
      inversedPickOrder.IsVisible = True
    else:
      inversedPickOrder.IsVisible = False
    noteBeh = comp.findBehaviour('Pattern::CustomPattern')
    if not noteBeh:
      noteBeh = comp.findBehaviour('Pattern::CustomPattern__HIDE__')
    noteBeh.Name = 'Pattern::CustomPattern__HIDE__'
  else:
    patternPerProdID.IsVisible = True
    noteBeh = comp.findBehaviour('Pattern::CustomPattern')
    if not noteBeh:
      noteBeh = comp.findBehaviour('Pattern::CustomPattern__HIDE__')
    noteBeh.Name = 'Pattern::CustomPattern'
    stepSixeXName.IsVisible = False
    stepSixeYName.IsVisible = False
    stepSixeZName.IsVisible = False
    partSpacingX.IsVisible = False
    partSpacingY.IsVisible = False
    partSpacingZ.IsVisible = False
    stepSizeX.IsVisible = False
    stepSizeY.IsVisible = False
    stepSizeZ.IsVisible = False
    patternSizeX.IsVisible = False
    patternSizeY.IsVisible = False
    patternSizeZ.IsVisible = False
    patternSizeXName.IsVisible = False
    patternSizeYName.IsVisible = False
    patternSizeZName.IsVisible = False
    inversedPickOrder.IsVisible = False
    componentSpacing.IsVisible = False
    patternDefinition.IsVisible = False


def updGui2(arg):
  if mount.Value == "Ceiling":
    legsShow.IsVisible = True
    legsOffset.IsVisible = True
    legsWeight.IsVisible = True
    legsAngle.IsVisible = True
  else:
    legsShow.IsVisible = False
    legsOffset.IsVisible = False
    legsWeight.IsVisible = False
    legsAngle.IsVisible = False
    

def onPPUnConnect(iface):
  palletIDFiltering.IsVisible = False

def onPPConnect(iface,retOf):
  palletIDFiltering.IsVisible = True

comp = getComponent()

stepSixeXName = comp.getProperty('Pattern::StepSize X Name')
stepSixeYName = comp.getProperty('Pattern::StepSize Y Name')
stepSixeZName = comp.getProperty('Pattern::StepSize Z Name')

partSpacingX = comp.getProperty('Pattern::PartSpacing X')
partSpacingY = comp.getProperty('Pattern::PartSpacing Y')
partSpacingZ = comp.getProperty('Pattern::PartSpacing Z')

stepSizeX = comp.getProperty('Pattern::StepSize X')
stepSizeY = comp.getProperty('Pattern::StepSize Y')
stepSizeZ = comp.getProperty('Pattern::StepSize Z')

patternSizeX = comp.getProperty('Pattern::PatternSize X')
patternSizeY = comp.getProperty('Pattern::PatternSize Y')
patternSizeZ = comp.getProperty('Pattern::PatternSize Z')

patternSizeXName = comp.getProperty('Pattern::PatternSize X Name')
patternSizeYName = comp.getProperty('Pattern::PatternSize Y Name')
patternSizeZName = comp.getProperty('Pattern::PatternSize Z Name')
  
patternDefinition = comp.getProperty('Pattern::PatternDefinition')
patternDefinition.OnChanged = updGUI
componentSpacing = comp.getProperty('Pattern::ComponentSpacing')
componentSpacing.OnChanged = updGUI
patternType = comp.getProperty('Pattern::PatternType')
patternType.OnChanged = updGUI

prodIDFiltering = comp.getProperty('ProductID Filtering')
placeParts = comp.getProperty('Advanced::PlaceParts')

inversedPickOrder = comp.getProperty('Advanced::InversedPickOrder')
partsOnPallet = comp.getProperty('Advanced::PartsEnteringOnPallet')
partsOnPallet.OnChanged = updGUI

readPartID = comp.getProperty('Advanced::ReadPartID')
readPalletID = comp.getProperty('Advanced::ReadPalletID')

cloneCount = comp.getProperty('PatternCloning::ClonePattern in Z')
cloneStep = comp.getProperty('PatternCloning::CloneStep')
cloneRotation = comp.getProperty('PatternCloning::CloneRotation')
cloneRotationType = comp.getProperty('PatternCloning::CloneRotationType')

ppIface = comp.findBehaviour('PickPalletInterface')
ppIface.OnConnect = onPPConnect
ppIface.OnUnConnect = onPPUnConnect
palletIDFiltering = comp.getProperty('PalletID Filtering')

legsShow = comp.getProperty('Legs::Offset')
legsOffset = comp.getProperty('Legs::ShowLegs')
legsWeight = comp.getProperty('Legs::StructureWeight')
legsAngle = comp.getProperty('Legs::LegAngle')
mount = comp.getProperty('Mount')
mount.OnChanged = updGui2

patternPerProdID = comp.getProperty('Pattern::Pattern Per ProdID')
palletChildsOnArrive = comp.getProperty('Pattern::PalletChildsOnArrive')

def warning(a,b = None):
  print 'WARNING from component '+comp.Name+': '+a
  if b:
    print '               -',b



def setRoboSpeeds(robo,comp):
  if roboSpeedsEnabled.Value:
    robo.Speed = roboSpeed.Value
    robo.AngularSpeed = roboAngularSpeed.Value
    robo.Acc = roboAcc.Value
    robo.AngularAcc = roboAngularAcc.Value
    robo.JointForce = roboJointForce.Value
    robo.JointSpeed = roboJointSpeed.Value
  
def updSpeedsGUI(arg):
  if roboSpeedsEnabled.Value:
    #
    roboSpeed.WritableWhenDisconnected = True
    roboAngularSpeed.WritableWhenDisconnected = True
    roboAcc.WritableWhenDisconnected = True 
    roboAngularAcc.WritableWhenDisconnected = True 
    roboJointForce.WritableWhenDisconnected = True 
    roboJointSpeed.WritableWhenDisconnected = True
    #
    roboSpeed.WritableWhenConnected = True
    roboAngularSpeed.WritableWhenConnected = True 
    roboAcc.WritableWhenConnected = True 
    roboAngularAcc.WritableWhenConnected = True 
    roboJointForce.WritableWhenConnected = True 
    roboJointSpeed.WritableWhenConnected = True
    #
    roboSpeed.WritableWhenSimulating = True
    roboAngularSpeed.WritableWhenSimulating = True 
    roboAcc.WritableWhenSimulating = True 
    roboAngularAcc.WritableWhenSimulating = True 
    roboJointForce.WritableWhenSimulating = True 
    roboJointSpeed.WritableWhenSimulating = True
  else:
    #
    roboSpeed.WritableWhenDisconnected = False
    roboAngularSpeed.WritableWhenDisconnected = False
    roboAcc.WritableWhenDisconnected = False 
    roboAngularAcc.WritableWhenDisconnected = False 
    roboJointForce.WritableWhenDisconnected = False 
    roboJointSpeed.WritableWhenDisconnected = False
    #
    roboSpeed.WritableWhenConnected = False
    roboAngularSpeed.WritableWhenConnected = False 
    roboAcc.WritableWhenConnected = False 
    roboAngularAcc.WritableWhenConnected = False 
    roboJointForce.WritableWhenConnected = False 
    roboJointSpeed.WritableWhenConnected = False
    #
    roboSpeed.WritableWhenSimulating = False
    roboAngularSpeed.WritableWhenSimulating = False 
    roboAcc.WritableWhenSimulating = False 
    roboAngularAcc.WritableWhenSimulating = False 
    roboJointForce.WritableWhenSimulating = False 
    roboJointSpeed.WritableWhenSimulating = False
    
  
roboSpeed = comp.getProperty('RobotSpeeds::CartesianSpeed')
roboAngularSpeed = comp.getProperty('RobotSpeeds::AngularSpeed')
roboAcc = comp.getProperty('RobotSpeeds::CartesianAcceleration')
roboAngularAcc = comp.getProperty('RobotSpeeds::AngularAcceleration')
roboJointForce = comp.getProperty('RobotSpeeds::JointForce')
roboJointSpeed = comp.getProperty('RobotSpeeds::JointSpeed')
roboSpeedsEnabled = comp.getProperty('RobotSpeeds::SpeedsEnabled')
roboSpeedsEnabled.OnChanged = updSpeedsGUI
