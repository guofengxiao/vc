#encoding: utf-8
from vcApplication import *



def OnStart():
  cmduri = getApplicationPath() + 'areaCheck.py'
  cmd = loadCommand("AreaCheckCommand",cmduri)
  #iconuri = getApplicationPath() + 'flyby.bmp'
  name = u"Area Check"
  #icon = loadBitmap(iconuri)
  #cmd.Icon = icon  
  addMenuItem(VC_MENU_ADDONS,name.encode('gbk'), -1, cmd)