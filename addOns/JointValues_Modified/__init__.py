from vcApplication import *

def OnStart():
  cmduri = getApplicationPath() + 'joints.py'
  cmd = loadCommand("joints",cmduri)
  iconuri = getApplicationPath() + 'joints.bmp'
  icon = loadBitmap(iconuri)
  cmd.Icon = icon
  createMenuItem('View Joint Values', -1, cmd)
  
