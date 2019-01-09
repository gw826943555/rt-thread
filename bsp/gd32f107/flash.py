# -*- coding: utf-8 -*
import pylink
import os
import re

# 烧录的文件
file = os.path.dirname(os.path.realpath(__file__)) + '\\rtthread.bin'

# 自动加载Jlink驱动
def flash_load_library():
   library = pylink.Library()
   ret = library.load_default()
   if ret == False:
      print("Load Jlink Driver Failed")
   return ret

# 获取通过USB连接的J-Link列表
def GetJlinkUsbList():
   jlink_list = []
   info = pylink.JLink().connected_emulators(host=pylink.enums.JLinkHost.USB)
   for i in info:
      jlink_list += re.findall(r"\d+\.?\d*", str(i))
   return jlink_list

def flash():
   jlink = pylink.JLink()
   #连接JLINK
   jlink.open(59400743)
   #设置接口
   jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
   #连接芯片
   jlink.connect('GD32F107RC', 'auto')
   #刷写文件
   jlink.flash_file(file, 0x8000000)
   #
   jlink.restart()
   #关闭连接
   jlink.close()

library = pylink.Library()
if library.load_default() == False:
   print("Load Jlink Driver Failed")
   exit()

jlink = pylink.JLink()
#连接JLINK
jlink.open(59400743)
#设置接口
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
#连接芯片
jlink.connect('GD32F107RC', 'auto')
#刷写文件
jlink.flash_file(file, 0x8000000)
#
jlink.reset(0, False)
#关闭连接
jlink.close()

print()