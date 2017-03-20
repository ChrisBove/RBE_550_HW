#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/chrisplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    ChrisModule = RaveCreateModule(env,'ChrisModule')
    print ChrisModule.SendCommand('help')
finally:
    RaveDestroy()
