#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2022 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import numpy as np
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa




# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>

class Traffic_light:
    def __init__(self, id, edge1,edge2):
        self.ID=id
        self.edge1=edge1
        self.edge2=edge2
    def lastcar1(self):
        return traci.edge.getLastStepVehicleNumber(self.edge1)
    def lastcar2(self):
        return traci.edge.getLastStepVehicleNumber(self.edge2)
    def lastcar(self,edge):
        return traci.edge.getLastStepVehicleNumber(edge)
    def getwait(self,edge_id):
        return traci.edge.getWaitingTime(edge_id)
    def control(self,other,phase):
       # print(traci.trafficlight.getPhaseDuration(self.ID))
        if(self.lastcar1()>self.lastcar2() and self.lastcar1()>10):
        #    print(self.getwait(self.edge2))
            if(self.getwait(self.edge2)<200):
                traci.trafficlight.setPhase(self.ID,phase[0])
        elif(self.lastcar1()<self.lastcar2() and self.lastcar2()>6):
            traci.trafficlight.setPhase(self.ID,phase[1])
            if(self.getwait(self.edge1)<200):
                traci.trafficlight.setPhase(self.ID,phase[1])
        elif(other.lastcar1()-self.lastcar1()>0 and self.lastcar2()<=5):
            traci.trafficlight.setPhase(self.ID,phase[0])
    def control2(self,other,phase,edge3):
        if(self.lastcar1()+self.lastcar(edge3)>self.lastcar2() and self.lastcar1()>15):
        #    print(self.getwait(self.edge2))
            if(self.getwait(self.edge2)<200):
                traci.trafficlight.setPhase(self.ID,phase[0])
        elif(self.lastcar1()+self.lastcar(edge3)<self.lastcar2() and self.lastcar2()>6):
            traci.trafficlight.setPhase(self.ID,phase[1])
            if(self.getwait(self.edge1)<200):
                traci.trafficlight.setPhase(self.ID,phase[1])
        elif(other.lastcar1()-self.lastcar1()-self.lastcar(edge3)>0 and self.lastcar1()<=5):
            traci.trafficlight.setPhase(self.ID,phase[0])
    def control3(self,other,phase,edge3):
       # print(traci.trafficlight.getPhaseDuration(self.ID))
        if(self.lastcar1()>self.lastcar2() and self.lastcar1()>10 and self.getwait(self.edge2)<200):
         #   print(self.getwait(self.edge2))
            traci.trafficlight.setPhase(self.ID,phase[0])
        elif(self.lastcar1()<self.lastcar2() and self.lastcar2()>6 and self.getwait(self.edge1)<200):
            traci.trafficlight.setPhase(self.ID,phase[1])
        elif(other.lastcar1()-self.lastcar1()>0 and self.lastcar2()<=5):
            traci.trafficlight.setPhase(self.ID,phase[0])
        elif(self.getwait(edge3)>100):
            if(len(phase)==3):
                traci.trafficlight.setPhase(self.ID,phase[2])
            else:
                traci.trafficlight.setPhase(self.ID,phase[1]) 
def metrics(edge,val):
    if val:
        return traci.edge.getWaitingTime(edge)
    else:
        return traci.edge.getLastStepVehicleNumber(edge)
def run():
    """execute the TraCI control loop"""
    step = 0
    t0=Traffic_light("T0","E3","E1")
    t1=Traffic_light("T1","E9","E5")
    t2=Traffic_light("T2","E11","455325004#2")
    t3=Traffic_light("T3","279868539#2","-E2")
    t4=Traffic_light("T4","E13","103732225#1")
    t5=Traffic_light("T5","E14","850274935#1")
    t6=Traffic_light("T6","E15","103732543#0")
    t7=Traffic_light("T7","E16","103732368#0")
    t8=Traffic_light("T8","E17","850275812#1")
    t9=Traffic_light("T9","E18","103732195#1")
    t10=Traffic_light("T10","1","101")
    t11=Traffic_light("T11","2","102")
    t12=Traffic_light("T12","3","103")
    t13=Traffic_light("T13","4","857603113#0")
    t14=Traffic_light("T14","6","103732462#4")
    t15=Traffic_light("T15","7","-E10")
    t16=Traffic_light("T16","8","455324931#0")
    t17=Traffic_light("T17","9","103732422#0")
    t18=Traffic_light("T18","10","851299493#0")
    A=[]
    B=[]
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
     #   metrics("279868539#2")
#        print(t1.lastcar1())
        t0.control2(t0,(2,0),"28383667#1")
        t1.control3(t0,(2,0),"-E12")
        t2.control(t1,(2,0))
        t3.control(t2,(0,2))	
        t4.control(t3,(0,2))
        t5.control(t4,(2,0))
        t6.control3(t5,(2,0),"-850275813#0")
        t7.control(t6,(0,2)) 
        t8.control(t7,(2,0)) 
        t9.control(t8,(2,0))
        t10.control(t10,(0,2))
        t11.control(t10,(0,2))
        t12.control3(t11,(2,0),"-935568346#0")
        t13.control(t12,(0,2))
        t14.control(t13,(0,2))
        t15.control3(t14,(2,0),"103732365#5")
        t16.control(t15,(0,2))
        t17.control(t16,(0,2))
        t18.control(t17,(0,2))
      #  A.append(metrics("4",True))
   #     B.append(metrics("4",False))
        if step>4000:
            break  
        step += 1
    np.savetxt('wait.csv', A, fmt="%d", delimiter=",")
    np.savetxt('numcar.csv', B, fmt="%d", delimiter=",")
    traci.close()
    sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "osm.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
