#! /usr/bin/python
# -*- coding: utf-8 -*-

#Copyright  2010, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.toolbox_beh as m3b
import m3.component_factory as m3f
import m3.unit_conversion as m3u
import m3.joint_zlift as m3z
import m3.pwr as m3rt
import numpy as nu
import m3.pwr
import math
import random
import m3.humanoid
import yaml

import m3simc_demo_head
import m3simc_demo_arm_beh


class ZLiftBehaviors:
    def __init__(self,zlift):
        self.zlift=zlift#m3.humanoid.M3Humanoid('x')#
	self.pan_des=0.0
	self.pan_slew=m3t.M3Slew()
	self.tilt_des=0.0
	self.tilt_slew=m3t.M3Slew()
    def stop(self):
	self.zlift.set_mode_off()
    def zero(self):
	self.zlift.set_mode_theta_gc()
	self.zlift.set_pos_mm(650)
	self.zlift.set_stiffness(1.0)
	self.zlift.set_slew_rate_proportion(1.0)
	self.pan_des=0.0
	self.pan_slew.val=0
	self.tilt_des=0.0
	self.tilt_slew.val=0
    def bow(self):
	self.zlift.set_mode_theta_gc()
	self.zlift.set_pos_mm(550)
	self.zlift.set_stiffness(1.0)
	self.zlift.set_slew_rate_proportion(1.0)
	self.pan_des=0.0
	self.pan_slew.val=0
	self.tilt_des=0.0
	self.tilt_slew.val=0

# ###################################################### 
class HandBehaviors:
    def __init__(self,hand,beh,body):
        self.hand=hand
	self.beh=beh
	self.body=body
	self.first_step=True
	self.open_start=True
	
    def is_contact(self):
	return self.body.is_contact()
    
    def stop(self):
	self.hand.set_mode_off()
    def open(self):
	self.hand.set_mode_theta_gc()
	self.hand.set_theta_deg([90.0,0,0,0,0])
	self.hand.set_stiffness([1.0]*5)
	self.hand.set_slew_rate_proportion([1.0]*5)
	if self.open_start:
	    self.time_start=time.time()
	self.open_start=False
	if time.time()-self.time_start>5.0: #turn off after 5s, just to avoid hearing chatter
	    self.hand.set_mode_off()
    def grasp(self):
	self.open_start=True
	self.hand.set_mode_theta_gc([0])
	self.hand.set_theta_deg([90.0],[0])
	self.hand.set_stiffness([1.0]*5)
	self.hand.set_slew_rate_proportion([1.0]*5)
	self.hand.set_mode_torque_gc([1,2,3,4])
	postures_close=[150.0,250.0,250.0,250.0,250.0]
	self.hand.set_torque_mNm(postures_close,[1,2,3,4])
	
	
#Glue class
# ###################################################### 
#class Sleeper:
    #def __init__(self):
	#pass
    #def start(self,proxy,bot,beh,head,arm,hand,torso):
	#self.bot=bot
	#self.beh=beh
	#self.head=head
	#self.body=body
	#self.hand=hand
	#self.torso=torso
	#self.first=True
    #def do_sleep(self):
	#self.beh.set_priority('eyelids','sleepy',5)
	#if self.first:
	    #self.first=False
	    #self.ts=time.time()
	#if time.time()-self.ts>25.0:
	    #print 'Waking...'
	    #self.beh.restore_priority('eyelids','sleepy')

    #def stop(self):
	#pass
# ###################################################### 	
if __name__ == '__main__':
    #Main creates bot and proxy, otherwise can be nested with other behavior script
    proxy = m3p.M3RtProxy()
    proxy.start()
    bot_name=m3t.get_robot_name()
    bot=m3f.create_component(bot_name)
    proxy.publish_param(bot) 
    proxy.subscribe_status(bot)
    proxy.publish_command(bot)
    proxy.make_operational_all()
    bot.set_motor_power_on()
    
    zlift_shm_names=proxy.get_available_components('m3joint_zlift_shm')
    if len(zlift_shm_names) > 0:
      proxy.make_safe_operational(zlift_shm_names[0])

    omnibase_shm_names=proxy.get_available_components('m3omnibase_shm')
    if len(omnibase_shm_names) > 0:
      proxy.make_safe_operational(omnibase_shm_names[0])

    humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
    if len(humanoid_shm_names) > 0:
      proxy.make_safe_operational(humanoid_shm_names[0])
      
    m3ledx2xn_ec_shm_names=proxy.get_available_components('m3ledx2xn_ec_shm')
    if len(m3ledx2xn_ec_shm_names) > 0:
      proxy.make_safe_operational(m3ledx2xn_ec_shm_names[0])
    else: 
      print 'no led found'
        
    beh=m3b.M3BehaviorEngine(rate=.03)
    bb=m3simc_demo_arm_beh.BodyBehaviors()
    bb.start(proxy,bot,beh)
    
    print 'Use head? [y]'
    use_head=m3t.get_yes_no('y')
    print 'Use zlift? [y]'
    use_zlift=m3t.get_yes_no('y')
    
    print 'Use right hand? [y]'
    use_hand=m3t.get_yes_no('y')
    
    print 'Use left hand? [y]'
    use_left_hand=m3t.get_yes_no('y')
    
	
    proxy.step()
    if use_zlift:
	zlift_names=proxy.get_available_components('m3joint_zlift')
	if len(zlift_names)!=1:
	    print 'Invalid number of zlift components available'
	    proxy.stop()
	    exit()
	zl=m3z.M3JointZLift(zlift_names[0])
	proxy.subscribe_status(zl)
	proxy.publish_command(zl)
	proxy.publish_param(zl) 
	zb=ZLiftBehaviors(zl)
	beh.define_resource('zlift')
	beh.always('zlift','zero',priority=0,action=zb.zero)
	#beh.random('torso','track_head',priority=1,action=tb.track_head,chance=0.1,timeout=30.0, inhibit=8.0)
	beh.random('zlift','bow',priority=1,action=zb.bow,chance=0.005,timeout=4.0, inhibit=15.0)
	pwr_name='m3pwr_pwr033'
	comp=m3rt.M3Pwr(pwr_name)
	proxy.subscribe_status(comp)
	proxy.publish_command(comp) 
	proxy.make_operational(pwr_name[0])
	comp.set_motor_power_on()
	
	proxy.step()
	time.sleep(0.25)
	proxy.step()
	
	
	if not zl.calibrate(proxy):
	  print 'Calibration of zlift failed.  Exiting'
	  comp.set_motor_power_off()
	  proxy.stop()	  
	  exit()
	print 'Calibration finished. Hit return when done'
        raw_input()


    if use_hand:
	#Hand 
	right_hand=m3f.create_component('m3hand_mh17')
	proxy.subscribe_status(right_hand)
	proxy.publish_command(right_hand)
	proxy.publish_param(right_hand) 
	hdb=HandBehaviors(right_hand,beh,bb)
	beh.define_resource('right_hand')
	beh.always('right_hand','open',priority=0,action=hdb.open)
	#beh.always('step','contact_right',priority=0,action=beh_right_hand.step_contact)
	#beh.whenever('right_hand','grasp',priority=1.0,action=hdb.grasp,cond=hdb.is_contact,timeout=5.0,inhibit=25.0)
	beh.random('right_hand','grasp',priority=1.0,action=hdb.grasp,timeout=5.0,inhibit=25.0,chance=0.005)
    
    if use_left_hand:
	#Hand 
	left_hand=m3f.create_component('m3hand_mh18')
	proxy.subscribe_status(left_hand)
	proxy.publish_command(left_hand)
	proxy.publish_param(left_hand) 
	hdb=HandBehaviors(left_hand,beh,bb)
	beh.define_resource('left_hand')
	beh.always('left_hand','open',priority=0,action=hdb.open)
	#beh.always('step','contact_right',priority=0,action=beh_right_hand.step_contact)
	#beh.whenever('right_hand','grasp',priority=1.0,action=hdb.grasp,cond=hdb.is_contact,timeout=5.0,inhibit=25.0)
	beh.random('left_hand','grasp',priority=1.0,action=hdb.grasp,timeout=5.0,inhibit=25.0,chance=0.005)

    
    if use_head:
	hb=m3simc_demo_head.HeadBehaviors()
	hb.start(proxy,bot,beh)
	
    ts=time.time()
    proxy.step() #Initialize data
    try:
	while True:
	    proxy.step() 
	    beh.step(verbose=True)
    except (KeyboardInterrupt,EOFError):
	pass
    proxy.step()
    print 'Exiting...'
    if use_head:
	hb.stop()
    bb.stop()
    if use_zlift:
	zb.stop()
	comp.set_motor_power_off()
    if use_hand:
	hdb.stop()
    bot.set_motor_power_off()
        
    
    proxy.step()
    time.sleep(0.25)
    proxy.stop()
