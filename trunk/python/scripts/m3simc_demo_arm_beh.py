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
import numpy as nu
import m3.pwr
import math
import random
import m3.humanoid
import yaml

# ######################################################  
postures={'right_arm':{}, 'left_arm':{},}
postures['right_arm']['stow']=[-26.5, 8.7, 5.0, 129.6, 80.6, -1.5, -7.6]
postures['right_arm']['reach']=[69.8, 13.1, -54.8, 53.0, 123.5, -2.3, 1.1]
postures['right_arm']['bow']=[129.9, 23.4, -43.3, 66.4, 132.7, 29.0, 18.0]
postures['right_arm']['reach_up']=[127.0, 31.3, -54.8, 69.4, 75.5, -1.7, 1.7]
postures['right_arm']['holdup']=[56.0, 36.0, -8.0, 84.0, 119.0, -36.0, 2.0]

postures['left_arm']['stow']=[-26.5, -8.7, -5.0, 129.6, -80.6, -1.5, 7.6]
postures['left_arm']['reach']=[69.8, -13.1, 54.8, 53.0, -123.5, -2.3, -1.1]
postures['left_arm']['bow']=[129.9, -23.4, 43.3, 66.4, -132.7, 29.0, -18.0]
postures['left_arm']['reach_up']=[127.0, -31.3, 54.8, 69.4, -75.5, -1.7, -1.7]
postures['left_arm']['holdup']=[56.0, -36.0, 8.0, 84.0, -119.0, -36.0, -2.0]
# ###################################################### 

class ArmBehaviors:
    def __init__(self,bot,limb,beh):
        self.bot=bot
	self.beh=beh
	self.limb=limb
	self.ndof=bot.get_num_dof(limb)
	self.des=[0.0]*self.ndof
	self.stiffness=[0.5]*self.ndof
	self.velocity=[15.0]*self.ndof
	self.velocity_traj=[25.0]*self.ndof
	self.slew_rate=[0.65]*self.ndof
	self.tooldot_avg=m3t.M3Average(20)
	self.x_last=nu.array([0.0,0.0,0.0])
	self.td=0.0
	self.hold_cnt=0
	self.hold_last=time.time()
	self.hold_pos=[0.0]*self.ndof
	self.contact=0
	self.holding=False
	self.hold_settle=0
	self.via_traj_first=True
	self.via_files={'TrajA':'kha1.via','TrajB':'icra_2011_htl.via'}
	pt=m3t.get_m3_animation_path()
	self.via_traj={}
	'''for k in self.via_files.keys():
	    fn=pt+self.via_files[k]
	    try:
		f=file(fn,'r')
		d=yaml.safe_load(f.read())
		self.via_traj[k]=d[self.limb]
	    except IOError:
		print 'Via file',k,'not present. Skipping...'
      '''
    def stop(self):
	self.bot.set_mode_off(self.limb)
    
    def step_via_traj_A(self):
	    self.bot.set_mode_splined_traj_gc(self.limb)
	    self.bot.set_stiffness(self.limb, self.stiffness)
	    self.bot.set_slew_rate_proportion(self.limb,self.slew_rate)
	    if self.via_traj_first and len(self.via_traj['TrajA']):
		    self.via_traj_first=False
		    theta_0 = self.bot.get_theta_deg(self.limb)[:]	
		    vias=[theta_0]+self.via_traj['TrajA']+[theta_0] #start and stop at current pos	
		    for v in vias:
			    self.bot.add_splined_traj_via_deg(self.limb, v,[self.velocity_traj[0]]*self.ndof)				
	    if  self.bot.is_splined_traj_complete(self.limb):
		self.via_traj_first=True
		return m3b.res_finished
	    else:
		return m3b.res_continue
	    
    def step_via_traj_B(self):
	self.bot.set_mode_splined_traj_gc(self.limb)
	self.bot.set_stiffness(self.limb, self.stiffness)
	self.bot.set_slew_rate_proportion(self.limb,self.slew_rate)
	if self.via_traj_first and len(self.via_traj['TrajB']):
		self.via_traj_first=False
		theta_0 = self.bot.get_theta_deg(self.limb)[:]	
		vias=[theta_0]+self.via_traj['TrajB']+[theta_0] #start and stop at current pos	
		for v in vias:
			self.bot.add_splined_traj_via_deg(self.limb, v,[self.velocity_traj[0]]*self.ndof)				
	if  self.bot.is_splined_traj_complete(self.limb):
	    self.via_traj_first=True
	    return m3b.res_finished
	else:
	    return m3b.res_continue
	
    def detect_contact(self):
	self.x=nu.array(self.bot.get_status(self.limb).end_pos)
	delta=self.x-self.x_last
	dp=math.sqrt(nu.dot(delta,delta))*1000.0#approx change in tool position mm/step
	self.td=self.tooldot_avg.step(dp)
	self.x_last=self.x
	if self.td<1.0:
	    self.hold_cnt=self.hold_cnt+1
	elif self.td>=1.0 and self.holding:
	    self.contact=100
	    self.hold_cnt=0
	    self.holding=False
	    print 'Contact'
	else:
	    self.hold_cnt=0
	self.contact=max(0,self.contact-1)
	if self.hold_cnt>20:
	    if not self.holding:
		print 'Holding'
	    self.holding=True
	    
	else:
	    self.holding=False
	#if self.holding:
	    #print 'Holding'
	#if self.contact:
	    #print 'Contact'
	 
    def is_contact(self):
	return self.contact>0
    
    def hold_curr(self):
	self.via_traj_first=True
	if time.time()-self.hold_last>0.5: #new entry
	    self.hold_pos=self.bot.get_theta_deg(self.limb)
	    self.hold_settle=20
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,self.hold_pos)
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	self.hold_last=time.time()
	self.hold_settle=max(0,self.hold_settle-1)
	if self.contact and self.hold_settle==0:
	    return m3b.res_finished
	else:
	    return m3b.res_continue
    
    def holdup(self):
	self.via_traj_first=True
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,postures[self.limb]['holdup'])
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	return m3b.res_continue
    
    def zero(self):
	self.via_traj_first=True
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,[0]*self.ndof)
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	return m3b.res_continue
    
    def stow(self):
	self.via_traj_first=True
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,postures[self.limb]['stow'])
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	return m3b.res_continue
    
    def reach(self):
	self.via_traj_first=True
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,postures[self.limb]['reach'])
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	return m3b.res_continue
    
    def reach_up(self):
	self.via_traj_first=True
	self.bot.set_mode_theta_gc_mj(self.limb)
	self.bot.set_theta_deg(self.limb,postures[self.limb]['reach_up'])
	self.bot.set_stiffness(self.limb,self.stiffness)
	self.bot.set_thetadot_deg(self.limb, self.velocity)
	return m3b.res_continue
    
# ######################################################   
#Glue class

class BodyBehaviors:
    def __init__(self):
	pass
    def start(self,proxy,bot,beh):

        	print 'Use left arm? [y]'
        	use_left_arm=m3t.get_yes_no('y')
        	
        	self.beh_right_arm=ArmBehaviors(bot,'right_arm',beh)
        	beh.define_resource('right_arm')
        	beh.always('right_arm','reach',priority=0,action=self.beh_right_arm.reach)
        	
        	#beh.random('right_arm','hold_up',priority=1,action=self.beh_right_arm.reach_up,chance=0.1,timeout=3.0, inhibit=5.0)
        	beh.random('right_arm','holdup',priority=1,action=self.beh_right_arm.holdup,chance=0.1,timeout=10.0, inhibit=12.0)
        	#beh.random('right_arm','reach_up',priority=1,action=self.beh_right_arm.reach_up,chance=0.1,timeout=10.0, inhibit=12.0)
        	beh.random('right_arm','stow',priority=1,action=self.beh_right_arm.stow,chance=0.1,timeout=5.0, inhibit=12.0)
        #	beh.random('right_arm','traj_b',priority=1,action=self.beh_right_arm.step_via_traj_B,chance=0.08,timeout=25.0, inhibit=10.0)
        	#beh.random('right_arm','traj_a',priority=1,action=self.beh_right_arm.step_via_traj_A,chance=0.05,timeout=25.0, inhibit=10.0)
        	
        	
        	#beh.random('right_arm','hold_curr',priority=2,action=self.beh_right_arm.hold_curr,chance=0.002,timeout=12000.0, inhibit=10.0)
        	
        	#beh.always('step','detect_contact',priority=0,action=self.beh_right_arm.detect_contact)
        	
        	if use_left_arm:
        	    self.beh_left_arm=ArmBehaviors(bot,'left_arm',beh)
        	    beh.define_resource('left_arm')
        	    #beh.always('left_arm','reach',priority=0,action=self.beh_left_arm.reach)
        	    
        	    #beh.random('right_arm','hold_up',priority=1,action=self.beh_right_arm.reach_up,chance=0.1,timeout=3.0, inhibit=5.0)
        	    beh.random('left_arm','holdup',priority=1,action=self.beh_left_arm.holdup,chance=0.1,timeout=10.0, inhibit=12.0)
        	    #beh.random('left_arm','reach_up',priority=1,action=self.beh_left_arm.reach_up,chance=0.1,timeout=10.0, inhibit=12.0)
        	    beh.random('left_arm','stow',priority=1,action=self.beh_left_arm.stow,chance=0.1,timeout=5.0, inhibit=12.0)
            #	beh.random('right_arm','traj_b',priority=1,action=self.beh_right_arm.step_via_traj_B,chance=0.08,timeout=25.0, inhibit=10.0)
        	    #beh.random('left_arm','traj_a',priority=1,action=self.beh_left_arm.step_via_traj_A,chance=0.05,timeout=25.0, inhibit=10.0)
        	    
        	    #beh.random('left_arm','hold_curr',priority=2,action=self.beh_left_arm.hold_curr,chance=0.002,timeout=12000.0, inhibit=10.0)
        	    
        	    #beh.always('step','detect_contact',priority=0,action=self.beh_left_arm.detect_contact)
        	
 
    def is_contact(self):
	return self.beh_right_arm.is_contact()
    def stop(self):
	self.beh_right_arm.stop()

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
    beh=m3b.M3BehaviorEngine(rate=.03)
    hb=BodyBehaviors()
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
    hb.stop()
    bot.set_motor_power_off()
    proxy.step()
    time.sleep(0.25)
    proxy.stop()
