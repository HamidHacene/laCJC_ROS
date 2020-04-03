function subscriber_speed_motor_callback(msg)
	spd = msg.data
	sim.setJointTargetVelocity(Motor, spd)
	sim.addStatusbarMessage('speed_motor subscriber received : spd ='..spd)
end

function subscriber_steer_angle_callback(msg)
	-- steer must be € [-0.5, 0.5]
	-- steer <0 --> turn right
	-- steer >0 --> turn left
	steer = msg.data
	sim.setJointTargetPosition(SteerRight, steer)
	sim.setJointTargetPosition(SteerLeft, steer)	
	sim.addStatusbarMessage('cmd_steer subscriber receiver : steer ='..steer)
end

function getPose(objectName)
	-- This function get the object pose at ROS format geometry_msgs/Pose
	objectHandle = sim.getObjectHandle(objectName)
	relTo = -1
	p = sim.getObjectPosition(objectHandle,relTo)
	o = sim.getObjectQuaternion(objectHandle,relTo)
	return {
	  		position={x=p[1],y=p[2],z=p[3]},
	  		orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
		   }
end

function getTransformStamped(objHandle,name,relTo,relToName)
	-- This function retrieves the stamped transform for a specific object
	t = sim.getSystemTime()
	p = sim.getObjectPosition(objHandle,relTo)
	o = sim.getObjectQuaternion(objHandle,relTo)
	return {
	 		header={
	 				stamp=t,
	 				frame_id=relToName
	  				},
	  		child_frame_id=name,
	  		transform={
	 					translation={x=p[1],y=p[2],z=p[3]},
	 					rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
	  				  }
			}
end

function sysCall_init()
	-- The child script initialization
	objectName = "Chassis"
	objectHandle = sim.getObjectHandle(objectName)
	-- get left and right motors handles
	Motor = sim.getObjectHandle("RearAxis")
	SteerRight = sim.getObjectHandle("SteeringRight")
	SteerLeft = sim.getObjectHandle("SteeringLeft")	
	rosInterfacePresent = simROS
	-- Prepare the publishers and subscribers :
	if rosInterfacePresent then
	  publisher1 = simROS.advertise('/simulationTime', 'std_msgs/Float32')
	  publisher2 = simROS.advertise('/pose', 'geometry_msgs/Pose')
	  subscriber1 = simROS.subscribe('/vrep_speed_motor', 'std_msgs/Float32', 'subscriber_speed_motor_callback')
	  subscriber2 = simROS.subscribe('/vrep_steer_angle', 'std_msgs/Float32', 'subscriber_steer_angle_callback')
	end
end

function sysCall_actuation()
	-- Send an updated simulation time message, and send the transform of the object attached to this script:
	if rosInterfacePresent then
	  -- publish time and pose topics
	  simROS.publish(publisher1, {data=sim.getSimulationTime()})
	  simROS.publish(publisher2, getPose("Chassis"))
	  -- send a TF
	  simROS.sendTransform(getTransformStamped(objectHandle, objectName, -1, 'world'))
	  -- To send several transforms at once, use simROS.sendTransforms instead
	end
end

function sysCall_cleanup()
	-- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
	if rosInterfacePresent then
		simROS.shutdownPublisher(publisher1)
		simROS.shutdownPublisher(publisher2)
		simROS.shutdownSubscriber(subscriber1)
		simROS.shutdownSubscriber(subscriber2)
	end
end