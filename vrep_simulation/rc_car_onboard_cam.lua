 function sysCall_init()        
    -- Get some handles (as usual !):
    onboard_camera = sim.getObjectHandle('OnboardCamera')    
    -- Enable an image publisher and subscriber:
    pub = simROS.advertise('/image', 'sensor_msgs/Image')
    simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)    
end
 
function sysCall_sensing()
    -- Publish the image of the vision sensor:
    local data,w,h = sim.getVisionSensorCharImage(onboard_camera)
    sim.transformImage(data,{w,h},4)
    d = {}
    d['header'] = {seq=0,stamp=simROS.getTime(), frame_id="a"}
    d['height'] = h
    d['width'] = w
    d['encoding'] = 'rgb8'
    d['is_bigendian'] = 1
    d['step'] = w*3
    d['data'] = data
    simROS.publish(pub,d)
end
 
function sysCall_cleanup()
    -- Shut down publisher and subscriber. Not really needed from a simulation script (automatic shutdown)
    simROS.shutdownPublisher(pub)
end