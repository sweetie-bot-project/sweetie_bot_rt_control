-- RESOURCE CONTROL MODULE SWITCHING TEST
--
require "helpfunc"

-- get Deployer
local depl = rtt.getTC():getPeer("Deployer")
depl:import("rtt_ros") 
local ros = rtt.provides("ros")

-- logger subsystem
require "logger"

logger.init_loglevels_log4cpp("resource_control.log4cpp")

-- allows ROS to call start/stop via scriptiong commands
ros:import("rtt_rosdeployment")
depl:loadService("Deployer", "rosdeployment")
-- use "rosservice call /motion/Delpoyer/rosdeployment cont1.start()"

-- init resource_control module
require "resource_control"

resource_control.arbiter:configure()
resource_control.arbiter:start()

-- import test componet package
ros:import("sweetie_bot_resource_client_usage_example")

-- load controllers
depl:loadComponent("cont1", "sweetie_bot::motion::controller::ControllerTemplate")
cont1 = depl:getPeer("cont1")
setProperty(cont1, "required_resources", "strings", {"leg1", "leg2", "eyes"})
resource_control.register_controller("cont1")
cont1:setPeriod(1.0)

depl:loadComponent("cont2", "sweetie_bot::motion::controller::ControllerTemplate")
cont2 = depl:getPeer("cont2")
setProperty(cont2, "required_resources", "strings", {"leg3", "leg4", "tail"})
resource_control.register_controller("cont2")
cont2:setPeriod(1.0)

depl:loadComponent("cont3", "sweetie_bot::motion::controller::ControllerTemplate")
cont3 = depl:getPeer("cont3")
setProperty(cont3, "required_resources", "strings", {"leg1", "leg2", "leg3", "leg4", "tail"})
resource_control.register_controller("cont3")
cont3:setPeriod(1.0)

-- ROS compatible operations support
cont1:loadService("rosservice")
cont1:provides("rosservice"):connect("rosSetOperational", "/motion/controller/cont1/set_operational", "std_srvs/SetBool")
-- use `rosservice call /motion/controller/cont1/set_operational 1` to start the controller

cont1:configure()
cont2:configure()
cont3:configure()

-- Run tests
print("\n\nEXPECTED: none is Running.")
rttlib.stat()

cont1:start()
cont2:start()
cont3:start()
os.execute("sleep 3")
print("\n\nEXPECTED: cont3 is Running.")
rttlib.stat()

cont1:start()
cont2:start()
print("\n\nEXPECTED: cont1, cont2 is Running.")
os.execute("sleep 3")
rttlib.stat()

cont1:stop()
cont2:stop()
print("\n\nEXPECTED: none is Running.")
rttlib.stat()
