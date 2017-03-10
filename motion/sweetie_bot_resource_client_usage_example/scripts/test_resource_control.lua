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
logger.set_root_category("sweetie_bot.motion")

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
depl:loadComponent("cont0", "sweetie_bot::motion::controller::ControllerTemplate")
cont0 = depl:getPeer("cont0")
setProperty(cont0, "required_resources", "strings", {})
setProperty(cont0, "optional_resources", "strings", {"head", "tail"})
resource_control.register_controller("cont0")
cont0:setPeriod(1.0)

depl:loadComponent("cont1", "sweetie_bot::motion::controller::ControllerTemplate")
cont1 = depl:getPeer("cont1")
setProperty(cont1, "required_resources", "strings", {"leg1", "leg2", "eyes"})
setProperty(cont1, "optional_resources", "strings", {"head"})
resource_control.register_controller("cont1")
cont1:setPeriod(1.0)

depl:loadComponent("cont2", "sweetie_bot::motion::controller::ControllerTemplate")
cont2 = depl:getPeer("cont2")
setProperty(cont2, "required_resources", "strings", {"leg3", "leg4", "tail"})
setProperty(cont2, "optional_resources", "strings", {"head"})
resource_control.register_controller("cont2")
cont2:setPeriod(1.0)

depl:loadComponent("cont3", "sweetie_bot::motion::controller::ControllerTemplate")
cont3 = depl:getPeer("cont3")
setProperty(cont3, "required_resources", "strings", {"leg1", "leg2", "leg3", "leg4"})
setProperty(cont3, "optional_resources", "strings", {"tail"})
resource_control.register_controller("cont3")
cont3:setPeriod(1.0)

-- ROS compatible operations support
cont1:loadService("rosservice")
cont1:provides("rosservice"):connect("rosSetOperational", "/motion/controller/cont1/set_operational", "std_srvs/SetBool")
-- use `rosservice call /motion/controller/cont1/set_operational 1` to start the controller

cont0:configure()
cont1:configure()
cont2:configure()
cont3:configure()

-- Run tests
print("\n\nEXPECTED: none is Running.")
rttlib.stat()

print("\n\nEXPECTED: cont0, cont3 is Running. cont0 owns head, cont3 owns tail.")
cont0:start()
cont1:start()
cont2:start()
cont3:start()
os.execute("sleep 3")
rttlib.stat()

print("\n\nEXPECTED: cont0, cont1, cont2 is Running. cont2 owns head, cont2 owns tail.")
cont1:start()
cont2:start()
os.execute("sleep 3")
rttlib.stat()

print("\n\nEXPECTED: cont1 is Running. cont1 owns head, cont0 owns tail.")
cont2:stop()
os.execute("sleep 3")
rttlib.stat()

print("\n\nEXPECTED: cont0 is Running. cont0 owns head, tail.")
cont1:stop()
os.execute("sleep 3")
rttlib.stat()

print("\n\nEXPECTED: none is Running.")
cont0:stop()
os.execute("sleep 3")
rttlib.stat()
