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

-- init resource_control module
require "resource_control"

resource_control.arbiter:configure()
resource_control.arbiter:start()

-- import test components
ros:import("sweetie_bot_resource_client_usage_example")

-- load controllers
depl:loadComponent("cont_actionlib", "sweetie_bot::motion::controller::ControllerActionlibTemplate")
cont_actionlib = depl:getPeer("cont_actionlib")
cont_actionlib:setPeriod(1.0)
setProperty(cont_actionlib, "required_resources", "strings", {"leg1", "leg2", "eyes"})
-- connect to ResourceArbiter
resource_control.register_controller("cont_actionlib")
-- configure actionlib
ros:import("rtt_actionlib")
cont_actionlib:loadService("actionlib")
cont_actionlib:provides("actionlib"):connect("/motion/cont_actionlib")

cont_actionlib:configure()
