-- RESOURCE CONTROL DEPLOYMENT MODULE
--

-- get Deployer
local depl = rtt.getTC():getPeer("Deployer")

-- import ros facilities
depl:import("rtt_ros") 
local ros = rtt.provides("ros")

-- import 
ros:import("sweetie_bot_resource_control")

-- Module table
resource_control = {}

-- Load arbiter
depl:loadComponent("resource_arbiter", "sweetie_bot::motion::ResourceArbiter")
resource_control.arbiter = depl:getPeer("resource_arbiter")
resource_control.arbiter:loadService("marshalling")
-- Load controlled resources list
resource_control.arbiter:provides("marshalling"):updateProperties("resource_control.cpf")

-- Connect a requester to the arbiter
function resource_control.register_controller(peer)
	local cp = rtt.Variable("ConnPolicy")

	success = depl:loadService(peer, "resource_client")
	success = success and depl:connect("resource_arbiter.out_resource_assigment", peer .. ".resource_client.in_resource_assigment", cp )
	success = success and depl:connect("resource_arbiter.in_resource_request", peer .. ".resource_client.out_resource_request", cp )
	success = success and depl:connect("resource_arbiter.in_resource_requester_status", peer .. ".resource_client.out_resource_requester_status", cp )

	success = success and depl:connectServices(peer, peer)

	return success
end

