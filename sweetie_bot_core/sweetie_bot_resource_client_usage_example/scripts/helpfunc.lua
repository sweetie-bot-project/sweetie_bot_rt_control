--- Per property setter for complex types
function setProperty(peer, prop, prop_type, value) 
	var = rtt.Variable(prop_type);
	var:fromtab(value)
	peer:getProperty(prop):set(var)
end
