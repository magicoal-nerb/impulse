--!strict

local Humanoid = require("../Humanoid")

local Running = {} :: Humanoid.State

function Running.calculateTorque(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the torques so far
	return self:computeBalanceTorque(dt)
		+ self:computeOrientTorque(dt)
end

function Running.calculateForce(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the current force so far
	return self:computeRunningForce(dt)
		+ self:computeBalanceForce(dt)
end

function Running.update(self: Humanoid.Humanoid, dt: number)
	if not self.floorResult.part then
		-- change the state to falling down because
		-- there are no floors anymore
		return self:changeState(Enum.HumanoidStateType.Freefall)
	elseif self.jump then
		-- the user is requesting to jump
		return self:changeState(Enum.HumanoidStateType.Jumping)
	end
end

return Humanoid.bindState(Enum.HumanoidStateType.Running, Running)