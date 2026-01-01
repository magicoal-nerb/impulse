--!strict

local Humanoid = require("../Humanoid")

local Freefall = {} :: Humanoid.State

function Freefall.calculateTorque(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the torques so far
	return self:computeBalanceTorque(dt)
		+ self:computeOrientTorque(dt)
end

function Freefall.calculateForce(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the current force so far
	return self:computeRunningForce(dt)
end

function Freefall.update(self: Humanoid.Humanoid, dt: number)
	if self.floorResult.part and self.floorResult.distance < self.hipHeight then
		-- change the state to running because we
		-- found a floor
		return self:changeState(Enum.HumanoidStateType.Running)
	end
end

function Freefall.enter(self: Humanoid.Humanoid)
	-- :3
end

return Humanoid.bindState(Enum.HumanoidStateType.Freefall, Freefall)