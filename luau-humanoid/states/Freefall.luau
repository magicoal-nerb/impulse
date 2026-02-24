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
	-- look for the floor
	self:updateFloor(dt)
	self:updateLadder(dt)
	
	if self.floorResult.hasFloor and self.linearVelocity.Y <= 0.0 then
		-- change the state to running because we
		-- found a floor
		return self:changeState(Enum.HumanoidStateType.Running)
	elseif self.ladderResult.hasLadder then
		-- the user is on a ladder
		return self:changeState(Enum.HumanoidStateType.Climbing)
	end
end

function Freefall.enter(self: Humanoid.Humanoid)
	-- :3
end

return Humanoid.bindState(Enum.HumanoidStateType.Freefall, Freefall)