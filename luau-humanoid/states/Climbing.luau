--!strict
local Humanoid = require("../Humanoid")

local Climbing = {} :: Humanoid.State

function Climbing.calculateTorque(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the torques so far
	return self:computeBalanceTorque(dt)
end

function Climbing.calculateForce(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the current force so far
	return self:computeRunningForce(dt)
		+ self:computeBalanceForce(dt)
end

function Climbing.update(self: Humanoid.Humanoid, dt: number)
	-- Check the floor
	self:updateFloor(dt)
	self:updateLadder(dt)

	if not self.ladderResult.hasLadder then
		-- The user is not on a ladder
		return self:changeState(Enum.HumanoidStateType.Running)
	elseif self.jump then
		-- The user is requesting to jump
		return self:changeState(Enum.HumanoidStateType.Jumping)
	end
end

function Climbing.enter(self: Humanoid.Humanoid)
	-- pass :3
end

return Humanoid.bindState(Enum.HumanoidStateType.Climbing, Climbing)