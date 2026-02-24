--!strict
local Humanoid = require("../Humanoid")

local COYOTE_HEIGHT = 0.05
local COYOTE_TIMER = 0.1

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
	-- check the floor
	self:updateLadder(dt)
	self:updateFloor(dt)
	
	if not self.floorResult.hasFloor then
		-- the user is not on the floor, increment
		-- the coyote timer
		self.coyoteTimer -= dt
		if self.coyoteTimer < 0.0 then
			-- set to freefall, coyote timer ended
			self:changeState(Enum.HumanoidStateType.Freefall)
		end
	elseif self.jump and self.floorResult.distance <= self.hipHeight + COYOTE_HEIGHT then
		-- the user is requesting to jump.
		self:changeState(Enum.HumanoidStateType.Jumping)
	elseif self.ladderResult.hasLadder then
		-- the user is on a ladder
		self:changeState(Enum.HumanoidStateType.Climbing)
	else
		self.coyoteTimer = COYOTE_TIMER
	end
end

function Running.enter(self: Humanoid.Humanoid)
	-- :3
	self.coyoteTimer = COYOTE_TIMER
end

return Humanoid.bindState(Enum.HumanoidStateType.Running, Running)