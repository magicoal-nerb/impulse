--!strict

local Humanoid = require("../Humanoid")

local JUMP_THRESHOLD = 5.0
local VEC_XZ = Vector3.new(1.0, 0.0, 1.0)

local Jumping = {} :: Humanoid.State

function Jumping.calculateTorque(self: Humanoid.Humanoid, dt: number): Vector3
	-- return the torques so far
	return self:computeBalanceTorque(dt)
		+ self:computeOrientTorque(dt)
end

function Jumping.calculateForce(self: Humanoid.Humanoid, dt: number): Vector3
	-- this is really silly, but roblox has a flag exposed
	-- about jump spring parameters, so we have to make jumps
	-- this way. because of this, lag highjumps exist
	local kP = 200
	
	local err = self.jumpPower - self.linearVelocity:Dot(self.jumpDirection)
	local cap = 1.1 * (self.jumpPower - self.linearVelocity:Dot(self.jumpDirection)) / dt
	local accel = math.min(kP * err, cap)
	
	if accel <= 0.0 then
		-- we reached the jump power
		self:changeState(Enum.HumanoidStateType.Freefall)
		return self:computeRunningForce(dt)
	end
	
	-- otherwise, exert the upward force
	-- until we reach the target jump power
	local jumpForce = self.jumpDirection * self.mass * accel
	return self:computeRunningForce(dt) + jumpForce
end

function Jumping.update(self: Humanoid.Humanoid, dt: number)	
	-- check the floor and ceiling
	local hitCeiling = self:updateCeiling(dt)
	self:updateLadder(dt)
	self:updateFloor(dt)
		
	if self.jumpPower - JUMP_THRESHOLD <= self.linearVelocity:Dot(self.jumpDirection) then
		-- We reached the jump power
		self:changeState(Enum.HumanoidStateType.Freefall)
	elseif hitCeiling then
		-- We hit a ceiling
		self:changeState(Enum.HumanoidStateType.Freefall)
	end
end

function Jumping.enter(self: Humanoid.Humanoid)
	-- :3
	if self.floorResult.hasFloor then
		-- if we were on the floor, then jump off of it
		self.jumpDirection = Vector3.yAxis
	elseif self.ladderResult.hasLadder then
		-- if we were on a ladder, then jump off of it
		local lookDirection = -(self.rootCoord.LookVector * VEC_XZ).Unit
		self.jumpDirection = (Vector3.yAxis + lookDirection).Unit
	else
		-- otherwise, we just default to the
		-- up direction
		self.jumpDirection = Vector3.yAxis
	end
end

return Humanoid.bindState(Enum.HumanoidStateType.Jumping, Jumping)