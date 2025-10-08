--!strict

local Humanoid = require("../Humanoid")

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
	local kP = 500

	-- keep on exerting an upward force until
	-- we reached the target jump power
	local accel = kP * (self.jumpPower - self.linearVelocity.Y)
		+ workspace.Gravity
	
	if accel < 0.0 then
		-- we reached the jump power
		return self:computeRunningForce(dt)
	end
	
	local cap = 1.0 * (self.jumpPower - self.linearVelocity.Y) / dt
	local jumpForce = Vector3.yAxis * math.min(accel, cap) * self.mass
	return self:computeRunningForce(dt)
		+ jumpForce
end

function Jumping.update(self: Humanoid.Humanoid, dt: number)
	if self.jumpPower - 5 <= self.linearVelocity.Y then
		-- we reached the jump power
		return self:changeState(Enum.HumanoidStateType.Freefall)
	elseif self:updateCeiling() then
		-- we hit a ceiling
		return self:changeState(Enum.HumanoidStateType.Freefall)
	end
end

return Humanoid.bindState(Enum.HumanoidStateType.Jumping, Jumping)