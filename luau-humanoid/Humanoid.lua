--!strict

-- Modern humanoid physics reimplementation
-- in lua

local LuauPhysics = require("../luau-physics")
local World = LuauPhysics.World

local UPDATE_HZ = 480

local VEC_Y_FLIP = Vector3.new(1, -1, 1)
local VEC_XZ = Vector3.new(1, 0, 1)

local CAST_POINTS = {
	-- center line
	Vector3.new(0, -1, 0),
	Vector3.new(0, -1, 1),
	Vector3.new(0, -1, -1),
	
	-- corners
	Vector3.new(-1, -1, -1),
	Vector3.new(-1, -1, 1),
	Vector3.new(1, -1, -1),
	Vector3.new(1, -1, 1),
}

local States = {} :: { [Enum.HumanoidStateType]: State }
local Humanoid = {}
Humanoid.__index = Humanoid

export type State = {
	-- function to calculate the torque
	calculateTorque: (self: Humanoid, dt: number) -> Vector3,
	
	-- function to calculate the force
	calculateForce: (self: Humanoid, dt: number) -> Vector3,
	
	-- function to update
	update: (self: Humanoid, dt: number) -> (),
}

export type FloorResult = {
	-- current floor, set this to nil
	-- if no floor is present
	part: BasePart?,
	
	-- average raycast floor position
	position: Vector3,
	
	-- average raycast normal
	normal: Vector3,
	
	-- average raycast distance
	distance: number,
	
	-- velocity of the floor
	linearVelocity: Vector3,
	
	-- rotational velocity of the floor
	angularVelocity: Vector3,
	
	-- origin of the floor
	origin: Vector3,
}

export type Humanoid = typeof(setmetatable({} :: {
	-- other state
	raycastParams: RaycastParams,
	floorResult: FloorResult,
	
	-- current state
	angularVelocity: Vector3,
	linearVelocity: Vector3,
	rootCoord: CFrame,
	
	-- physics state
	inertia: CFrame,
	mass: number,
	
	-- used for state
	state: Enum.HumanoidStateType,
	
	-- used for movement
	moveDirection: Vector3,
	hipHeight: number,
	walkSpeed: number,
	jumpPower: number,
	
	-- physics body
	body: LuauPhysics.Body,
	
	-- used for physics
	rootPart: BasePart,
	clock: number,
	
	-- user jump
	jump: boolean,
}, Humanoid))

function Humanoid.bindState(
	enum: Enum.HumanoidStateType,
	state: State
): State
	-- binds a state onto the humanoid
	-- so we can use it
	States[enum] = state
	return state
end

function Humanoid.new(rootPart: BasePart): Humanoid
	local angularVelocity = rootPart.AssemblyAngularVelocity
	local linearVelocity = rootPart.AssemblyLinearVelocity
	local character = rootPart.Parent :: Model
	
	-- create params incase
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = { character }
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	raycastParams.RespectCanCollide = true
	
	-- these are used for calculating state stuff
	local body = World.addDynamicBody(rootPart)
	World.markBodyActive(body, true)
	
	return setmetatable({		
		-- current state
		angularVelocity = rootPart.AssemblyAngularVelocity,
		linearVelocity = rootPart.AssemblyLinearVelocity,
		rootCoord = rootPart.CFrame,
		
		-- other state
		raycastParams = raycastParams,
		floorResult = {
			position = Vector3.zero,
			normal = Vector3.zero,
			distance = 0.0,
			
			linearVelocity = Vector3.zero,
			angularVelocity = Vector3.zero,
			origin = Vector3.zero,
		} :: FloorResult,
		
		-- physics state
		inertia = body.inertia,
		mass = body.mass,
		state = Enum.HumanoidStateType.Running,
		
		-- props
		moveDirection = Vector3.zero,
		hipHeight = 2.0,
		walkSpeed = 16.0,
		jumpPower = 50.0,
		jump = false,
		
		body = body,
		
		-- state
		rootPart = rootPart,
		clock = os.clock(),
	}, Humanoid)
end

local function angleBetween(a: Vector3, b: Vector3): number
	-- gives a signed angle from a to b
	local cos = math.clamp(a:Dot(b), -1.0, 1.0)
	
	-- angle sign is sign((a x y).b)
	return math.acos(cos) * math.sign(a.X*b.Z - a.Z*b.X)
end

function Humanoid.getDesiredVelocity(self: Humanoid): Vector3
	-- gets the desired velocity
	local desiredVelocity = self.moveDirection * self.walkSpeed

	local floorResult = self.floorResult
	local floorPart = floorResult.part
	if floorPart then
		-- add the floor velocity aswell
		desiredVelocity += floorResult.linearVelocity
			+ (floorResult.origin - self.rootCoord.Position):Cross(floorResult.angularVelocity)
	end

	return desiredVelocity
end

function Humanoid.computeBalanceTorque(self: Humanoid, dt: number)
	local kP = 2250
	local kD = 50
	
	-- get the angular offset from the y pole
	local tiltWorld = -Vector3.yAxis:Cross(self.rootCoord.UpVector)
	return self.inertia * (kP*tiltWorld - kD*self.angularVelocity)
end

function Humanoid.computeRunningForce(self: Humanoid, dt: number): Vector3
	local kP = 150
	
	local desiredVelocity = self:getDesiredVelocity()
	local floorPart = self.floorResult.part
	local maxForce = if floorPart
		then 250
		else 143
	
	-- get the desired acceleration and clamp
	local desiredAccel = kP * (desiredVelocity - self.linearVelocity)
	local horizontal = desiredAccel * VEC_XZ
	if horizontal:Dot(horizontal) > maxForce * maxForce then
		-- reached max force, clamp it please
		horizontal = horizontal.Unit * maxForce
	end
	
	local deltaForce = self.mass * horizontal
	if floorPart then
		-- apply friction
		deltaForce *= 0.9
	end
	
	return deltaForce
end

function Humanoid.computeBalanceForce(self: Humanoid, dt: number): Vector3
	local kP = 20000
	local kD = 1100
	
	-- get the current floor
	local floorResult = self.floorResult
	
	-- calculate acceleration needed to stabilize our
	-- player onto the ground
	local dV = self.linearVelocity.Y - floorResult.linearVelocity.Y
	local dX = self.hipHeight - floorResult.distance
	local desiredAccel = kP * dX - kD * dV
	if desiredAccel < -1 then
		-- let gravity cause the player to sink
		-- to the floor
		return Vector3.zero
	end
	
	-- add upwards acceleration
	local gravity = workspace.Gravity
	local deltaAccel = (desiredAccel * 0.1) + gravity	
	return Vector3.yAxis * self.mass * deltaAccel
end

function Humanoid.changeState(self: Humanoid, state: Enum.HumanoidStateType)
	-- changes the state to another one instead
	self.state = state
end

function Humanoid.computeOrientTorque(self: Humanoid, dt: number): Vector3
	local kMaxAccel = 10000
	local kTurn = 10
	local kP = 150
	
	local floorAngular = self.floorResult.angularVelocity.Y
	if self.moveDirection.Magnitude < 0.1 then
		-- no need to set angular velocity here
		local desiredAccel = kP * (floorAngular - self.angularVelocity.Y)
		desiredAccel = math.clamp(desiredAccel, -kMaxAccel, kMaxAccel)
		return self.inertia * Vector3.yAxis * desiredAccel
	end
	
	-- our desired angular velocity
	local desiredAngular = angleBetween(self.moveDirection, self.rootCoord.LookVector)
	if math.abs(desiredAngular) < 1e-3 then
		-- really small angle, we don't need to
		-- apply a correctional torque
		desiredAngular *= 0.0
	else
		-- apply turn
		desiredAngular *= kTurn
	end	
	
	local desiredAccel = kP * (desiredAngular + floorAngular - self.angularVelocity.Y)
	desiredAccel = math.clamp(desiredAccel, -kMaxAccel, kMaxAccel)
	return self.inertia * Vector3.yAxis * desiredAccel
end

function Humanoid.updateCeiling(self: Humanoid): RaycastResult?
	-- checks for a ceiling, used in the jumping code
	local halfSize = self.rootPart.Size * 0.4
	local rotation = self.rootCoord.Rotation
	local origin = self.rootCoord.Position

	-- get raycast results
	local numResults = 0
	local direction = Vector3.yAxis * (self.hipHeight + 1)
	for i, castPoint in CAST_POINTS do
		local result = workspace:Raycast(
			origin + rotation * (castPoint * halfSize * VEC_Y_FLIP),
			direction,
			self.raycastParams
		) :: RaycastResult

		if result then
			return result
		end
	end
	
	return nil
end

function Humanoid.updateFloor(self: Humanoid, dt: number)
	local halfSize = self.rootPart.Size * 0.4
	local rotation = self.rootCoord.Rotation
	local origin = self.rootCoord.Position
	
	local hysterisis = math.max(math.abs(self.linearVelocity.Y / 100), 1)
	hysterisis += self.hipHeight

	local floorResult = self.floorResult
	local floorPart: BasePart
	
	if floorResult.part then
		-- if there was a floor, add onto
		-- the hysterisis
		hysterisis += 1.5
	else
		-- if there was no floor, just add onto the hysterisis
		-- anyways
		hysterisis += 1.1
	end
	
	-- reset the floor result
	floorResult.part = nil
	floorResult.angularVelocity *= 0.0
	floorResult.linearVelocity *= 0.0
	floorResult.position *= 0.0
	floorResult.distance *= 0.0
	floorResult.origin *= 0.0
	floorResult.normal *= 0.0

	-- get raycast results
	local numResults = 0
	local direction = -Vector3.yAxis * hysterisis
	for i, castPoint in CAST_POINTS do
		local result = workspace:Raycast(
			origin + rotation * (castPoint * halfSize),
			direction,
			self.raycastParams
		) :: RaycastResult
		
		if result then
			-- add to the results table
			numResults += 1
			floorResult.position += result.Position
			floorResult.distance += result.Distance
			floorResult.normal += result.Normal
			floorPart = result.Instance :: BasePart
		end
	end
	
	if numResults ~= 0 and floorPart then
		-- if we did have results, make sure we
		-- average them in the end
		
		-- set floor result and velocity
		local invNumResults = 1.0 / numResults
		floorResult.position *= invNumResults
		floorResult.distance *= invNumResults
		floorResult.normal *= invNumResults

		floorResult.angularVelocity = floorPart.AssemblyAngularVelocity
		floorResult.linearVelocity = floorPart.AssemblyLinearVelocity
		floorResult.origin = floorPart.Position
		
		floorResult.part = floorPart :: BasePart
	end
end

local function castVector(vec: Vector3): vector
	return vector.create(vec.X, vec.Y, vec.Z)
end

local function castVector3(vec: vector): Vector3
	return Vector3.new(vec.x, vec.y, vec.z)
end

function Humanoid.update(self: Humanoid, dt: number)
	self.angularVelocity = castVector3(self.body.angularVelocity)
	self.linearVelocity = castVector3(self.body.linearVelocity)
	self.rootCoord = self.body.cframe
	
	-- get initial state
	debug.profilebegin("humanoid::solve")

	-- first, we check for any floors
	self:updateFloor(dt)

	-- second, we update the state
	States[self.state].update(self, dt)
	
	local gravity = Vector3.yAxis * workspace.Gravity * self.mass	
	local currentState = States[self.state]	
	self.body.force = castVector(currentState.calculateForce(self, dt) - gravity)
	self.body.torque = castVector(currentState.calculateTorque(self, dt))
	
	debug.profileend()
	
	-- finalize state
	self.rootPart.AssemblyAngularVelocity = self.angularVelocity
	self.rootPart.AssemblyLinearVelocity = self.linearVelocity
end

return Humanoid