--!strict
local GRAVITY = 196.2

local Hull = require("./narrowphase/Hull")
local Mat3 = require("./Mat3")

local Body = {}
Body.__index = Body

export type Hull = Hull.Hull
export type Mat3 = Mat3.Mat3

export type Body = {
	-- body
	part: BasePart,
	hull: Hull,
	
	-- sleep
	sleep: number,
	flags: number,
	
	-- queries
	smooth: boolean,
	min: Vector3,
	max: Vector3,
	
	-- state
	cframe: CFrame,
	size: Vector3,
	
	-- velocities
	linearVelocity: Vector3,
	angularVelocity: Vector3,
	
	-- momentum
	linearMomentum: Vector3,
	angularMomentum: Vector3,
	
	-- other state if necessary lol
	torque: Vector3,
	force: Vector3,
	
	-- inertia
	invInertia: CFrame,
	inertia: CFrame,
	
	-- mass
	invMass: number,
	mass: number,
	
	-- surface props
	restitution: number,
	friction: number,
	beta: number,
}

local function getWedgeInertia(mass: number, size: Vector3): Vector3
	-- https://freedium.cfd/https://rjallain.medium.com/the-moment-of-inertia-tensor-for-a-triangle-18482978a938
	local x = size.X
	local y = size.Y
	local z = size.Z

	return Vector3.new(
		mass * z*z,
		mass * x*x,
		mass * y*y
	) / 6
end

local function getCubeInertia(mass: number, size: Vector3): Vector3
	-- gets moment of inertia of a cube, we assume this
	-- for all of the parts in our body
	local x = size.X
	local y = size.Y
	local z = size.Z
	
	return Vector3.new(
		mass * (y*y + z*z),
		mass * (x*x + z*z),
		mass * (x*x + y*y)
	) / 12
end

local function getSphereInertia(mass: number, radius: number): Vector3
	return Vector3.one * 0.4 * mass * radius * radius
end

local function getExtentsSize(cframe: CFrame, size: Vector3): (Vector3, Vector3)
	local halfSize = size * 0.5
	local halfAbsSize = cframe.RightVector:Abs() * halfSize.X
		+ cframe.UpVector:Abs() * halfSize.Y
		+ cframe.LookVector:Abs() * halfSize.Z
	
	-- gets the position
	local position = cframe.Position
	return position - halfAbsSize,
		position + halfAbsSize
end

local CF_ZERO = CFrame.new(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
function Body.newStatic(part: Part, hull: Hull.Hull): Body
	-- if we're anchored, then we have infinite mass
	-- so we cannot apply linear velocity or angular velocity
	-- at all to this
	local cframe = part.CFrame
	local size = part.Size

	-- get the other stuff please :D
	local min, max = getExtentsSize(cframe, size)
	return {
		-- body
		part = part,
		hull = hull,
		
		-- sleep
		sleep = 0.0,
		flags = 0.0,

		-- queries
		smooth = part.Shape ~= Enum.PartType.Block,
		min = min,
		max = max,

		-- state
		cframe = part.CFrame,
		size = size,

		-- velocities
		angularVelocity = part.AssemblyAngularVelocity,
		linearVelocity = part.AssemblyLinearVelocity,
		
		-- momentum
		angularMomentum = Vector3.zero,
		linearMomentum = Vector3.zero,

		-- other state
		force = Vector3.zero,
		torque = Vector3.zero,

		-- inertia
		invInertia = CF_ZERO,
		inertia = CF_ZERO,

		-- mass
		invMass = 0.0,
		mass = math.huge,

		-- surface props
		restitution = 0.45,
		friction = 0.4,
		beta = 0.32,
	}
end

function Body.newDynamic(part: Part, hull: Hull.Hull): Body
	-- if we're anchored, then we have infinite mass
	-- so we cannot apply linear velocity or angular velocity
	-- at all to this
	local mass = part:GetMass()
	local cframe = part.CFrame
	local size = part.Size
	
	-- get the other stuff please :D
	local min, max = getExtentsSize(cframe, size)
	local inertia = getCubeInertia(mass, size)
	
	local inertiaMat = Mat3.fromVector(inertia)
	return {
		-- body
		part = part,
		hull = hull,

		-- sleep
		sleep = 0.0,
		flags = 0.0,
		
		-- queries
		smooth = part.Shape ~= Enum.PartType.Block,
		min = min,
		max = max,
		
		-- state
		cframe = part.CFrame,
		size = size,
		
		-- velocities
		angularVelocity = part.AssemblyAngularVelocity,
		linearVelocity = part.AssemblyLinearVelocity,

		-- momentum
		angularMomentum = Vector3.zero,
		linearMomentum = Vector3.zero,
		
		-- other state
		force = Vector3.new(0, -mass * GRAVITY, 0),
		torque = Vector3.zero,
		
		-- inertia
		invInertia = Mat3.toCFrame(Mat3.inverse(inertiaMat)),
		inertia = Mat3.toCFrame(inertiaMat),
		
		-- mass
		invMass = 1.0 / mass,
		mass = mass,
		
		-- surface props
		restitution = 0.45,
		friction = 0.4,
		beta = 0.32,
	}
end

function Body.getSystemProps(parts: { BasePart }): (Vector3, number, Mat3)
	-- gets the system properties of a specific system, good for
	-- offsets and stuff ig lol
	local mass = 0.0
	local inertia = Mat3.identity
	local centroid = Vector3.zero
	
	for i, part in parts do
		-- get the center of mass
		local partMass = part:GetMass()
		centroid += part.Position * partMass
		mass += partMass
	end
	
	centroid /= mass
	for i, part in parts do
		local partMass = part:GetMass()
		local r = part.Position - centroid

		-- apply parallel axis theorem
		local partInertia = getCubeInertia(partMass, part.Size)
		inertia += (Mat3.identity * r:Dot(r) - Mat3.outerProduct(r)) * partMass
		inertia += Mat3.fromVector(partInertia)
	end
	
	return centroid, mass, inertia
end

Body.getExtentsSize = getExtentsSize

return Body