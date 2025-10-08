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
	min: vector,
	max: vector,
	
	-- state
	cframe: CFrame,
	size: vector,
	
	-- velocities
	linearVelocity: vector,
	angularVelocity: vector,
	
	-- momentum
	linearMomentum: vector,
	angularMomentum: vector,
	
	-- other state if necessary lol
	torque: vector,
	force: vector,
	
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

local function getWedgeInertia(mass: number, size: vector): vector
	-- https://freedium.cfd/https://rjallain.medium.com/the-moment-of-inertia-tensor-for-a-triangle-18482978a938
	local x = size.x
	local y = size.y
	local z = size.z

	return vector.create(
		mass * z*z,
		mass * x*x,
		mass * y*y
	) / 6
end

local function getCubeInertia(mass: number, size: vector): vector
	-- gets moment of inertia of a cube, we assume this
	-- for all of the parts in our body
	local x = size.x
	local y = size.y
	local z = size.z
	
	return vector.create(
		mass * (y*y + z*z),
		mass * (x*x + z*z),
		mass * (x*x + y*y)
	) / 12
end

local function castVector(vec: Vector3): vector
	-- casts a vector3 to a vector
	return vector.create(vec.X, vec.Y, vec.Z)
end

local function getExtentsSize(cframe: CFrame, size: vector): (vector, vector)
	local halfSize = size * 0.5
	local halfAbsSize = cframe.RightVector:Abs() * halfSize.x
		+ cframe.UpVector:Abs() * halfSize.y
		+ cframe.LookVector:Abs() * halfSize.z
	
	-- gets the position
	local position = cframe.Position
	return castVector(position - halfAbsSize),
		castVector(position + halfAbsSize)
end

local CF_ZERO = CFrame.new(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
function Body.newStatic(part: BasePart, shape: Hull.Obj): Body
	-- if we're anchored, then we have infinite mass
	-- so we cannot apply linear velocity or angular velocity
	-- at all to this
	local cframe = part.CFrame
	local size = castVector(part.Size)

	-- get the other stuff please :D
	local min, max = getExtentsSize(cframe, size)
	return {
		-- body
		part = part,
		hull = Hull.new(shape, cframe, size),
		
		-- sleep
		sleep = 0.0,
		flags = 0.0,

		-- queries
		min = min,
		max = max,

		-- state
		cframe = part.CFrame,
		size = size,

		-- velocities
		angularVelocity = castVector(part.AssemblyAngularVelocity),
		linearVelocity = castVector(part.AssemblyLinearVelocity),
		
		-- momentum
		angularMomentum = vector.zero,
		linearMomentum = vector.zero,

		-- other state
		force = vector.zero,
		torque = vector.zero,

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

function Body.newDynamic(part: BasePart, shape: Hull.Obj): Body
	-- if we're anchored, then we have infinite mass
	-- so we cannot apply linear velocity or angular velocity
	-- at all to this
	local mass = part:GetMass()
	local cframe = part.CFrame
	local size = castVector(part.Size)
	
	-- get the other stuff please :D
	local min, max = getExtentsSize(cframe, size)
	local inertia = getCubeInertia(mass, size)
	
	local inertiaMat = Mat3.fromVector(inertia)
	return {
		-- body
		part = part,
		hull = Hull.new(shape, cframe, size),

		-- sleep
		sleep = 0.0,
		flags = 0.0,
		
		-- queries
		min = min,
		max = max,
		
		-- state
		cframe = part.CFrame,
		size = size,
		
		-- velocities
		angularVelocity = castVector(part.AssemblyAngularVelocity),
		linearVelocity = castVector(part.AssemblyLinearVelocity),

		-- momentum
		angularMomentum = vector.zero,
		linearMomentum = vector.zero,
		
		-- other state
		force = vector.create(0, -mass * GRAVITY, 0),
		torque = vector.zero,
		
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

function Body.getSystemProps(parts: { BasePart }): (vector, number, Mat3)
	-- gets the system properties of a specific system, good for
	-- offsets and stuff ig lol
	local mass = 0.0
	local inertia = Mat3.identity
	local centroid = vector.zero
	
	for i, part in parts do
		-- get the center of mass
		local partMass = part:GetMass()
		centroid += castVector(part.Position) * partMass
		mass += partMass
	end
	
	centroid /= mass
	for i, part in parts do
		local partMass = part:GetMass()
		local r = castVector(part.Position) - centroid

		-- apply parallel axis theorem
		local partInertia = getCubeInertia(partMass, castVector(part.Size))
		inertia += (Mat3.identity * vector.dot(r, r) - Mat3.outerProduct(r)) * partMass
		inertia += Mat3.fromVector(partInertia)
	end
	
	return centroid, mass, inertia
end

Body.getExtentsSize = getExtentsSize

return Body