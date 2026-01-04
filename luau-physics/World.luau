--!strict

local Arbiter = require("./narrowphase/Arbiter")
local Hull = require("./narrowphase/Hull")
local Mpr = require("./narrowphase/Mpr")
local Bvh = require("./broadphase/Bvh")

local BlockHull = Hull.fromObj(require("./shapes/Cube"))
local Body = require("./Body")

local FLAG_NO_SLEEP = 1

local SLEEP_THRESHOLD = 2
local SLEEP_TIMER = 0.5

export type Arbiter = Arbiter.Arbiter
export type Body = Body.Body
export type Bvh = Bvh.Bvh<Body>
export type Support = Hull.Support

export type RaycastResult = {
	Instance: Body,
	Position: Vector3,
	Normal: Vector3,
	Distance: number,
}

export type World = {
	Gravity: number,
	
	numActive: number,
	arbiters: { [number]: Arbiter },
	active: { [number]: boolean },
	bodies: { Body },
	bvh: Bvh,
}

local World = {}
World.Gravity = 196.2
World.numActive = 0
World.bvh = Bvh.new() :: Bvh
World.bodies = {}
World.active = {}
World.arbiters = {}

local function getShape(part: BasePart): Hull.Obj
	-- current shape right now is a block, but you can do other stuff
	-- like wedges tris, etc. however, the biggest limitation are smooth
	-- objects(because i didn't add incremental manifolds yet and clipping is
	-- horribly inefficient for smooth shapes)
	return BlockHull
end

local function shouldSleep(body: Body): boolean
	-- checks if the linear velocity isnt much
	local velocity = body.linearVelocity
	local angular = body.angularVelocity
	return velocity:Dot(velocity) < SLEEP_THRESHOLD * SLEEP_THRESHOLD
		and angular:Dot(angular) < SLEEP_THRESHOLD * SLEEP_THRESHOLD
end

function World.addDynamicBody(part: BasePart): Body
	-- adds a dynamic body to the bvh
	local body = Body.newDynamic(part, getShape(part))
	local id = World.bvh:insert(World.bvh.createNode(body))
	World.bodies[id] = body
	World.active[id] = true
	part.Anchored = true
	return body
end

function World.addStaticBody(part: BasePart): Body
	-- adds a static body to the bvh
	local body = Body.newStatic(part, getShape(part))	
	local id = World.bvh:insert(World.bvh.createNode(body))
	World.bodies[id] = body
	
	return body
end

function World.wakeBody(id: number)
	local body = World.bodies[id]
	if body.mass == math.huge then
		-- do not awake objects that are static
		return
	end
	
	-- set the body to be active
	World.active[id] = true
end

function World.markBodyActive(body: Body, state: boolean)
	-- this makes a body always active, ie.
	-- it cannot sleep
	if state then
		body.flags = bit32.bor(body.flags, FLAG_NO_SLEEP)
	else
		body.flags = bit32.band(body.flags, bit32.bnot(FLAG_NO_SLEEP))
	end
end

function World.sleepBody(id: number, dt: number)
	-- this adds onto the sleep timer
	local body = World.bodies[id]
	body.sleep = math.min(body.sleep + dt, SLEEP_TIMER)
	
	if body.sleep >= SLEEP_TIMER
		and not bit32.btest(body.flags, FLAG_NO_SLEEP) then
		-- set the object to sleep
		World.active[id] = nil
	end
end

local updateArbiter = Arbiter.update
local getExtentsSize = Body.getExtentsSize
local padding = Vector3.one
function World.broadphase(dt: number)
	-- this phase requres you to calculate everything... not really lol
	local arbiters = World.arbiters
	local bodies = World.bodies
	local bvh = World.bvh
	
	debug.profilebegin("world::broadphase")
	
	-- create new arbiters list
	local newArbiters = {}
	for idA in World.active do
		local shapeA = bodies[idA]
		if shouldSleep(shapeA) then
			-- add to the sleep timer
			World.sleepBody(idA, dt)
		else
			-- reset the sleep timer thanks
			shapeA.sleep = 0.0
		end
		
		local min, max = getExtentsSize(shapeA.cframe, shapeA.size)
		local hits = bvh:query(min - padding, max + padding)
		for i, idB in hits do
			if idB == idA then
				-- do not create arbiters of yourself lol
				continue
			elseif newArbiters[bit32.lshift(idA, 16) + idB] then
				-- do not create repeat contacts
				continue
			end
			
			-- get the arbiter id for contact pair
			-- matching nice
			local left = math.min(idA, idB)
			local right = math.max(idA, idB)
			
			local arbiterId = bit32.lshift(left, 16) + right
			local shapeB = bodies[idB]
			
			local maybeArbiter = arbiters[arbiterId]
			local arbiter
			if maybeArbiter then
				-- we can exploit warm starting because
				-- we already have an existing contact here
				arbiter = updateArbiter(maybeArbiter, dt)
			else
				-- otherwise just create a new arbiter
				arbiter = Arbiter.new(shapeA, shapeB)
			end
			
			if not arbiter then
				-- no contact constraint created
				continue
			end
			
			World.wakeBody(idB)
			newArbiters[arbiterId] = arbiter
		end
	end
	
	debug.profileend()
	
	World.arbiters = newArbiters
end

function World.integrateForces(dt: number)
	-- add forces and torques here
	local bodies = World.bodies
	for id in World.active do
		local body = bodies[id]
		body.linearVelocity += body.invMass * body.force * dt
		body.angularVelocity += body.invInertia * body.torque * dt
	end
end

local solveArbiter = Arbiter.solve
function World.integrateConstraints(dt: number)
	-- integrates constraints such as contact constraints(for now)
	debug.profilebegin("world::integrate_constraints")
	
	for i = 1, 32 do
		for id, arbiter in World.arbiters do
			solveArbiter(arbiter, dt)
		end
	end
	
	debug.profileend()
end

function World.query(min: Vector3, max: Vector3): { Body }
	-- queries the bodies from the world bvh
	local bodies = World.bodies
	local output = {}

	for i, hitId in World.bvh:query(min, max) do
		table.insert(output, bodies[hitId])
	end
	
	return output
end

function World.shapecast(
	origin: Vector3,
	direction: Vector3,
	shape: Support,
	size: Vector3
): (Body?, Vector3, Vector3)
	-- the size is the approximate size of the support point
	-- think of it as the maximum bounding box
	local bodies = World.bodies

	local normal: Vector3 = Vector3.zero
	local object: Body?
	local t: number = 1.0

	for i, hitId in World.bvh:trace(
		origin,
		direction,
		size
	) do
		local hitObject = bodies[hitId]
		local support = hitObject.hull.support
		local hitT, hitNormal = Mpr(function(direction: Vector3)
			return support(direction) - shape(direction)
		end, direction)

		if hitT < t then
			-- this is the closest object that we have
			-- hit thus far
			object = hitObject
			normal = hitNormal
			t = hitT
		end
	end

	return object, origin + direction*t, normal
end

function World.raycast(
	origin: Vector3,
	direction: Vector3,
	ignore: Instance
): RaycastResult?
	local bodies = World.bodies
	local normal: Vector3 = Vector3.zero
	local body: Body? = nil
	local t: number = 1.0

	for i, hitId in World.bvh:trace(
		origin,
		direction,
		Vector3.zero
	) do
		-- we raycast on the candidates and we track
		-- the one with the least T value.
		local hitObject = bodies[hitId]
		if ignore and hitObject.part:IsDescendantOf(ignore) then
			-- from the ignore list
			continue
		end

		local hitT, hitPosition, hitNormal = hitObject.hull:raycast(
			origin,
			direction
		)

		if hitT < t then
			normal = hitNormal
			body = hitObject
			t = hitT
		end
	end

	if not body then
		-- no need to return anything lol
		return nil
	end

	return {
		Instance = body,
		Position = origin + direction*t,
		Normal = normal,
		Distance = direction.Magnitude * t,
	}
end

function World.integrateBodies(dt: number)
	-- loop through all bodies and integrate them
	debug.profilebegin("world::integrate_bodies")
	
	local bodies = World.bodies
	local bvh = World.bvh
	
	World.numActive = 0
	
	for id in World.active do
		-- add rotation
		local body = bodies[id]
		local deltaAngularVelocity = (body.angularVelocity + body.angularMomentum) * dt
		local halfAngle = deltaAngularVelocity.Magnitude * 0.5
		if 0.0 < halfAngle then
			-- this integrates position and rotation
			local cos = math.cos(halfAngle)
			local sin = math.sin(halfAngle)

			local position = body.cframe.Position
				+ (body.linearVelocity + body.linearMomentum)*dt
			body.cframe = CFrame.new(
				position.X, position.Y, position.Z,
				sin * deltaAngularVelocity.X,
				sin * deltaAngularVelocity.Y,
				sin * deltaAngularVelocity.Z,
				2 * cos * halfAngle
			) * body.cframe.Rotation
		else
			-- this only integrates position
			body.cframe += (body.linearVelocity + body.linearMomentum) * dt
		end

		-- zero out the split impulse
		body.linearMomentum *= 0
		body.angularMomentum *= 0
		
		-- update the current hull with the new
		-- updated stuff
		bvh:update(
			id,
			getExtentsSize(body.cframe, body.size)
		)
		
		body.hull:update(
			body.cframe,
			body.size
		)
		
		-- body.part.AssemblyAngularVelocity = body.angularVelocity
		-- body.part.AssemblyLinearVelocity = body.linearVelocity
		body.part.CFrame = body.cframe
		
		World.numActive += 1
	end
	
	debug.profileend()
end

function World.step(dt: number)
	-- do broadphase
	World.broadphase(dt)
	
	-- integrate forces
	World.integrateForces(dt)
	
	-- initialize constraints
	World.integrateConstraints(dt)
	
	-- integrate velocities
	World.integrateBodies(dt)
end

return World