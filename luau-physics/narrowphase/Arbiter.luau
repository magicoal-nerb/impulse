--!native
--!strict

local EPSILON = 1e-2

local getContactStubborn = require("./getContactStubborn")
local getContactManifold = require("./getContactManifold")
local areHullsColliding = require("./areHullsColliding")

local Contact = require("./Contact")
local Body = require("../Body")
local Epa = require("./Epa")

export type Contact = Contact.Contact
export type Body = Body.Body

local Arbiter = {}
Arbiter.__index = Arbiter

local function hashvec(vec: Vector3): Vector3
	return (vec * 1e3):Floor()
end

export type Arbiter = {
	shapeA: Body,
	shapeB: Body,

	rotA: CFrame,
	rotB: CFrame,
	
	normal: Vector3,
	contacts: { [Vector3]: Contact },
}

local function getClosestPoint(set: { Vector3 }, to: Vector3): Vector3
	local dotB = math.huge
	local pointB: Vector3
	for j, other in set do
		-- Loop through the set and calculate
		-- the difference. The one with the least
		-- distance is the closest point
		local what = other - to
		local dot = what:Dot(what)
		if dot < dotB then
			-- set closest point
			pointB = other
			dotB = dot
		end
	end

	return pointB
end

function Arbiter.new(shapeA: Body, shapeB: Body): Arbiter?
	-- Add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center
	
	if not areHullsColliding(
		shapeA.hull.support,
		shapeB.hull.support,
		originB - originA
	) then
		return nil
	end
		
	local manifold = getContactManifold(shapeA.hull, shapeB.hull)
	assert(manifold, "gjk fail?")

	local contacts = {}
	for i, pointA in manifold.contactA do		
		-- Get the closest point to apply the impulse
		-- towards
		local pointB = getClosestPoint(manifold.contactB, pointA)
		local rA = pointA - originA
		local rB = pointB - originB

		-- We do this for warm starting, I arbritrarily
		-- chose rA as the hash
		local relative = shapeA.cframe:PointToObjectSpace(pointA)
		contacts[hashvec(relative)] = Contact.new(
			shapeA, shapeB,
			-manifold.axisB, -- normal
			rA, -- ra
			rB, -- rb
			manifold.sepB -- depth
		)
	end

	return {
		shapeA = shapeA,
		shapeB = shapeB,
		
		normal = -manifold.axisB,

		rotA = shapeA.cframe.Rotation:Inverse(),
		rotB = shapeB.cframe.Rotation:Inverse(),

		contacts = contacts
	}
end

function Arbiter.updateFromManifold(self: Arbiter, manifold: getContactManifold.Contact)
	local shapeA = self.shapeA
	local shapeB = self.shapeB

	-- add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center

	local contacts = {}
	for i, pointA in manifold.contactA do
		-- Get the closest point to apply the impulse
		-- towards
		local pointB = getClosestPoint(manifold.contactB, pointA)
		local rA = pointA - originA
		local rB = pointB - originB	

		-- We do this for warm starting, I chose rA
		-- as the hash
		local relative = shapeA.cframe:PointToObjectSpace(pointA)
		local id = hashvec(relative)
		local contact = self.contacts[id]
		if contact then
			-- Then, warm start.
			contact:update(rA, rB, -manifold.axisB, manifold.sepB)
			contacts[id] = contact
			continue
		end

		-- Generate a new contact
		local contact = Contact.new(
			shapeA, shapeB,
			-manifold.axisB, -- normal
			rA, -- ra
			rB, -- rb
			manifold.sepB -- depth
		)

		contacts[id] = contact
	end

	self.contacts = contacts
end

local function getCenter(a: { Vector3 }): Vector3
	local center = Vector3.zero
	for i, p in a do
		center += p
	end

	return center / #a
end

function Arbiter.update(self: Arbiter, dt: number): Arbiter?
	local shapeA = self.shapeA
	local shapeB = self.shapeB
	
	local contact = getContactStubborn(shapeA.hull, shapeB.hull, -self.normal)
	if not contact then
		-- The face contact failed.
		-- So, try doing the brute force.
		local bruteForce = getContactManifold(shapeA.hull, shapeB.hull)
		if not bruteForce then
			return nil
		end

		self.normal = -bruteForce.axisB
		Arbiter.updateFromManifold(self, bruteForce)
	else
		self.normal = -contact.axisB
		Arbiter.updateFromManifold(self, contact)
	end
	
	return self
end

function Arbiter.solve(self: Arbiter, dt: number)
	-- Solve our contacts!
	for i, contact in self.contacts do
		contact:solve(dt)
	end
end

return Arbiter