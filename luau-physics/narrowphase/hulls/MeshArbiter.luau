--!native
--!strict

local EPSILON = 1e-2

local getContactPersistent = require("../collision/getContactPersistent")
local getContactManifold = require("../collision/getContactManifold")
local areHullsColliding = require("../collision/areHullsColliding")

local Contact = require("../Contact")
local Body = require("../../Body")

export type Contact = Contact.Contact
export type Body = Body.Body

local MeshArbiter = {}
MeshArbiter.__index = MeshArbiter

local function hashvec(vec: Vector3): Vector3
	return (vec * 1e3):Floor()
end

export type MeshArbiter = typeof(setmetatable({} :: {
	shapeA: Body,
	shapeB: Body,

	rotA: CFrame,
	rotB: CFrame,
	
	normal: Vector3,
	contacts: { [Vector3]: Contact },
}, MeshArbiter))

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

function MeshArbiter.new(shapeA: Body, shapeB: Body): MeshArbiter?
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
	if not manifold then
		return nil
	end

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
			-manifold.axis, -- normal
			rA, -- ra
			rB, -- rb
			manifold.sep -- depth
		)
	end

	return setmetatable({
		shapeA = shapeA,
		shapeB = shapeB,

		normal = -manifold.axis,

		rotA = shapeA.cframe.Rotation:Inverse(),
		rotB = shapeB.cframe.Rotation:Inverse(),

		contacts = contacts
	}, MeshArbiter)
end

function MeshArbiter.updateFromManifold(self: MeshArbiter, manifold: getContactManifold.Contact)
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
			contact:update(rA, rB, -manifold.axis, manifold.sep)
			contacts[id] = contact
			continue
		end

		-- Generate a new contact
		local contact = Contact.new(
			shapeA, shapeB,
			-manifold.axis, -- normal
			rA, -- ra
			rB, -- rb
			manifold.sep -- depth
		)

		contacts[id] = contact
	end

	self.contacts = contacts
end

function MeshArbiter.update(self: MeshArbiter, dt: number): MeshArbiter?
	local shapeA = self.shapeA
	local shapeB = self.shapeB

	local contact = getContactPersistent(shapeA.hull, shapeB.hull, -self.normal)
	if not contact then
		-- The face contact failed.
		-- So, try doing the brute force.
		local bruteForce = getContactManifold(shapeA.hull, shapeB.hull)
		if not bruteForce then
			return nil
		end

		self.normal = -bruteForce.axis
		MeshArbiter.updateFromManifold(self, bruteForce)
	else
		self.normal = -contact.axis
		MeshArbiter.updateFromManifold(self, contact)
	end

	return self
end

function MeshArbiter.solve(self: MeshArbiter, dt: number)
	-- Solve our contacts!
	for i, contact in self.contacts do
		contact:solve(dt)
	end
end

return MeshArbiter