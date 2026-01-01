--!native
--!strict

local FLAG_DEBUG = true
local EPSILON = 1e-2


local Contact = require("./Contact")
local Body = require("../Body")
local Gjk = require("./Gjk")
local Sat = require("./Sat")

export type Contact = Contact.Contact
export type Body = Body.Body

local Arbiter = {}
Arbiter.__index = Arbiter

export type Arbiter = {
	shapeA: Body,
	shapeB: Body,
	
	rotA: CFrame,
	rotB: CFrame,
	
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
	
	local manifold = Sat(shapeA.hull, shapeB.hull)
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
		local id = (rA * 1e3):Floor()
		contacts[id] = Contact.new(
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
		
		rotA = shapeA.cframe.Rotation:Inverse(),
		rotB = shapeB.cframe.Rotation:Inverse(),
		
		contacts = contacts
	}
end

function Arbiter.update(self: Arbiter, dt: number): Arbiter?
	local shapeA = self.shapeA
	local shapeB = self.shapeB
	
	local manifold = Sat(shapeA.hull, shapeB.hull)
	if not manifold then
		-- If we can't generate a contact manifold, then we
		-- are no longer colliding
		return nil
	end

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
		
		local contact = Contact.new(
			shapeA, shapeB,
			-manifold.axisB, -- normal
			rA, -- ra
			rB, -- rb
			manifold.sepB -- depth
		)

		-- We do this for warm starting, I chose rA
		-- as the hash
		local id = (rA * 1e3):Floor()
		local closest = self.contacts[id]
		if closest then
			-- warm start the lambdas
			contact.jN.totLambda = closest.jN.totLambda
			contact.jT.totLambda = closest.jB.totLambda
			contact.jB.totLambda = closest.jT.totLambda
		end
		
		contacts[id] = contact
	end
	
	self.contacts = contacts
	return self
end

function Arbiter.solve(self: Arbiter, dt: number)
	-- Solve our contacts!
	for i, contact in self.contacts do
		contact:solve(dt)
	end
end

return Arbiter