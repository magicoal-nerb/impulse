--!native
--!strict

local FLAG_DEBUG = true
local EPSILON = 1e-2

local getContactManifold = require("./getManifoldEdges")

local Contact = require("./Contact")
local Body = require("../Body")
local Gjk = require("./Gjk")

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

local function drawPt(vec: Vector3, color: Color3)
	if not FLAG_DEBUG then
		return
	end
	
	local point = Instance.new("Part")
	point.Anchored = true
	point.CanCollide = false
	point.CanQuery = false
	point.Size = Vector3.one * 0.2
	point.CFrame = CFrame.new(vec.X, vec.Y, vec.Z)
	point.Color = color
	point.Parent = workspace.Folder
end

local function getClosestPoint(set: { Vector3 }, to: Vector3): Vector3
	local dotB = math.huge
	local pointB
	for j, other in set do
		-- loop through the set and calculate
		-- the difference. the one with the least
		-- distance is the closest point
		local what = other - to
		local dot = Vector3.dot(what, what)
		if dot < dotB then
			-- set closest point
			pointB = other
			dotB = dot
		end
	end
	
	return pointB
end

function Arbiter.new(shapeA: Body, shapeB: Body): Arbiter?
	-- add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center
	
	local manifold = getContactManifold(shapeA.hull, shapeB.hull)
	if not manifold then
		return nil
	end
	
	local contacts = {}
	for i, pointA in manifold.contactA do		
		-- get the closest point to apply the impulse
		-- towards
		local pointB = getClosestPoint(manifold.contactB, pointA)
		local rA = pointA - originA
		local rB = pointB - originB
		
		-- we do this for warm starting, i chose rA
		-- as the hash
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
	
	local manifold = getContactManifold(shapeA.hull, shapeB.hull)
	if not manifold then
		-- if we can't generate a contact manifold, then we
		-- are no longer colliding
		return nil
	end

	-- add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center
	
	local contacts = {}
	for i, pointA in manifold.contactA do
		-- get the closest point to apply the impulse
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

		-- we do this for warm starting, i chose rA
		-- as the hash
		local id = Vector3.floor(rA * 1e3)
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
	-- solve given some time lol
	for i, contact in self.contacts do
		contact:solve(dt)
	end
end

return Arbiter