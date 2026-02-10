--!native
--!strict

local getContantGjk = require("../collision/getContantGjk")
local getContactEpa = require("../collision/getContactEpa")

local Contact = require("../Contact")
local Body = require("../../Body")

export type Contact = Contact.Contact
export type Body = Body.Body

local SupportArbiter = {}
SupportArbiter.__index = SupportArbiter

export type SupportArbiter = typeof(setmetatable({} :: {
	shapeA: Body,
	shapeB: Body,

	rotA: CFrame,
	rotB: CFrame,

	normal: Vector3,
	contact: Contact,
}, SupportArbiter))

function SupportArbiter.new(shapeA: Body, shapeB: Body): SupportArbiter?
	local manifold = getContantGjk(shapeA.hull, shapeB.hull)
	if not manifold then
		-- No contact possible, so do not
		-- use this.
		return nil
	end

	-- add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center

	local pointB = manifold.contactB[1]
	local pointA = manifold.contactA[1]

	local rA = pointA - originA
	local rB = pointB - originB

	local contact = Contact.new(
		shapeA, shapeB,
		-manifold.axis, -- normal
		rA, -- ra
		rB, -- rb
		manifold.sep -- depth
	)

	return setmetatable({
		shapeA = shapeA,
		shapeB = shapeB,

		normal = -manifold.axis,

		rotA = shapeA.cframe.Rotation:Inverse(),
		rotB = shapeB.cframe.Rotation:Inverse(),

		contact = contact,
	}, SupportArbiter)
end

function SupportArbiter.update(self: SupportArbiter, dt: number): SupportArbiter?
	local shapeA = self.shapeA
	local shapeB = self.shapeB

	local manifold = getContantGjk(shapeA.hull, shapeB.hull)
	if not manifold then
		-- No contact possible, so do not
		-- use this.
		return nil
	end

	-- add contacts for A->B
	local originA = shapeA.hull.center
	local originB = shapeB.hull.center

	local pointB = manifold.contactB[1]
	local pointA = manifold.contactA[1]

	local rA = pointA - originA
	local rB = pointB - originB

	self.contact:update(rA, rB, -manifold.axis, manifold.sep)
	return self
end

function SupportArbiter.solve(self: SupportArbiter, dt: number)
	-- Solve our contacts!
	self.contact:solve(dt)
end

return SupportArbiter