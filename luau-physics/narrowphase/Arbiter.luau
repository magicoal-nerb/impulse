--!native
--!strict

local Body = require("../Body")

local SupportArbiter = require("./hulls/SupportArbiter")
local MeshArbiter = require("./hulls/MeshArbiter")

local Arbiter = {}

type Body = Body.Body

export type Arbiter = {
	-- abstract base class :3
	new: (Body, Body) -> Arbiter?,
	update: (Arbiter, number) -> Arbiter?,
	solve: (Arbiter, number) -> (),
}

return function(bodyA: Body, bodyB: Body): Arbiter?
	if bodyA.smooth or bodyB.smooth then
		return SupportArbiter.new(bodyA, bodyB) :: Arbiter
	else
		return MeshArbiter.new(bodyA, bodyB) :: Arbiter
	end
end