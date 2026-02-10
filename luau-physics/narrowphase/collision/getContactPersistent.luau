--!strict

local EPSILON = 1e-3
local GJK_MAX_ITRS = 20
local MARGIN = 0.05

local Hull = require("../Hull")

local clipHullFaces = require("./clipHullFaces")

--[[
	The incremental strategy here is to just keep on
	holding onto the current normal until we can't clip anymore.
	From testing, this just straight up yields better performance
	than just GJK + clipping
]]

export type Hull = Hull.Hull
export type Face = Hull.Face
export type Support = Hull.Support

export type Contact = {
	sep: number,
	axis: Vector3,

	contactA: { Vector3 },
	contactB: { Vector3 },
}

local function getFaceInDirection(hull: Hull, direction: Vector3): Face
	-- Gets a face pointing most in the specified
	-- direction. Works only for convex stuff
	local bestFace: Face
	local bestDot = -math.huge
	
	for i, face in hull.faces do
		local dot = face.normal:Dot(direction)
		if bestDot < dot then
			bestFace = face
			bestDot = dot
		end
	end
	
	return bestFace
end

local function draw(simplex: { Vector3 })
	local length = #simplex
	for i = 1, length do
		local j = (i % length) + 1
		workspace.Wireframe:AddLine(
			simplex[i],
			simplex[j]
		)
	end
end

return function(
	hullA: Hull.Hull,
	hullB: Hull.Hull,
	lastNormal: Vector3
): Contact?
	debug.profilebegin("narrowphase::stubborn")
	
	local faceB = getFaceInDirection(hullB, lastNormal)
	local vertexB = hullB.verticies[faceB.verticies[1]]
	local vertexA = hullA.support(-lastNormal)
	
	--draw({
	--	hullB.verticies[faceB.verticies[1]],
	--	hullB.verticies[faceB.verticies[2]],
	--	hullB.verticies[faceB.verticies[3]],
	--	hullB.verticies[faceB.verticies[4]]
	--})
	
	local distance = (vertexB - vertexA):Dot(lastNormal)
	local contact = clipHullFaces(hullB, hullA, faceB, 0.1)

	debug.profileend()
	
	if -0.1 > distance then
		return nil
	elseif 1 < #contact then
		return {
			sep = distance,
			axis = lastNormal,

			contactA = contact,
			contactB = contact,
		}
	else
		return nil
	end
end