--!native
--!strict

--[[
	Gets a one-shot contact manifold between
	different shapes by using the separating axis theorem.
]]

local Hull = require("./Hull")

local clipHullFaces = require("./clipHullFaces")

local EPSILON = 1e-3

export type Support = Hull.Support
export type Face = Hull.Face
export type Hull = Hull.Hull

export type Contact = {
	sepA: number,
	sepB: number,
	axisA: Vector3,
	axisB: Vector3,
	
	contactA: { Vector3 },
	contactB: { Vector3 },
}

return function(
	hullA: Hull,
	hullB: Hull
): Contact?
	-- the possible separating axes between two convex
	-- polyhedra are between: face normals of A and B
	-- 		edge cross product combinations of A and B
	debug.profilebegin("narrowphase::sat")
	
	local faceA, distanceA = hullA:queryFaceDirections(hullB)
	if EPSILON < distanceA then
		-- we found a separating axis
		debug.profileend()
		return nil
	end

	local faceB, distanceB = hullB:queryFaceDirections(hullA)
	if EPSILON < distanceB then
		-- we found a separating axis
		debug.profileend()
		return nil
	end
	
	local edgeAB, distanceAB, edgeA, edgeB = hullA:queryEdgeDirections(hullB)
	if EPSILON < distanceAB then
		-- we found a separating axis
		debug.profileend()
		return nil
	end
	
	if distanceA >= distanceAB and distanceA >= distanceB then
		-- Face contact with B
		local contact = clipHullFaces(hullA, hullB, faceA)	
		debug.profileend()	
		return {
			sepA = -distanceA,
			axisA = faceA.normal,

			sepB = -distanceA,
			axisB = -faceA.normal,

			contactA = contact,
			contactB = contact,
		}
	elseif distanceB >= distanceAB then
		-- Face contact with B
		local contact = clipHullFaces(hullB, hullA, faceB)		
		debug.profileend()
		return {
			sepA = -distanceB,
			axisA = -faceB.normal,

			sepB = -distanceB,
			axisB = faceB.normal,

			contactA = contact,
			contactB = contact,
		}
	end

	-- use an edge contact, there only exists 1 contact point
	-- and it is the closest point between the two edges
	local a, b = hullA:queryEdge(edgeA)
	local c, d = hullB:queryEdge(edgeB)

	-- a + ab*t_1 = c + cd*t_2
	-- a - c = cd*t_2 - ab*t_1
	local ab = b - a
	local cd = d - c
	local ca = a - c

	-- ca x cd = (cd*t_2 - ab*t_1) x cd
	-- ca x cd = -(ab x cd)t_1
	-- (ca x cd) . (ab x cd) = -((ab x cd) . (ab x cd))t_1
	-- -((ca x cd) . (ab x cd)) / |ab x cd|^2 = t_1
	local cacd = ca:Cross(cd)
	local abcd = ab:Cross(cd)
	local t1 = math.clamp(-cacd:Dot(abcd) / abcd:Dot(abcd), 0.0, 1.0)

	-- ca x ab = (cd*t_2 - ab*t_1) x ab
	-- ca x ab = (cd x ab)t_2
	-- (ca x ab) . (cd x ab) = ((cd x ab) . (cd x ab))t_2
	-- ((ca x ab) . (cd x ab)) / |cd x ab|^2 = t_2
	local caab = ca:Cross(ab)
	local cdab = cd:Cross(ab)
	local t2 = math.clamp(caab:Dot(cdab) / cdab:Dot(cdab), 0.0, 1.0)
	
	local rootA = a + ab * t1
	local rootB = c + cd * t2

	debug.profileend()
	return {
		sepA = distanceAB,
		axisA = edgeAB,
		sepB = -distanceAB,
		axisB = -edgeAB,
		contactA = { rootA },
		contactB = { rootB },
	}
end