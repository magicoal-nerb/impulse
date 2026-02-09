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

local function twin(edge: number): number
	-- Gets the dual
	local a = bit32.rshift(edge, 16)
	local b = bit32.band(edge, 0xFFFF)

	return bit32.lshift(b, 16) + a
end

local function queryEdge(
	verticies: { Vector3 },
	edge: number
): (Vector3, Vector3)
	-- Query edges of the verticies
	return verticies[bit32.band(edge, 0xFFFF)],
		verticies[bit32.rshift(edge, 16)]
end

local function areArcsIntersecting(
	bxa: Vector3,
	dxc: Vector3,

	a: Vector3,
	b: Vector3,
	c: Vector3,
	d: Vector3
): boolean
	-- Checks if the arc spanning a->b and c->d are
	-- colliding. If it is, then we can test this pair
	local cba = c:Dot(bxa)
	local dba = d:Dot(bxa)
	local adc = a:Dot(dxc)
	local bdc = b:Dot(dxc)

	local sameHemisphere = 0.0 < cba * bdc
	local arcIntersection = cba * dba < 0.0
		and adc * bdc < 0.0

	return sameHemisphere and arcIntersection
end

local function queryEdgeDirections(hullA: Hull, hullB: Hull): (Vector3, number, number, number)
	-- Check all possible edge combinations of A and B as potential separating
	-- axes. Only happens in 3D
	local facesA = hullA.faces
	local facesB = hullB.faces

	local vertsA = hullA.verticies
	local vertsB = hullB.verticies

	local normsA = hullA.normals
	local normsB = hullB.normals

	local bestDirection = Vector3.zero
	local bestDistance = -math.huge
	local bestEdgeA = 0
	local bestEdgeB = 0

	local origin = hullA.center
	for i, edgeA in hullA.edges do
		-- get the edge from A
		local normalB = facesA[normsA[twin(edgeA)]].normal
		local normalA = facesA[normsA[edgeA]].normal

		local pointA, pointB = queryEdge(vertsA, edgeA)
		local dirAB = pointB - pointA

		for j, edgeB in hullB.edges do
			-- Get the edge from B
			local normalD = facesB[normsB[twin(edgeB)]].normal
			local normalC = facesB[normsB[edgeB]].normal

			local pointC, pointD = queryEdge(vertsB, edgeB)
			local dirCD = pointD - pointC

			-- The idea is that we only care for edges
			-- that lie on the minkowski sum of both the hulls.
			-- We can check this through seeing if the gauss map of
			-- these edges collide.
			if not areArcsIntersecting(
				-dirAB,
				-dirCD,
				normalA,
				normalB,
				-normalC,
				-normalD
			) then
				-- A and B are not on the minkowski sum
				continue
			end

			-- Compute the separating axis
			local axis = dirAB:Cross(dirCD)
			if axis:Dot(axis) < 0.005^2 then
				-- Don't calculate the axis
				continue
			elseif axis:Dot(pointA - origin) < 0.0 then
				-- Assume that we maintain
				-- consistent normal orientation
				axis = -axis
			end

			-- Check the support point of B onto A
			local direction = axis.Unit
			local distance = (pointC - pointA):Dot(direction)
			if EPSILON < distance then
				-- Do an early exit
				return direction, distance,
					edgeA, edgeB
			elseif bestDistance < distance then
				-- Mark our direction and edges
				bestDirection = direction
				bestDistance = distance
				bestEdgeA = edgeA
				bestEdgeB = edgeB
			end
		end
	end

	if bestEdgeA == 0 then
		return Vector3.zero, -math.huge, 1, 1
	end

	return bestDirection, bestDistance,
		bestEdgeA, bestEdgeB
end

local function queryFaceDirections(hullA: Hull, hullB: Hull): (Face, number)
	local bestDistance = -math.huge
	local bestFace = 0

	-- It is known for a convex shape, support(face.normal) lies on
	-- the current face
	local vertsA = hullA.verticies	
	for i, face in hullA.faces do
		-- query the support point
		local normal = face.normal
		local vertexA = vertsA[face.verticies[1]]
		local vertexB = hullB.support(-normal)

		local distance = (vertexB - vertexA):Dot(normal)
		if EPSILON < distance then
			-- Terminate early because we
			-- already hit something!
			return face, distance
		elseif bestDistance < distance then
			bestDistance = distance
			bestFace = i
		end
	end

	-- return a record of the current face
	assert(bestFace ~= 0, "hull had no faces!")
	return hullA.faces[bestFace], bestDistance
end

return function(
	hullA: Hull,
	hullB: Hull
): Contact?
	-- the possible separating axes between two convex
	-- polyhedra are between: face normals of A and B
	-- 		edge cross product combinations of A and B
	debug.profilebegin("narrowphase::sat")
		
	local faceA, distanceA = queryFaceDirections(hullA, hullB)
	if EPSILON < distanceA then
		-- we found a separating axis
		debug.profileend()
		return nil
	end

	local faceB, distanceB = queryFaceDirections(hullB, hullA)
	if EPSILON < distanceB then
		-- we found a separating axis
		debug.profileend()
		return nil
	end
	
	local edgeAB, distanceAB, edgeA, edgeB = queryEdgeDirections(hullA, hullB)
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