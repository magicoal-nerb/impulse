--!native
--!strict

-- Separating axis theorem helper module
-- Used to get proper contact points for our physics engine

--[[
	According to the SAT, if there exists any projection where the two
	shapes are not colliding, then they do not collide. This should make
	intuitive sense because if you're below a wall's top, and above a wall's bottom,
	it does not imply you are colliding with it.

	A covenient thing we can get from the separating axis theorem is that if an object is colliding,
	then if we get the minimum collision depth from all of the projections, we can find the
	minimal translation vector.

	With this projection, if it is a face:
		We can clip the closest face on one shape and clip it along the opposite shape.
		This operation will give us a contact manifold that we will apply impulses to in the integration
		step. This contact manifold is just a list of the contact points.

		For stability's sake, we limit the amount of contact points to 4, and we try to maximize area
		along those four points for a more stable contact manifold 

	If it is an edge:
		Then we can generate one contact point which is the closest point between both edges
]]

local Hull = require("./Hull")

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

local function cleanupVerticies(inputList: { Vector3 }): { Vector3 }
	-- Inplace vertex cleanup
	local hashmap = {} :: { [Vector3]: boolean }
	local left = 1
	
	for i, point in inputList do
		local hash = (point * 1e3):Floor()
		if hashmap[hash] then
			-- We have a similar vertex!
			continue
		end
		
		hashmap[hash] = true
		inputList[left] = point
		left += 1
	end
	
	for i = left, #inputList do
		table.remove(inputList)
	end
	
	return inputList
end

local function optimizeManifold(
	inputList: { Vector3 },
	normal: Vector3
): { Vector3 }
	if #inputList <= 4 then
		return cleanupVerticies(inputList)
	end

	-- There's more than 4 contact points, so we will have
	-- to find a combination that maximizes area
	local a = table.remove(inputList) :: Vector3
	local bi = 0
	local ci = 0
	local di = 0

	-- then, find the furthest point away from A.
	local dot = 0.0
	for j, point in inputList do
		local ap = point - a
		local apap = ap:Dot(ap)

		if dot < apap then
			dot = apap
			bi = j
		end
	end

	assert(bi ~= 0)

	-- Then, find the point which would create 
	-- the largest/smallest triangular area.
	local b = table.remove(inputList, bi) :: Vector3
	local maxArea = -math.huge
	local minArea = math.huge
	for j, p in inputList do
		local pa = a - p
		local pb = b - p

		-- Calculate the area of a triangle, and
		-- check if they are the furthest away from the triangle.
		local area = pa:Cross(pb):Dot(normal)
		if maxArea < area then
			-- maximum area created.
			maxArea = area
			ci = j
		end

		if minArea > area then
			-- minimum area created.
			minArea = area
			di = j
		end
	end

	assert(ci ~= 0)
	assert(di ~= 0)

	return cleanupVerticies({
		a, b,
		inputList[ci],
		inputList[di],
	})
end

local function clipHullFaces(
	hullA: Hull,
	hullB: Hull,
	faceA: Face
): { Vector3 }
	-- Gather the input list
	local initialVerts = #faceA.verticies
	local inputList = table.create(initialVerts) :: { Vector3 }
	for i, vertex in faceA.verticies do
		inputList[i] = hullA.verticies[vertex]
	end

	-- Extension of the sutherland-hodgman algorithm into 3D,
	-- clipping edges and planes. Assumes the initial verticies
	-- maintain their winding order, which will be maintained
	-- throughout the algorithm
	local vertsB = hullB.verticies
	for i, face in hullB.faces do
		local normal = face.normal
		local w = vertsB[face.verticies[1]]:Dot(normal) + 1e-2

		local outputList = {} :: { Vector3 }
		local length = #inputList
		for j = 1, length do
			local k = if j == 1
				then length
				else j - 1

			local vj = inputList[j] -- current vertex
			local vk = inputList[k] -- previous vertex

			-- calculate dot products to check
			-- which side j and k lie on the plane
			local vjk = vj - vk
			local vjd = vj:Dot(normal)
			local vkd = vk:Dot(normal)

			if vjd < w then
				if w < vkd then
					-- The previous vertex is infront of the plane, so insert
					-- the intersection from the previous to the current plane(cuz IVT.. yk)

					-- dot(vk + (vj-vk)t, normal) = w
					-- dot(vk, normal) + dot(vj-vk,normal)t = w
					-- t = (w - dot(vk, normal)) / dot(vj-vk, normal)
					-- t = (w - vkd) / (vjd - vkd)
					local t = (w - vkd)/(vjd - vkd)
					table.insert(outputList, vk + vjk*t)
				end

				-- Our current vertex is behind the plane, so add it
				table.insert(outputList, vj)
			elseif vkd < w then
				-- The previous vertex was behind the plane, and the current
				-- vertex is infront of the plane, so insert an intersection point(also cuz IVT)

				-- dot(vk + (vj-vk)t, normal) = w
				-- dot(vk, normal) + dot(vj-vk,normal)t = w
				-- t = (w - dot(vk, normal)) / dot(vj-vk, normal)
				-- t = (w - vkd) / (vjd - vkd)
				local t = (w - vkd)/(vjd - vkd)
				table.insert(outputList, vk + vjk*t)
			end
		end

		inputList = outputList
	end

	return optimizeManifold(inputList, faceA.normal)
end

local EPSILON = 1e-3

return function(
	hullA: Hull,
	hullB: Hull
): Contact?
	-- The possible separating axes between two convex
	-- polyhedra are between: face normals of A and B
	-- 		edge cross product combinations of A and B

	local faceA, distanceA = hullA:queryFaceDirections(hullB)
	if EPSILON < distanceA then
		-- We found a separating axis
		return nil
	end

	local faceB, distanceB = hullB:queryFaceDirections(hullA)
	if EPSILON < distanceB then
		-- We found a separating axis
		return nil
	end
	
	local edgeAB, distanceAB, edgeA, edgeB = hullA:queryEdgeDirections(hullB)
	if EPSILON < distanceAB then
		-- We found a separating axis
		return nil
	end

	-- Check if we have greater separation on the faces, because
	-- we will prioritize faces over edges for stability
	if distanceA <= distanceAB or distanceB <= distanceAB then
		-- use a face contact between A and B
		local sep = math.max(distanceA, distanceB)
		local contactA = clipHullFaces(hullA, hullB, faceA)
		local contactB = clipHullFaces(hullB, hullA, faceB)
		if #contactA == 0 or #contactB == 0 then
			return nil
		end

		return {
			sepA = -distanceA,
			axisA = faceA.normal,

			sepB = -distanceB,
			axisB = faceB.normal,

			contactA = contactA,
			contactB = contactB,
		}
	end
	
	-- Use an edge contact, there only exists 1 contact point
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
	return {
		sepA = distanceAB,
		axisA = edgeAB,
		sepB = -distanceAB,
		axisB = -edgeAB,
		contactA = { rootA },
		contactB = { rootB },
	}
end