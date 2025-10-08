--!native
--!strict

-- separating axis theorem helper module
-- used to get proper contact points for our physics engine
-- lol

local EPSILON = 1e-1

local Hull = require("./Hull")

export type Support = Hull.Support
export type Face = Hull.Face
export type Hull = Hull.Hull

export type Contact = {
	sepA: number,
	sepB: number,
	axisA: vector,
	axisB: vector,
	
	contactA: { vector },
	contactB: { vector },
}

-- incase if the hashing function doesnt work properly
-- i backported the luau vector hasher
local hashBuf = buffer.create(12)
local function hashVector3(vec: vector): number
	vec *= 1024.0

	buffer.writef32(hashBuf, 0, vec.x)
	buffer.writef32(hashBuf, 4, vec.y)
	buffer.writef32(hashBuf, 8, vec.z)

	local x = buffer.readu32(hashBuf, 0)
	local y = buffer.readu32(hashBuf, 4)
	local z = buffer.readu32(hashBuf, 8)
	x = x == 0x80000000 and 0 or x
	y = y == 0x80000000 and 0 or y
	z = z == 0x80000000 and 0 or z

	x = bit32.bxor(bit32.rshift(x, 17))
	y = bit32.bxor(bit32.rshift(y, 17))
	z = bit32.bxor(bit32.rshift(z, 17))

	return bit32.bxor(bit32.bxor(x * 73856093, y * 19349663), z * 83492791)
end

local function cleanupVerticies(inputList: { vector }): { vector }
	-- inplace vertex cleanup
	local hashmap = {} :: { [vector]: boolean }
	local left = 1
	
	for i, point in inputList do
		local hash = vector.floor(point * 10000)
		if hashmap[hash] then
			-- we have a similar vertex!
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
	inputList: { vector },
	normal: vector
): { vector }
	if #inputList <= 4 then
		return cleanupVerticies(inputList)
	end

	-- there's more than 4 contact points, so we will have
	-- to find a combination that maximizes area
	local a = table.remove(inputList) :: vector
	local bi = 0
	local ci = 0
	local di = 0

	-- then, find the furthest point away from A.
	local dot = 0.0
	for j, point in inputList do
		local ap = point - a
		local apap = vector.dot(ap, ap)

		if dot < apap then
			dot = apap
			bi = j
		end
	end

	assert(bi ~= 0)

	-- then, find the point which would create 
	-- the largest/smallest triangular area.
	local b = table.remove(inputList, bi) :: vector
	local maxArea = -math.huge
	local minArea = math.huge
	for j, p in inputList do
		local pa = a - p
		local pb = b - p

		-- calculate the area of a triangle, and
		-- check if they are the furthest away from the triangle.
		local area = vector.dot(vector.cross(pa, pb), normal)
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
): { vector }
	-- gather the input list
	local initialVerts = #faceA.verticies
	local inputList = table.create(initialVerts) :: { vector }
	for i, vertex in faceA.verticies do
		inputList[i] = hullA.verticies[vertex]
	end

	-- extension of the sutherland-hodgman algorithm into 3D,
	-- clipping edges and planes. assumes the initial verticies
	-- maintain their winding order, which will be maintained
	-- throughout the algorithm
	local vertsB = hullB.verticies
	for i, face in hullB.faces do
		local normal = face.normal
		local w = vector.dot(vertsB[face.verticies[1]], normal) + EPSILON

		local outputList = {} :: { vector }
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
			local vjd = vector.dot(vj, normal)
			local vkd = vector.dot(vk, normal)

			if vjd < w then
				if w < vkd then
					-- the previous vertex is infront of the plane, so insert
					-- the intersection from the previous to the current plane(cuz IVT.. yk)

					-- dot(vk + (vj-vk)t, normal) = w
					-- dot(vk, normal) + dot(vj-vk,normal)t = w
					-- t = (w - dot(vk, normal)) / dot(vj-vk, normal)
					-- t = (w - vkd) / (vjd - vkd)
					local t = (w - vkd)/(vjd - vkd)
					table.insert(outputList, vk + vjk*t)
				end

				-- our current vertex is behind the plane, so add it
				table.insert(outputList, vj)
			elseif vkd < w then
				-- the previous vertex was behind the plane, and the current
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

return function(
	hullA: Hull,
	hullB: Hull
): Contact?
	-- the possible separating axes between two convex
	-- polyhedra are between: face normals of A and B
	-- 		edge cross product combinations of A and B

	local faceA, distanceA = hullA:queryFaceDirections(hullB)
	if EPSILON < distanceA then
		-- we found a separating axis
		return
	end

	local faceB, distanceB = hullB:queryFaceDirections(hullA)
	if EPSILON < distanceB then
		-- we found a separating axis
		return
	end
	
	local sep = math.max(distanceA, distanceB)
	local contactA = clipHullFaces(hullA, hullB, faceA)
	local contactB = clipHullFaces(hullB, hullA, faceB)
	if #contactA == 0 or #contactB == 0 then
		-- no contact lol
		return
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