--!strict

local EPSILON = 1e-3
local GJK_MAX_ITRS = 20
local MARGIN = 0.05

local Hull = require("./Hull")

local clipHullFaces = require("./clipHullFaces")

--[[
	This is the incremental strategy of getting contact manifolds
	through using a GJK closest point query to get the closest features
	of two shapes.
	
	The tricky part here is to make the GJK numerically stable, 
	which was very annoying to debug!
	
	The main trick is that we have a margin we multiply by beforehand,
	which allows us to run GJK with some contact depth.
	
	However, if the scaled versions of the objects collide, then we have
	to fallback to another algorithm like EPA/SAT/MPR.
	
	This algorithm gives us a point at a time, but this can be
	incredibly helpful for computing the collision depth of smooth convex shapes,
	and reducing other expensive algorithms(like SAT/EPA) from running constantly.
]]

export type Hull = Hull.Hull
export type Face = Hull.Face
export type Support = Hull.Support

export type Feature = {
	pA: Vector3,
	pB: Vector3,
	delta: Vector3,
}

export type Contact = {
	sepA: number,
	sepB: number,
	axisA: Vector3,
	axisB: Vector3,

	contactA: { Vector3 },
	contactB: { Vector3 },
}

local function tripleProduct(a: Vector3, b: Vector3, c: Vector3): Vector3
	-- a x (b x c)
	-- b(a.c) - c(a.b)
	return b * a:Dot(c) - c * a:Dot(b)
end

local function saturate(x: number): number
	-- Helper clamp function
	return math.clamp(x, 0.0, 1.0)
end

local function createFeature(
	pA: Vector3, cA: Vector3,
	pB: Vector3, cB: Vector3,
	direction: Vector3
): Feature
	-- Helper create function
	local unit = direction.Unit * MARGIN * 0.5
	local vA = pA - unit
	local vB = pB + unit
	
	return {
		delta = vA - vB,
		pA = vA,
		pB = vB,
	}
end

local function siftFromBits(simplex: { Feature }, bits: number)
	-- Go through the points on the simplex
	-- and add to the left. Inplace removal
	local length = #simplex
	local left = 1
	for i = 1, length do
		if bit32.btest(bits, bit32.lshift(1, i - 1)) then
			simplex[left] = simplex[i]
			left += 1
		end
	end

	for j = left, length do
		simplex[j] = nil
	end
end

local function slerp(a: Vector3, b: Vector3, t: number): Vector3
	local theta = math.acos(math.clamp(a:Dot(b), -0.99, 0.99))
	if math.abs(theta) < EPSILON then
		-- Orthogonal case
		return a * math.cos(t * math.pi * 0.5)
			+ b * math.sin(t * math.pi * 0.5)
	end
	
	local csc = 1.0 / math.sin(theta)
	return (a * math.sin(theta * (1.0 - t)) + b * math.sin(theta * t)) * csc
end

local function closestPointOnPlane(a: Vector3, normal: Vector3): (Vector3, number)
	-- Gets distance away from plane
	if normal:Dot(normal) == 0 then
		return Vector3.zero, math.huge
	end
	
	local unit = normal.Unit
	local dot = a:Dot(unit)
	
	return unit * dot, math.abs(dot)
end

local function closestPointOnTriangle(a: Vector3, b: Vector3, c: Vector3): (Vector3, number)
	-- https://github.com/embree/embree/blob/master/tutorials/common/math/closest_point.h
	local ab = b - a
	local ac = c - a
	local ap = -a


	-- [ab:Dot(ap), ac:Dot(ap), 1
	--	ab:Dot(bp), ac:Dot(bp), 1
	--	ab:Dot(cp), ac:Dot(cp), 1]

	-- Check if we are the the voronoi
	-- region of A
	local d1 = ab:Dot(ap)
	local d2 = ac:Dot(ap)
	if d1 <= 0 and d2 <= 0 then
		return a, 0b001
	end

	-- Check if we are in the voronoi
	-- region of B
	local bp = -b
	local d3 = ab:Dot(bp)
	local d4 = ac:Dot(bp)
	if d3 >= 0.0 and d4 <= d3 then
		return b, 0b010
	end

	-- Check if we are in the voronoi
	-- region of C
	local cp = -c
	local d5 = ab:Dot(cp)
	local d6 = ac:Dot(cp)
	if d6 >= 0.0 and d5 <= d6 then
		return c, 0b100
	end

	-- Gather I, voronoi of A
	local vc = d1 * d4 - d3 * d2
	if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0 then
		local v = d1 / (d1-d3)
		return a + v * ab, 0b011
	end

	-- Gather J, voronoi of B
	local vb = d5 * d2 - d1 * d6
	if vb <= 0.0 and d2 >= 0 and d6 <= 0.0 then
		local v = d2 / (d2 - d6)
		return a + v * ac, 0b101
	end

	-- Gather K, voronoi of C
	local va = d3 * d6 - d5 * d4
	if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0 then
		local v = (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return b + v * (c - b), 0b110
	end

	-- Inside of the triangle...
	local denom = 1 / (va + vb + vc)
	local v = vb * denom
	local w = vc * denom
	return a + v*ab + w*ac, 0b111
end

local function cartesian(simplex: { Vector3 }, barycentric: { number })
	-- Converts barycentric coordinates back to the
	-- cartesian coordinates
	local length = #simplex
	if length == 3 then
		-- Triangle case
		return simplex[1] * barycentric[1]
			+ simplex[2] * barycentric[2]
			+ simplex[3] * barycentric[3]
	elseif length == 2 then
		-- Line case
		return simplex[1] * barycentric[1]
			+ simplex[2] * barycentric[2]
	else
		-- Point case
		return simplex[1]
	end
end

local function barycentric(simplex: { Feature }): { number }
	local length = #simplex
	if length == 3 then
		local a, b, c = simplex[1].delta, simplex[2].delta, simplex[3].delta

		local ab = b - a
		local ac = c - a
		local bc = c - b

		local d00 = ab:Dot(ab)
		local d01 = ab:Dot(ac)
		local d11 = ac:Dot(ac)
		local d20 = -a:Dot(ab)
		local d21 = -a:Dot(ac)
		local d22 = bc:Dot(bc)
		
		-- Degeneracy tests
		local d12 = ac:Dot(bc)
		if math.abs(d00 * d11 - d01 * d01) < EPSILON
			or math.abs(d11 * d22 - d12 * d12) < EPSILON
		then
			-- One of the edges is degenerate, so
			-- we just choose the largest edge and return
			if d11 > d22 and d11 > d00 then
				-- Edge AC
				table.remove(simplex, 2)
				return barycentric(simplex)
			elseif d22 > d00 and d22 > d11 then
				-- Edge BC
				table.remove(simplex, 1)
				return barycentric(simplex)
			else
				-- Edge AB
				table.remove(simplex, 3)
				return barycentric(simplex)
			end
		end
		
		-- From Christer Ericson's Real-Time Collision Detection book
		local denom = d00 * d11 - d01 * d01
		local v = (d11 * d20 - d01 * d21) / denom
		local w = (d00 * d21 - d01 * d20) / denom
		local u = 1.0 - v - w
		
		if u < 0.0 then
			-- Edge BC
			local t = saturate(-b:Dot(bc) / bc:Dot(bc))
			u = 0.0
			v = 1.0 - t
			w = t
		elseif v < 0.0 then
			-- Edge AC
			local t = saturate(c:Dot(ac) / ac:Dot(ac))
			u = t
			v = 0.0
			w = 1.0 - t
		elseif w < 0.0 then
			-- Edge AB
			local t = saturate(-ab:Dot(a) / ab:Dot(ab))
			u = 1.0 - t
			v = t
			w = 0.0
		end

		return { u, v, w }
	elseif length == 2 then
		-- Line
		local a, b = simplex[1].delta, simplex[2].delta
		local ab = b - a

		local abab = ab:Dot(ab)
		if abab < EPSILON * EPSILON then
			-- Line is degenerate
			if a:Dot(a) < b:Dot(b) then
				-- A is closer
				return { 1.0, 0.0 }
			else
				-- B is closer
				return { 0.0, 1.0 }
			end
		else
			local t = saturate(-a:Dot(ab) / ab:Dot(ab))
			return { 1 - t, t }
		end
	else
		return { }
	end
end

local function solveSimplex(simplex: { Feature }): (Vector3, number)
	local length = #simplex
	if length == 1 then
		-- Give back opposite point
		return simplex[1].delta, 0b1
	elseif length == 2 then
		-- Check which part of the line
		-- the origin lies on
		local a = simplex[1].delta
		local b = simplex[2].delta
		
		local ao = -a
		local ab = b - a
		
		local abab = ab:Dot(ab)
		local dot = ab:Dot(ao)
		
		if dot <= 0.0 then
			-- Point A
			return a, 0b01
		elseif abab <= dot then
			-- Point B
			return b, 0b10
		else
			-- Line AB
			local t = dot / abab
			return a + ab * t, 0b11
		end
	elseif length == 3 then
		return closestPointOnTriangle(
			simplex[1].delta,
			simplex[2].delta,
			simplex[3].delta
		)
	else
		-- Check which part of the tetrahedron
		-- the origin lies on
		-- tetrahedron case
		local a = simplex[1].delta
		local b = simplex[2].delta
		local c = simplex[3].delta
		local d = simplex[4].delta
		
		local ab = b - a
		local ac = c - a
		local ad = d - a
		local bc = c - b
		local bd = d - b
		
		local abc = ab:Cross(ac)
		local acd = ac:Cross(ad)
		local adb = ad:Cross(ab)
		local bcd = bc:Cross(bd)
		
		abc *= -math.sign(abc:Dot(ad))
		acd *= -math.sign(acd:Dot(ab))
		adb *= -math.sign(adb:Dot(ac))
		bcd *= math.sign(bcd:Dot(ab))
		
		local bestDist = math.huge
		local bestFlag = 0b0000
		local bestPt = Vector3.zero
		
		bestPt, bestDist = closestPointOnPlane(a, abc)
		bestFlag = 0b0111
		
		local pt, dot = closestPointOnPlane(a, acd)
		if dot < bestDist then
			bestPt = pt
			bestDist = dot
			bestFlag = 0b1101
		end
		
		pt, dot = closestPointOnPlane(a, adb)
		if dot < bestDist then
			bestPt = pt
			bestDist = dot
			bestFlag = 0b1011
		end
		
		pt, dot = closestPointOnPlane(b, bcd)
		if dot < bestDist then
			bestPt = pt
			bestDist = dot
			bestFlag = 0b1110
		end
		
		-- Nothing!
		return bestPt, bestFlag
	end
end

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

return function(
	hullA: Hull.Hull,
	hullB: Hull.Hull,
	lastNormal: Vector3
): (Vector3?, Contact?)
	debug.profilebegin("narrowphase::incremental")
	
	local suppA = hullA.support
	local suppB = hullB.support
	
	local centerA = hullA.center
	local centerB = hullB.center
	
	local direction = centerA - centerB
	
	-- Flip because closest point queries
	-- have the direction vector flipped
	local previousDist = math.huge
	local simplex: { Feature } = {}
	local flags = 0
	
	for i = 1, GJK_MAX_ITRS do
		local feature = createFeature(
			suppA(-direction), centerA,
			suppB(direction), centerB,
			-direction
		)
		
		table.insert(simplex, feature)		
		direction, flags = solveSimplex(simplex)
		
		local distanceSquared = direction:Dot(direction) + EPSILON
		if direction:Dot(direction) < EPSILON then
			-- Converged through a collision...
			break
		elseif previousDist - distanceSquared < EPSILON * previousDist then
			-- Converged :D
			break
		end
		
		siftFromBits(simplex, flags)
		previousDist = distanceSquared
	end
	
	-- Cull out bits
	if direction == Vector3.zero then
		-- If we intersect the origin, we do not do anything.
		-- We could pass this onto EPA if we wanted to
		debug.profileend()
		return nil, nil
	elseif #simplex == 4 then
		direction, flags = solveSimplex(simplex)
		siftFromBits(simplex, flags)
	end

	-- Get the closest point on this simplex
	local closestPoint = solveSimplex(simplex)
	local faceB = getFaceInDirection(
		hullB,
		closestPoint.Unit
	)
	
	local currentNormal = -faceB.normal
	if currentNormal:Dot(lastNormal) < 0.9 then
		-- Then just clip based off of the previous normal
		return nil, nil
	end
	
	local contact = clipHullFaces(hullB, hullA, faceB)
	local vertexB = suppB(-currentNormal)
	local vertexA = suppA(currentNormal)
	
	workspace.Wireframe:AddLine(vertexA, vertexB)
	
	local distance = (vertexA - vertexB):Dot(currentNormal)
	debug.profileend()
	
	return currentNormal, {
		sepA = distance,
		axisA = currentNormal,

		sepB = distance,
		axisB = -currentNormal,

		contactA = contact,
		contactB = contact,
	}
end