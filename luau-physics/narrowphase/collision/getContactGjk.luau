--!strict

local COLLISION_SLOP = 1e-2
local GJK_MAX_ITRS = 12
local EPSILON = 1e-3
local MARGIN = 0.1

local Hull = require("../Hull")

local extractCoordinates = require("./extractCoordinates")
local getContactEpa = require("./getContactEpa")

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

export type Support = Hull.Support
export type Hull = Hull.Hull
export type Face = Hull.Face

type Feature = extractCoordinates.Feature

local barycentric = extractCoordinates.barycentric
local cartesian = extractCoordinates.cartesian

export type Contact = {
	sep: number,
	axis: Vector3,

	contactA: { Vector3 },
	contactB: { Vector3 },
}

local function tripleProduct(a: Vector3, b: Vector3, c: Vector3): Vector3
	-- a x (b x c)
	-- b(a.c) - c(a.b)
	return b * a:Dot(c) - c * a:Dot(b)
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

		if 0.0 <= abc:Dot(-a) then
			bestPt = closestPointOnTriangle(a, b, c)
			bestFlag = 0b0111
			bestDist = bestPt:Dot(bestPt)
		end

		if 0.0 <= acd:Dot(-a) then
			local pt = closestPointOnTriangle(a, c, d)
			local dot = pt:Dot(pt)
			if dot < bestDist then
				bestPt = pt
				bestDist = dot
				bestFlag = 0b1101
			end
		end

		if 0.0 <= adb:Dot(-a) then
			local pt = closestPointOnTriangle(a, d, b)
			local dot = pt:Dot(pt)
			if dot < bestDist then
				bestPt = pt
				bestDist = dot
				bestFlag = 0b1011
			end
		end

		if 0.0 <= bcd:Dot(-b) then
			local pt = closestPointOnTriangle(b, c, d)
			local dot = pt:Dot(pt)
			if dot < bestDist then
				bestPt = pt
				bestDist = dot
				bestFlag = 0b1110
			end
		end

		-- Nothing!
		return bestPt, bestFlag
	end
end

local function sign(x: number)
	return if x > 0.0 then 1.0 else -1.0
end

return function(
	hullA: Hull.Hull,
	hullB: Hull.Hull
): Contact?
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
		if direction == Vector3.zero then
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
		return getContactEpa(
			simplex,
			hullA,
			hullB
		)
	elseif #simplex == 4 then
		direction, flags = solveSimplex(simplex)
		siftFromBits(simplex, flags)
	end

	debug.profileend()
	
	local coordinates = barycentric(simplex)
	local length = #simplex
	
	-- Get the simplex
	local simplexA = table.create(length, Vector3.zero)
	local simplexB = table.create(length, Vector3.zero)
	
	for i, feature in simplex do
		simplexA[i] = feature.pA
		simplexB[i] = feature.pB
	end
	
	local pA = cartesian(simplexA, coordinates)
	local pB = cartesian(simplexB, coordinates)
	local delta = pA - pB
	local separation = MARGIN - delta.Magnitude

	if separation < -EPSILON then
		return nil
	end
	
	return {
		sep = separation,
		axis = delta.Unit * sign(delta:Dot(centerA - centerB)),

		contactA = { pA },
		contactB = { pB },
	}
end