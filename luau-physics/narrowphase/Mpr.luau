--!strict
-- see https://arxiv.org/pdf/2304.07357
local EPSILON = 1e-3

-- This needs to be precise.
local MPR_DISCOVER_STEPS = 6
local MPR_REFINE_STEPS = 16

local SQR_ITERATIONS = 5

local function sameSide(a, b)
	-- This allows us to check dot products
	-- without needing to fix them first.
	return (a<0) == (b<0)
end

local function tripleProduct(a: vector, b: vector, c: vector)
	-- a x (b x c) = b(a.c) - c(a.b)
	return b*vector.dot(a, c) - c*vector.dot(a, b)
end

export type Support = (direction: vector) -> vector

return function(
	supp: Support,
	direction: vector
): (number, vector)
	local v0 = vector.normalize(direction)

	-- Finds a candidate portal.
	-- The portal is a triangle, but the
	-- overall simplex is a tetrahedron.
	local v1 = supp(-v0)
	local v2 = supp(vector.cross(v1, v0))
	local v3 = supp(vector.cross(v2-v0, v1-v0))

	-- Now, try to find a portal
	-- whose triangle will collide
	-- with the origin ray.
	for i = 1, MPR_DISCOVER_STEPS do
		-- In this step, we check if our tetrahedron
		-- planes would collide with the origin.

		-- We only care about cases where the triangle
		-- shares a vertex with V0.
		local n012 = vector.cross(v1-v0, v2-v0)
		local n023 = vector.cross(v2-v0, v3-v0)
		local n031 = vector.cross(v3-v0, v1-v0)

		-- If find a triangle which can see the origin point, make sure the normals
		-- are outwards facing(ie. away from the centeroid of the tetrahedron)
		-- such that we can replace the complement vertex with the proposed support point
		if sameSide(vector.dot(n012, v3), -vector.dot(n012, v0)) then
			-- Our origin lies on t_012
			if vector.dot(n012, v0) < 0.0 then
				v1, v2 = v2, v1
				n012 *= -1.0
			end

			v3 = supp(n012)
			if vector.dot(v3, n012) < 0.0 then
				return 1, vector.zero
			end
		elseif sameSide(vector.dot(n023, v1), -vector.dot(n023, v0)) then
			-- Our origin lies on t_023
			if vector.dot(n023, v0) < 0.0 then
				v2, v3 = v3, v2
				n023 *= -1.0
			end

			v1 = supp(n023)
			if vector.dot(v1, n023) < 0.0 then
				return 1, vector.zero
			end
		elseif sameSide(vector.dot(n031, v2), -vector.dot(n031, v0)) then
			-- Our origin lies on t_031
			if vector.dot(n031, v0) < 0.0 then
				v3, v1 = v1, v3
				n031 *= -1.0
			end

			v2 = supp(n031)
			if vector.dot(v2, n031) < 0.0 then
				return 1, vector.zero
			end
		else
			-- This means our ray collides with our tetrahedron
			-- shape.
			break
		end
	end

	-- Check if our figure can generate a root
	local v12 = v2-v1
	local v13 = v3-v1

	local normal = vector.cross(v12, v13)
	local proj = v0 * (vector.dot(v1, normal) / vector.dot(v0, normal))
	if vector.dot(tripleProduct(v12, v12, v13), proj-v1) > 0.0 then
		-- Facing towards line V1, V2
		return 1, vector.zero
	elseif vector.dot(tripleProduct(-v13, v12, v13), proj-v1) > 0.0 then
		-- Facing towards line V1, V3
		return 1, vector.zero
	elseif vector.dot(tripleProduct(v3-v2, v12, v13), proj-v2) > 0.0 then
		-- Facing towards line V2, V3
		return 1, vector.zero
	end

	-- Since we found a triangle that collides with our ray,
	-- we can refine it so it minimizes the distance from the ray
	-- and the plane the triangle will form.
	for i = 1, MPR_REFINE_STEPS do
		normal = vector.cross(v3 - v1, v2 - v1)
		if vector.dot(normal, v0) > 0.0 then
			v2, v3 = v3, v2
			normal *= -1.0
		end

		local p = supp(normal)
		if vector.dot(v1, normal) > (vector.dot(p, normal) + 1e-3) then
			break
		elseif math.abs(vector.dot(p - v1, normal)) < 1e-3 then
			break
		end

		-- This is the second phase, where
		-- we just check which triangles are either
		-- inside or outside. 

		-- Triangle v4,v0,v1		
		local cross = vector.cross(p, v1)
		if vector.dot(cross, v0) > 0.0 then
			-- Inside v4,v0,v1
			cross = vector.cross(p, v2)
			if vector.dot(cross, v0) < 0.0 then
				-- Inside triangles
				-- v4,v0,v1
				-- v4,v2,v0
				v3 = p
			else
				-- Inside triangles
				-- v4,v0,v1
				-- v4,v0,v3
				v1 = p
			end
		else
			-- Outside v4,v0,v1
			cross = vector.cross(p, v3)
			if vector.dot(cross, v0) < 0.0 then
				-- Outside triangles
				-- v4,v0,v1
				-- v4,v0,v3
				v1 = p
			else
				-- Outside triangles
				-- v4,v0,v1
				-- v4,v0,v2
				v2 = p
			end
		end
	end

	-- dot(v0 * lambda, normal) = dot(v1, normal)
	-- lambda = dot(v1, normal) / dot(v0, normal)
	local t = vector.dot(v1, normal) / (vector.dot(v0, normal) * vector.magnitude(direction))
	if 0.0 <= t and t <= 1.0 then
		return t, vector.normalize(normal)
	end

	return 1, vector.zero
end