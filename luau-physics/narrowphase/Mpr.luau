--!strict

--[[
	MPR attempts to find a closest shape that collides with a ray.
	ie, like a GJK which is biased on hitting a ray.

	In essence, we are raycasting through the minkowski sum of an entire shape, so
	think of the way Quake derived games do a "box trace," which in reality,
	is just a raycast through an inflated version of the scene geometry.
]]

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

local function tripleProduct(a: Vector3, b: Vector3, c: Vector3)
	-- a x (b x c) = b(a.c) - c(a.b)
	return b*a:Dot(c) - c*a:Dot(b)
end

export type Support = (direction: Vector3) -> Vector3

return function(
	supp: Support,
	direction: Vector3
): (number, Vector3)
	local v0 = direction.Unit

	-- Finds a candidate portal.
	-- The portal is a triangle, but the
	-- overall simplex is a tetrahedron.
	local v1 = supp(-v0)
	local v2 = supp(v1:Cross(v0))
	local v3 = supp((v2-v0):Cross(v1-v0))

	-- Now, try to find a portal
	-- whose triangle will collide
	-- with the origin ray.
	for i = 1, MPR_DISCOVER_STEPS do
		-- In this step, we check if our tetrahedron
		-- planes would collide with the origin.

		-- We only care about cases where the triangle
		-- shares a vertex with V0.
		local n012 = (v1-v0):Cross(v2-v0)
		local n023 = (v2-v0):Cross(v3-v0)
		local n031 = (v3-v0):Cross(v1-v0)

		-- If find a triangle which can see the origin point, make sure the normals
		-- are outwards facing(ie. away from the centeroid of the tetrahedron)
		-- such that we can replace the complement vertex with the proposed support point
		if sameSide(n012:Dot(v3), -n012:Dot(v0)) then
			-- Our origin lies on t_012
			if n012:Dot(v0) < 0.0 then
				v1, v2 = v2, v1
				n012 *= -1.0
			end

			v3 = supp(n012)
			if v3:Dot(n012) < 0.0 then
				return 1, Vector3.zero
			end
		elseif sameSide(n023:Dot(v1), -n023:Dot(v0)) then
			-- Our origin lies on t_023
			if n023:Dot(v0) < 0.0 then
				v2, v3 = v3, v2
				n023 *= -1.0
			end

			v1 = supp(n023)
			if v1:Dot(n023) < 0.0 then
				return 1, Vector3.zero
			end
		elseif sameSide(n031:Dot(v2), -n031:Dot(v0)) then
			-- Our origin lies on t_031
			if n031:Dot(v0) < 0.0 then
				v3, v1 = v1, v3
				n031 *= -1.0
			end

			v2 = supp(n031)
			if v2:Dot(n031) < 0.0 then
				return 1, Vector3.zero
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

	local normal = v12:Cross(v13)
	local proj = v0 * (v1:Dot(normal) / v0:Dot(normal))
	if tripleProduct(v12, v12, v13):Dot(proj-v1) > 0.0 then
		-- Facing towards line V1, V2
		return 1, Vector3.zero
	elseif tripleProduct(-v13, v12, v13):Dot(proj-v1) > 0.0 then
		-- Facing towards line V1, V3
		return 1, Vector3.zero
	elseif tripleProduct(v3-v2, v12, v13):Dot(proj-v2) > 0.0 then
		-- Facing towards line V2, V3
		return 1, Vector3.zero
	end

	-- Since we found a triangle that collides with our ray,
	-- we can refine it so it minimizes the distance from the ray
	-- and the plane the triangle will form.
	for i = 1, MPR_REFINE_STEPS do
		normal = (v3 - v1):Cross(v2 - v1)
		if normal:Dot(v0) > 0.0 then
			v2, v3 = v3, v2
			normal *= -1.0
		end

		local p = supp(normal)
		if v1:Dot(normal) > (p:Dot(normal) + 1e-3) then
			break
		elseif math.abs((p - v1):Dot(normal)) < 1e-3 then
			break
		end

		-- This is the second phase, where
		-- we just check which triangles are either
		-- inside or outside. 

		-- Triangle v4,v0,v1		
		local cross = p:Cross(v1)
		if cross:Dot(v0) > 0.0 then
			-- Inside v4,v0,v1
			cross = p:Cross(v2)
			if cross:Dot(v0) < 0.0 then
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
			cross = p:Cross(v3)
			if cross:Dot(v0) < 0.0 then
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
	local t = v1:Dot(normal) / (v0:Dot(normal) * direction.Magnitude)
	if 0.0 <= t and t <= 1.0 then
		return t, normal.Unit
	end

	return 1, Vector3.zero
end