--!strict

local MARGIN_FACTOR = 1

local Hull = require("./Hull")

--[[
	This is the incremental strategy of getting contact manifolds
	through using a GJK closest point query to get the closest features
	of two shapes.
	
	The main trick is that we have a margin we multiply by beforehand,
	which allows us to run GJK with some contact depth.
	
	However, if the scaled versions of the objects collide, then we have
	to fallback to another algorithm like EPA/SAT/MPR.
	
	This algorithm gives us a point at a time, but this can be
	incredibly helpful for computing the collision depth of smooth convex shapes,
	and reducing other expensive algorithms(like SAT/EPA) from running constantly.

	Also, this is a big WIP!!!!!

	The strategy is to use this for smooth objects like cylinders and spheres.
	This is because SAT performs very poorly for smooth shapes!! (plus, it's significantly discretized aswell)
]]

export type Support = (direction: Vector3) -> Vector3
export type FeaturePair = {
	suppA: Vector3,
	suppB: Vector3,
	delta: Vector3,
}

local function tripleProduct(
	a: Vector3,
	b: Vector3,
	c: Vector3
): Vector3
	-- (a x b) x c
	-- -c x (a x b)
	-- b(c.a) - a(c.b)
	return b*c:Dot(a) - a*c:Dot(b)
end

local function area(
	ax: number, ay: number,
	bx: number, by: number,
	cx: number, cy: number
): number
	-- ((b - a)x(c - a)).k
	return (ax - bx) * (by - cy)
		- (bx - cx) * (ay - by)
end

local function saturate(x: number): number
	-- Helper clamp function
	return math.clamp(x, -0.0, 1.0)
end

local function cartesian(simplex: { Vector3 }, barycentric: { number })
	-- Converts barycentric coordinates back to the
	-- cartesian coordinates
	local length = #simplex
	if length == 4 then
		-- Tetrahedron case
		return simplex[1] * barycentric[1]
			+ simplex[2] * barycentric[2]
			+ simplex[3] * barycentric[3]
			+ simplex[4] * barycentric[4]
	elseif length == 3 then
		-- Triangle case
		return simplex[1] * barycentric[1]
			+ simplex[2] * barycentric[2]
			+ simplex[3] * barycentric[3]
	elseif length == 2 then
		-- Line case
		local a, b = simplex[1], simplex[2]
		local ab = b - a
		return a + ab * barycentric[1]
	else
		-- Point case
		return simplex[1]
	end
end

local function barycentric(
	simplex: { Vector3 },
	p: Vector3
): { number }
	local length = #simplex
	if length == 4 then
		-- https://www.cdsimpson.net/2014/10/barycentric-coordinates.html
		local a, b, c, d = simplex[1], simplex[2], simplex[3], simplex[4]
		local ao = -a
		local bo = -b
		local ab = b - a
		local ac = c - a
		local ad = d - a
		local bc = c - b
		local bd = d - b

		local c1 = bd:Cross(bc)
		local c2 = ac:Cross(ad)
		local c3 = ad:Cross(ab)
		local c4 = ab:Cross(ac)
		local c5 = ac:Cross(ad)

		local d1 = bo:Dot(c1)
		local d2 = ao:Dot(c2)
		local d3 = ao:Dot(c3)
		local d4 = ao:Dot(c4)
		local e1 = ab:Dot(c5)
		local d5 = 1.0 / e1
		local ux = saturate(d1 * d5) -- bdc
		local uy = saturate(d2 * d5) -- acd
		local uz = saturate(d3 * d5) -- adb
		local uw = saturate(d4 * d5) -- abc
		
		return {ux, uy, uz, uw}
		
		--local vap = p - a
		--local vbp = p - b
		
		--local vab = b - a
		--local vac = c - a
		--local vad = d - a
		
		--local vbc = c - b
		--local vbd = d - b
		
		--local va = vbp:Dot(vbd:Cross(vbc))
		--local vb = vap:Dot(vac:Cross(vad))
		--local vc = vap:Dot(vad:Cross(vab))
		--local vd = vap:Dot(vab:Cross(vac))
		
		--local v = 1.0 / vab:Dot(vac:Cross(vad))
		
		--return { va * v, vb * v, vc * v, vd * v }
	elseif length == 3 then
		-- http://realtimecollisiondetection.net/
		local a, b, c = simplex[1], simplex[2], simplex[3]

		local abc = (b - a):Cross(c - a)

		local x = math.abs(abc.X)
		local y = math.abs(abc.Y)
		local z = math.abs(abc.Z)
		
		local nu = 0.0
		local nv = 0.0
		local ood = 0
		if x >= y and x >= z then
			-- x largest
			nu = area(p.Y, p.Z, b.Y, b.Z, c.Y, c.Z)	
			nv = area(p.Z, p.Y, c.Y, c.Z, a.Y, a.Z)
			ood = 1 / abc.X
		elseif y >= x and y >= z then
			-- y largest
			nu = area(p.X, p.Z, b.X, b.Z, c.X, c.Z)
			nv = area(p.X, p.Z, c.X, c.Z, a.X, a.Z)
			ood = -1 / abc.Y
		else
			-- z largest
			nu = area(p.X, p.Y, b.X, b.Y, c.X, c.Y)
			nv = area(p.X, p.Y, c.X, c.Y, a.X, a.Y)
			ood = 1 / abc.Z
		end

		local nx = saturate(nu * ood)
		local ny = saturate(nv * ood)
		local nz = 1 - nx - ny
		return { nx, ny, nz }
		
		--local v0 = b - a
		--local v1 = c - a
		--local v2 = p - a
		
		--local d00 = v0:Dot(v0)
		--local d01 = v0:Dot(v1)
		--local d11 = v1:Dot(v1)
		--local d20 = v2:Dot(v0)
		--local d21 = v2:Dot(v1)
		
		--local denom = 1.0 / (d00 * d11 - d01 * d01)
		--local v = (d11 * d20 - d01 * d21) * denom
		--local w = (d00 * d21 - d01 * d20) * denom
		
		--return { 1.0 - v - w, v, w }
	elseif length == 2 then
		-- Line
		local a, b = simplex[1], simplex[2]
		local ab = b - a
		local ap = p - a
		
		return { saturate(ap:Dot(ab) / ab:Dot(ab)) }
	else
		-- Point
		return { }
	end
end

local visa = Instance.new("Part")
visa.Anchored = true
visa.Size = Vector3.one
visa.Parent = workspace

local visb = visa:Clone()
visb.Parent = workspace

return function(
	hullA: Hull.Hull,
	hullB: Hull.Hull
): (Vector3, Vector3)
	-- gjk to build incremental contact manifolds
	-- quickly. just check the final simplex over multiple frames
	-- and determine if that closest simplex result is good
	
	local pA = hullA.center
	local pB = hullB.center
	
	local suppA = hullA.support
	local suppB = hullB.support
	
	local direction = pB - pA
	local simplex = {} :: { FeaturePair }
	
	for i = 1, 8 do
		local vA = suppA(direction)
		local vB = suppB(-direction)
		
		-- pA * (1.0 - MARGIN_FACTOR) + va * MARGIN_FACTOR
		local delta = (pA - pB) * (1.0 - MARGIN_FACTOR)
			+ (vA - vB) * MARGIN_FACTOR
		
		table.insert(simplex, {
			suppA = vA,
			suppB = vB,
			delta = delta,
		})

		local length = #simplex
		if length == 1 then
			-- point
			direction = -simplex[1].delta
		elseif length == 2 then
			-- line
			local a = simplex[2].delta
			local b = simplex[1].delta

			local ab = b - a
			local ao = -a

			-- an issue might arise with colinear
			-- points
			local normal = tripleProduct(ao, ab, ab).Unit
			direction = math.sign(normal:Dot(ao))
				* normal
		elseif length == 3 then
			-- triangle
			local a = simplex[3].delta
			local b = simplex[2].delta
			local c = simplex[1].delta

			local ab = b - a
			local ac = c - a
			local bc = c - b
			local ao = -a

			local abPerp = tripleProduct(ab, ac, ab).Unit
			local acPerp = tripleProduct(ab, ac, ac).Unit
			local bcPerp = tripleProduct(bc, ab, ac).Unit

			if 0.0 < abPerp:Dot(ao) then
				-- edge ab
				direction = abPerp
			elseif 0.0 < acPerp:Dot(ao) then
				-- edge ac
				direction = acPerp
			elseif 0.0 < bcPerp:Dot(ao) then
				-- edge bc
				direction = bcPerp
			else
				-- expand into a tetrahedron
				local normal = ab:Cross(ac)
				if normal:Dot(ao) < 0.0 then
					direction = -normal
				else
					direction = normal
				end
			end
		elseif length == 4 then
			-- tetrahedron, just simplify this afterwards
			local a = simplex[1].delta
			local b = simplex[2].delta
			local c = simplex[3].delta
			local d = simplex[4].delta

			local ao = -a
			local acd = (d - a):Cross(c - a).Unit
			local abc = (b - a):Cross(c - a).Unit
			local abd = (b - a):Cross(d - a).Unit
			local bcd = (d - c):Cross(b - c).Unit
			
			acd *= -math.sign(acd:Dot(a))
			abc *= -math.sign(abc:Dot(a))
			abd *= -math.sign(abd:Dot(a))
			bcd *= -math.sign(bcd:Dot(b))

			if 0.0 < abc:Dot(ao) then
				-- triangle abc
				table.remove(simplex, 4)
				direction = abc
			elseif 0.0 < acd:Dot(ao) then
				-- triangle acd
				table.remove(simplex, 2)
				direction = acd
			elseif 0.0 < abd:Dot(ao) then
				-- triangle abd
				table.remove(simplex, 3)
				direction = abd
			elseif 0.0 < bcd:Dot(-b) then
				-- triangle bcd
				table.remove(simplex, 1)
				direction = bcd
			else
				-- this means our margin collided!
				-- so, we have to default using another algorithm
				-- for this case
				return Vector3.zero, Vector3.zero
			end
		end
	end
	
	-- Calculate the final simplex
	local simplexA = {}
	local simplexB = {}
	local final = {}
	
	for i, feature in simplex do
		table.insert(final, feature.suppA - feature.suppB)
		table.insert(simplexA, feature.suppA)
		table.insert(simplexB, feature.suppB)
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
	
	draw(simplexA)
	draw(simplexB)
	
	print(#simplexA, #simplexB)
	
	-- https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
		
	local result = barycentric(final, Vector3.zero)	
	local pointA = cartesian(simplexA, result)
	local pointB = cartesian(simplexB, result)
	
	visa.CFrame = CFrame.new(pointA)
	visb.CFrame = CFrame.new(pointB)
	
	return pointA, pointB
end