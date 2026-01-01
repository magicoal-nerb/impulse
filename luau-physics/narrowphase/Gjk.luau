--!native
--!strict

local Gjk = {}

export type Support = (direction: Vector3) -> Vector3

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

function Gjk.isColliding(
	suppA: Support,
	suppB: Support,
	direction: Vector3
): boolean
	-- this uses casey's GJK shortcut which is faster
	-- than doing the closest point query gjk which has more
	-- checks
	local simplex = { suppA(direction) - suppB(-direction) }
	local length = 1
	direction *= -1

	for i = 1, 8 do
		local a = suppA(direction) - suppB(-direction)
		if a:Dot(direction) <= 0.1 then
			-- behind the direction, that means that we found
			-- a separation, so no collision
			return false
		end

		table.insert(simplex, a)
		length += 1

		if length == 1 then
			-- point case
			direction = -a
		elseif length == 2 then
			-- line case
			local b = simplex[1]
			local ab = b - a
			local ao = -a

			-- ab x ao x ab
			direction = ab*ab:Dot(a)
			- a*ab:Dot(ab)
		elseif length == 3 then
			-- triangle case 
			local b = simplex[2]
			local c = simplex[1]

			local ab = b - a
			local ac = c - a

			-- ac x ab x ab
			local abPerp = tripleProduct(ac, ab, ab)

			-- ab x ac x ac
			local acPerp = tripleProduct(ab, ac, ac)
			if abPerp:Dot(a) < 0 then
				table.remove(simplex, 1)
				direction = abPerp
				length -= 1
			elseif acPerp:Dot(a) < 0 then
				table.remove(simplex, 2)
				direction = acPerp
				length -= 1
			else
				-- ontop or beneath the triangle
				local abc = ab:Cross(ac)
				direction = -abc * math.sign(abc:Dot(a))
			end
		elseif length == 4 then
			-- tetrahedron case
			local b, c, d = simplex[3], simplex[2], simplex[1]
			local ab, ac, ad = b - a, c - a, d - a
			local abc = ab:Cross(ac)
			local acd = ac:Cross(ad)
			local adb = ad:Cross(ab)

			abc *= -math.sign(abc:Dot(ad))
			acd *= -math.sign(acd:Dot(ab))
			adb *= -math.sign(adb:Dot(ac))

			if abc:Dot(a) < 0 then
				-- facing infront of abc
				table.remove(simplex, 1)
				direction = abc
				length -= 1
			elseif acd:Dot(a) < 0 then
				-- facing infront of acd
				table.remove(simplex, 2)
				direction = acd
				length -= 1
			elseif adb:Dot(a) < 0 then
				-- facing infront of adb
				table.remove(simplex, 3)
				direction = adb
				length -= 1
			else
				-- inside of tetrahedron, end this
				return true
			end
		end
	end

	return false
end

function Gjk.getClosestPoint(simplex: { Vector3 }): Vector3
	local length = #simplex
	if length == 1 then
		-- point case
		return simplex[1]
	elseif length == 2 then
		-- line case
		local a = simplex[1]
		local b = simplex[2]
		local ab = b - a

		local t = math.clamp(-ab:Dot(a) / ab:Dot(ab), 0.0, 1.0)
		return a + ab * t
	elseif length == 3 then
		-- triangle case
		local a = simplex[1]
		local b = simplex[2]
		local c = simplex[3]

		-- a x (b x c) -> -c x (a x b)
		-- ab x -a x ab
		local ab = b - a
		local abPerp = tripleProduct(-ab, ab, -a)
		if abPerp:Dot(a) < 0 then
			-- line ab
			local abab = ab:Dot(ab)
			local t = math.clamp(-a:Dot(ab) / abab, 0.0, 1.0)
			return a + ab*t
		end

		-- ac x -a x ac
		local ac = c - a
		local acPerp = tripleProduct(-ac, ac, -a)
		if acPerp:Dot(a) < 0 then
			-- line ac
			local acac = ac:Dot(ac)
			local t = math.clamp(-a:Dot(ac) / acac, 0.0, 1.0)
			return a + ac*t
		end

		-- -b x cb x cb
		local bc = c - b
		local bcPerp = tripleProduct(-bc, bc, -b)
		if bcPerp:Dot(b) < 0 then
			-- line bc
			local bcbc = bc:Dot(bc)
			local t = math.clamp(-b:Dot(bc) / bcbc, 0.0, 1.0)
			return b + bc*t
		end

		-- otherwise, it's just a point on the plane
		local normal = ab:Cross(ac)
		return normal * normal:Dot(a) / normal:Dot(normal)
	else
		-- tetrahedron, just simplify this afterwards
		local a = simplex[1]
		local b = simplex[2]
		local c = simplex[3]
		local d = simplex[4]

		local ao = -a
		local acd = (d-a):Cross(c-a)
		if 0.0 < acd:Dot(ao) then
			-- face ACD
			table.remove(simplex, 2)
			return Gjk.getClosestPoint(simplex)
		end

		local abd = (b-a):Cross(d-a)
		if 0.0 < abd:Dot(ao) then
			-- face ABD
			table.remove(simplex, 3)
			return Gjk.getClosestPoint(simplex)
		end

		local bcd = (d-c):Cross(b-c)
		if 0.0 < -bcd:Dot(b) then
			-- face BCD
			table.remove(simplex, 1)
			return Gjk.getClosestPoint(simplex)
		end

		-- got face ABC
		table.remove(simplex, 4)
		return Gjk.getClosestPoint(simplex)
	end
end

function Gjk.getClosestSimplex(
	support: Support,
	direction: Vector3
): { Vector3 }
	-- gjk to build incremental contact manifolds
	-- quickly. just check the final simplex over multiple frames
	-- and determine if that closest simplex result is good
	local simplex = { support(direction) }
	for i = 1, 8 do
		local length = #simplex
		if length == 1 then
			-- point
			direction = -simplex[1]
		elseif length == 2 then
			-- line
			local a = simplex[1]
			local b = simplex[2]

			local ab = b - a
			local ao = -a

			-- an issue might arise with colinear
			-- points
			local normal = tripleProduct(ao, ab, ab)
			direction = math.sign(normal:Dot(ao))
				* normal
		elseif length == 3 then
			-- triangle
			local a = simplex[1]
			local b = simplex[2]
			local c = simplex[3]

			local ab = b - a
			local ac = c - a
			local bc = c - b
			local ao = -a

			local abPerp = tripleProduct(ab, ac, ab)
			local acPerp = tripleProduct(ab, ac, ac)
			local bcPerp = tripleProduct(bc, ab, ac)

			if 0.0 < abPerp:Dot(ao) then
				-- ab
				direction = abPerp
			elseif 0.0 < acPerp:Dot(ao) then
				-- ac
				direction = acPerp
			elseif 0.0 < bcPerp:Dot(ao) then
				-- bc
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
			local a = simplex[1]
			local b = simplex[2]
			local c = simplex[3]
			local d = simplex[4]

			local ao = -a
			local acd = (d-a):Cross(c-a)
			if 0.0 < acd:Dot(ao) then
				table.remove(simplex, 2)
				direction = acd
				continue
			end

			local abd = (b-a):Cross(d-a)
			if 0.0 < abd:Dot(ao) then
				table.remove(simplex, 3)
				direction = abd
				continue
			end

			local bcd = (d-c):Cross(b-c)
			if 0.0 < -bcd:Dot(b) then
				table.remove(simplex, 1)
				direction = bcd
				continue
			end
			
			
			direction = (b-c):Cross(a-c)
			table.remove(simplex, 4)
		end
	end

	return simplex
end

return Gjk