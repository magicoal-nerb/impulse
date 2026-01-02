--!native
--!strict

local Gjk = {}

export type Support = (direction: Vector3) -> Vector3

--[[
	The premise around the GJK algorithm is to create a shape such that
	it contains the origin point. We care about the origin point because consider
	a shape with an infinite amount of points inside of it. If these two shapes
	collide, then if we get the difference of those two shapes' points, there must
	exist atleast one point that is the origin point(ie. the zero vector).

	Each step of this algorithm is trying to surround the origin point, and that
	the length of the "simplex" (which you can consider as a hull of some points)
	represents the shape it has.
		Let n = |simplex|, if n = 1, it is a point.
		n = 2 => line
		n = 3 => triangle
		n = 4 => tetrahedron

	In the closest point query, my strategy is to compute the closest point of the simplex
	and point opposite towards it.

	In the general case, my strategy is to compute which shape normal faces the origin point
	and we will search in that direction
]]

local function tripleProduct(
	a: Vector3,
	b: Vector3,
	c: Vector3
): Vector3
	-- (a x b) x c
	-- -c x (a x b)
	-- b(c.a) - a(c.b)
	return b * c:Dot(a) - a * c:Dot(b)
end

function Gjk.getClosingSimplex(
	suppA: Support,
	suppB: Support,
	direction: Vector3
): { Vector3 }
	-- this uses casey's GJK shortcut which is faster
	-- than doing the closest point query gjk which has more
	-- checks
	local simplex = { suppA(direction) - suppB(-direction) }
	local length = 1
	direction *= -1

	for i = 1, 16 do
		local a = suppA(direction) - suppB(-direction)
		if a:Dot(direction) <= 0.1 then
			-- behind the direction, that means that we found
			-- a separation, so no collision
			return simplex
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
				return simplex
			end
		end
	end

	return simplex
end

return Gjk