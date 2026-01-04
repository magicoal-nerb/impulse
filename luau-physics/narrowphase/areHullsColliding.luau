--!strict

--[[
	This uses Casey's GJK shortcut which is optimized
	for yes/no intersection tests. This gives pretty poor
	closest point simplexes though.
]]

export type Support = (direction: Vector3) -> Vector3

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

return function(
	suppA: Support,
	suppB: Support,
	direction: Vector3
): boolean
	local simplex = { suppA(direction) - suppB(-direction) }
	local length = 1
	direction *= -1.0

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
			direction = ab * ab:Dot(a)
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
			if abPerp:Dot(a) < 0.0 then
				table.remove(simplex, 1)
				direction = abPerp
				length -= 1
			elseif acPerp:Dot(a) < 0.0 then
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

			if abc:Dot(a) < 0.0 then
				-- facing infront of abc
				table.remove(simplex, 1)
				direction = abc
				length -= 1
			elseif acd:Dot(a) < 0.0 then
				-- facing infront of acd
				table.remove(simplex, 2)
				direction = acd
				length -= 1
			elseif adb:Dot(a) < 0.0 then
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