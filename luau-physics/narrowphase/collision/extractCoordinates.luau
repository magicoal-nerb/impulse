--!strict

--[[
	Used to get the world space offsets
	of feature points in the GJK/EPA subroutines
]]

local EPSILON = 1e-3

export type Feature = {
	pA: Vector3,
	pB: Vector3,
	delta: Vector3,
}

local function saturate(x: number): number
	-- Helper clamp function
	return math.clamp(x, 0.0, 1.0)
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

return {
	cartesian = cartesian,
	barycentric = barycentric,
}