--!native
--!strict

export type Support = (vec: Vector3) -> Vector3

local function createConvexSupport(
	cframe: CFrame,
	size: Vector3,
	verticies: { Vector3 }
): Support
	-- this creates a mini support object
	-- for our given object. we don't pass over
	-- the part so we don't spam indexes
	local supports = table.create(#verticies) :: { Vector3 }
	local halfSize = size * 0.5

	for i, vertex in verticies do
		local localPoint = vertex * halfSize
		supports[i] = cframe:PointToWorldSpace(localPoint)
	end

	return function(vec: Vector3): Vector3
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in supports do
			local dot = point:Dot(vec)
			if bestDot < dot then
				bestDot = dot
				bestItr = i
			end
		end

		return supports[bestItr]
	end
end

local GjkCube = table.freeze({
	-- a cube has 8 verticies and has 8 support points
	-- because it is already convex
	Vector3.new(-1, -1, -1),
	Vector3.new(-1, -1, 1),
	Vector3.new(-1, 1, -1),
	Vector3.new(-1, 1, 1),

	Vector3.new(1, -1, -1),
	Vector3.new(1, -1, 1),
	Vector3.new(1, 1, -1),
	Vector3.new(1, 1, 1),
})

return function(part: Part): Support?
	-- only support shape thus far is a cube
	return createConvexSupport(
		part.CFrame,
		part.Size,
		GjkCube
	)
end