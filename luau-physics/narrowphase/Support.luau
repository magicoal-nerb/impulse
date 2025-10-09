--!native
--!strict

export type Support = (vec: vector) -> vector

local function castVector(vec: Vector3): vector
	-- roblox should just make a cast here, but we have
	-- to do this explicitly
	return vector.create(vec.X, vec.Y, vec.Z)
end

local function castVector3(vec: vector): Vector3
	-- roblox should just make a cast here, but we have
	-- to do this explicitly
	return Vector3.new(vec.x, vec.y, vec.z)
end

local function createConvexSupport(
	cframe: CFrame,
	size: Vector3,
	verticies: { vector }
): Support
	-- this creates a mini support object
	-- for our given object. we don't pass over
	-- the part so we don't spam indexes
	local supports = table.create(#verticies) :: { vector }
	local halfSize = size * 0.5

	for i, vertex in verticies do
		local localPoint = castVector3(vertex) * halfSize
		supports[i] = castVector(cframe:PointToWorldSpace(localPoint))
	end

	return function(vec: vector): vector
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in supports do
			local dot = vector.dot(point, vec)
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
	vector.create(-1, -1, -1),
	vector.create(-1, -1, 1),
	vector.create(-1, 1, -1),
	vector.create(-1, 1, 1),

	vector.create(1, -1, -1),
	vector.create(1, -1, 1),
	vector.create(1, 1, -1),
	vector.create(1, 1, 1),
})

return function(part: Part): Support?
	-- only support shape thus far is a cube
	return createConvexSupport(
		part.CFrame,
		part.Size,
		GjkCube
	)
end