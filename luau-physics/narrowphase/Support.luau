--!native
--!strict

export type Support = (vec: Vector3) -> Vector3

local function rayPlane(p: Vector3, v: Vector3, s: Vector3, n: Vector3)
	local r = p - s;
	local t = -r:Dot(n) / v:Dot(n)
	return p + t * v, t
end

local function createBallSupport(cframe: CFrame, halfSize: Vector3)
	local position = cframe.Position
	local radius = math.min(halfSize.X, halfSize.Y, halfSize.Z)

	return function(direction: Vector3): Vector3
		return direction.Unit * radius + position
	end
end

local function createBlockSupport(cframe: CFrame, halfSize: Vector3)
	return function(direction: Vector3): Vector3
		local delta = cframe:VectorToObjectSpace(direction)
		return cframe * (delta:Sign() * halfSize)
	end
end

local function createCylinderSupport(cframe: CFrame, halfSize: Vector3)
	local radius = math.min(halfSize.Y, halfSize.Z)
	return function(direction: Vector3)
		direction = cframe:VectorToObjectSpace(direction)

		local dotT, cPoint = direction.X, Vector3.new(halfSize.X, 0, 0)
		local h, t, final

		if dotT == 0 then
			final = direction.Unit * radius
		else
			cPoint = dotT > 0 and cPoint or -cPoint
			h, t = rayPlane(Vector3.zero, direction, cPoint, Vector3.xAxis)
			final = cPoint + (h - cPoint).Unit * radius
		end

		return cframe:PointToWorldSpace(final)
	end
end

local function createConvexSupport(
	cframe: CFrame,
	halfSize: Vector3,
	verticies: { Vector3 }
): Support
	-- this creates a mini support object
	-- for our given object. we don't pass over
	-- the part so we don't spam indexes
	local supports = table.create(#verticies) :: { Vector3 }
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

local PartTypes = table.freeze({
	[Enum.PartType.Ball] = createBallSupport,
	[Enum.PartType.Block] = createBlockSupport,
	[Enum.PartType.Cylinder] = createCylinderSupport,
})

return function(part: Part): Support?
	-- only support shape thus far is a cube
	local partType = part.Shape
	local shape = PartTypes[partType]
		or PartTypes[Enum.PartType.Block]

	return shape(part.CFrame, part.Size * 0.5)
end