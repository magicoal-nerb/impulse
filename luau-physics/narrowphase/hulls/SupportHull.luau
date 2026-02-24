--!native
--!strict

--[[
	Hull that uses support points instead of mesh
	stuff. Uses EPA/GJK internally.
	
	TODO: adapt the inertia tensor and volume calculations
	algorithms through support points! Could possibly sample
	the convex hull of the support points, then construct from there.
]]

local Mpr = require("../Mpr")

export type Support = (Vector3) -> Vector3

local SupportHull = {}
SupportHull.__index = SupportHull

export type SupportHull = typeof(setmetatable({} :: {
	ref: (CFrame, Vector3) -> (Support),
	center: Vector3,
	support: Support,
}, SupportHull))

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

local function createWedgeSupport(cframe: CFrame, halfSize: Vector3)
	return function(direction: Vector3)
		local delta = cframe:VectorToObjectSpace(direction)
		local sign = delta:Sign()
		if sign.Z < 1.0 then
			sign *= Vector3.new(1, -math.sign(sign.Y), 1)
		end

		return cframe * (sign * halfSize)
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

local PartTypes = table.freeze({
	[Enum.PartType.Ball] = createBallSupport,
	[Enum.PartType.Wedge] = createWedgeSupport,
	[Enum.PartType.Cylinder] = createCylinderSupport,
})

function SupportHull.new(
	partType: Enum.PartType,
	cframe: CFrame,
	size: Vector3
): SupportHull
	return setmetatable({
		support = PartTypes[partType](cframe, size * 0.5),
		center = cframe.Position,
		ref = PartTypes[partType],
	}, SupportHull)
end

function SupportHull.update(
	self: SupportHull,
	cframe: CFrame,
	size: Vector3
)
	self.support = self.ref(cframe, size * 0.5)
	self.center = cframe.Position
end

function SupportHull.raycast(
	self: SupportHull,
	origin: Vector3,
	direction: Vector3
): (number, Vector3, Vector3)
	local hitT, hitNormal = Mpr(
		origin,
		self.support,
		direction
	)
	
	return hitT, origin + direction * hitT, hitNormal
end

return SupportHull