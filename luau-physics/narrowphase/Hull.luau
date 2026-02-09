--!native
--!strict

local EPSILON = 1e-3

local Support = require("./Support")
local Mat3 = require("../Mat3")

local Hull = {}
Hull.__index = Hull

type Mat3 = Mat3.Mat3

-- canonical inertia tensor
local INERTIA_CANONICAL = Mat3.new(
	2.0 / 120.0, 1.0 / 120.0, 1.0 / 120.0,
	1.0 / 120.0, 2.0 / 120.0, 1.0 / 120.0,
	1.0 / 120.0, 1.0 / 120.0, 2.0 / 120.0
)

export type Support = Support.Support
export type Face = {
	verticies: { number },
	normal: Vector3,
}

export type Obj = {
	-- used for querying gjk/sat
	support: Support,

	-- center of mass
	center: Vector3,

	-- list of verticies
	verticies: { Vector3 },
	
	-- center of mass
	centroid: Vector3,
	
	-- inertia tensor
	inertia: Vector3,
	
	-- mass
	mass: number,

	-- edge and normal
	normals: { [number]: number },

	-- edges are defined are 2 byte numbers
	edges: { number },

	-- list of faces, `w` = p.n
	faces: { Face },
}

export type Hull = typeof(setmetatable({} :: {
	-- used for querying gjk/sat
	support: Support,

	-- center of mass
	center: Vector3,

	-- list of verticies
	verticies: { Vector3 },

	-- edge and normal
	normals: { [number]: number },

	-- edges are defined are 2 byte numbers
	edges: { number },

	-- list of faces, `w` = p.n
	faces: { Face },	

	-- base shape
	ref: Obj,
}, Hull))


local function getInertiaTensor(
	verticies: { Vector3 },
	faces: { Face },
	centroid: Vector3,
	mass: number
): Vector3
	local inertia = Mat3.fromVector(Vector3.zero)
	for i, face in faces do
		local faceVerts = face.verticies
		local v1 = verticies[faceVerts[1]] - centroid
		local v2 = verticies[faceVerts[2]] - centroid

		for j = 3, #faceVerts do
			local v3 = verticies[faceVerts[j]] - centroid

			-- Create a tetrahedron matrix
			local tetrahedron = Mat3.fromMatrix(v1, v2, v3)
			local det = Mat3.det(tetrahedron)

			local covar = Mat3.matMul(tetrahedron, Mat3.matMul(INERTIA_CANONICAL, Mat3.transpose(tetrahedron)))
			inertia = Mat3.matAdd(inertia, covar)

			-- Make sure that v2 = verts[j - 1]
			v2 = v3
		end
	end

	-- Construct our final inertia tensor
	return Vector3.new(
		inertia[1][1] - INERTIA_CANONICAL[1][1],
		inertia[2][2] - INERTIA_CANONICAL[2][2],
		inertia[3][3] - INERTIA_CANONICAL[3][3]
	)
end

local function getCenterVolume(
	verticies: { Vector3 },
	faces: { Face }
): (Vector3, number)
	-- First, calculate the center point
	-- of the entire shape
	local centroid = Vector3.zero
	for i, face in faces do
		local faceVerts = face.verticies
		local center = Vector3.zero

		for j, vertex in faceVerts do
			center += verticies[vertex]
		end

		centroid += center / #faceVerts
	end

	-- Gathered our centroid!
	centroid /= #faces

	-- Go through the triangle and
	-- create a tetrahedron from the center
	-- and the triangles of our mesh
	local totCenter = Vector3.zero
	local totVolume = 0.0

	for i, face in faces do
		local faceVerts = face.verticies
		local v1 = verticies[faceVerts[1]]
		local v2 = verticies[faceVerts[2]]

		for j = 3, #faceVerts do
			local v3 = verticies[faceVerts[j]]

			-- Get the verticies and then we
			-- calculate the volume of the tetrahedron
			local volume = (v1 - centroid):Dot((v2 - centroid):Cross(v3 - centroid))
			local center = (v1 + v2 + v3 + centroid) * 0.25

			totVolume += volume
			totCenter += center * volume

			-- Make sure that v2 = verts[j - 1]
			v2 = v3
		end
	end

	return totCenter / totVolume, totVolume / 6.0
end

local function empty(v: Vector3): Vector3
	-- Empty support vector for
	-- optimization purposes
	return Vector3.zero
end

local function raycastTriangle(
	tri: { Vector3 },
	origin: Vector3,
	direction: Vector3
): (number, Vector3, Vector3)
	-- https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
	local a = tri[1]
	local b = tri[2]
	local c = tri[3]
	
	local ab = b - a
	local ac = c - a

	local rayAC = direction:Cross(ac)
	local det = ab:Dot(rayAC)
	if math.abs(det) < EPSILON then
		-- The ray is parallel to the triangle
		return 1, Vector3.zero, Vector3.zero
	end

	local invDet = 1.0 / det
	local s = origin - a
	local u = invDet * s:Dot(rayAC)

	if (u > 1 and math.abs(u - 1) > EPSILON)
	or (u < 0 and math.abs(u) > EPSILON) then
		-- First case, u is out of bounds
		return 1, Vector3.zero, Vector3.zero
	end

	local se = s:Cross(ab)
	local v = invDet*direction:Dot(se)
	if (u + v > 1 and math.abs(u + v - 1) > EPSILON)
	or (v < 0 and math.abs(v) > EPSILON) then
		-- Second case, v is out of bounds
		return 1, Vector3.zero, Vector3.zero
	end

	local t = invDet*ac:Dot(se)	
	if t > EPSILON then
		-- The ray hits the triangle
		return t,
			origin+direction*t,
			ab:Cross(ac).Unit
	else
		-- the ray does not hit the triangle
		return 1, Vector3.zero, Vector3.zero
	end
end

local function createCentroid(verticies: { Vector3 }): Vector3
	-- Calculate center from point cloud
	local centroid = Vector3.zero
	for i, vertex in verticies do
		centroid += vertex
	end

	return centroid / #verticies
end

local function createSupportQuery(verticies: { Vector3 }): Support
	return function(direction: Vector3): Vector3
		-- get the vertex with the highest dot
		-- product with the direction
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in verticies do
			local dot = point:Dot(direction)
			if bestDot < dot then
				bestDot = dot
				bestItr = i
			end
		end

		return verticies[bestItr]
	end
end

function Hull.queryEdge(hullA: Hull, edge: number): (Vector3, Vector3)
	-- Query edges of the verticies
	return hullA.verticies[bit32.band(edge, 0xFFFF)],
		hullA.verticies[bit32.rshift(edge, 16)]
end

function Hull.update(
	self: Hull,
	cframe: CFrame,
	size: Vector3
)
	-- Clones a base hull, good for parts
	local ref = self.ref

	local centroid = Vector3.zero
	local verticies = self.verticies
	local faces = self.faces
	
	for i, vertex in ref.verticies do
		-- Create our own vertex since it changed
		local point = cframe:PointToWorldSpace(vertex * size)
		centroid += point
		verticies[i] = point
	end

	local invSize = Vector3.one / size
	for i, face in ref.faces do
		-- Make a new normal but preserve
		-- verticies table
		local normal = cframe:VectorToWorldSpace(face.normal * invSize)
		faces[i].normal = normal.Unit
	end
	
	self.center = cframe.Position
end

function Hull.raycast(
	self: Hull,
	origin: Vector3,
	direction: Vector3
): (number, Vector3, Vector3)
	local verticies = self.verticies
	local temporary = table.create(3, Vector3.zero)

	local bestNormal = Vector3.zero
	local bestT = 1.0

	for i, face in self.faces do
		local verts = face.verticies
		local numVerts = #verts

		for j = 1, numVerts - 1, 2 do
			-- We assume that the convex face creates
			-- a triangle fan, so we just keep on testing
			-- if our ray hits the triangle.
			temporary[3] = verticies[verts[((j + 1) % numVerts) + 1]]
			temporary[2] = verticies[verts[(j % numVerts) + 1]]
			temporary[1] = verticies[verts[j]]

			local t, position, normal = raycastTriangle(
				temporary,
				origin,
				direction
			)

			if t < bestT then
				bestNormal = normal
				bestT = t
			end
		end
	end

	return bestT, origin + direction * bestT, bestNormal
end

function Hull.new(
	ref: Obj,
	cframe: CFrame,
	size: Vector3
): Hull
	local faces = {}
	local refFaces = ref.faces
	for i = 1, #ref.faces do
		faces[i] = {
			verticies = refFaces[i].verticies,
			normal = Vector3.zero,
		}
	end
	
	local verticies = table.create(#ref.verticies, Vector3.zero) :: { Vector3 }
	local self = setmetatable({
		ref = ref,
		support = empty,
		center = cframe.Position,
		verticies = verticies,
		normals = ref.normals,
		edges = ref.edges,
		faces = faces :: { Face },
	}, Hull)
	
	function self.support(direction: Vector3): Vector3
		-- Get the vertex with the highest dot
		-- product with the direction.
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in verticies do
			local dot = point:Dot(direction)
			if bestDot < dot then
				bestDot = dot
				bestItr = i
			end
		end

		return verticies[bestItr]
	end
	
	self:update(cframe, size)
	return self
end

function Hull.fromObj(obj: string): Obj
	-- Create a hull from an OBJ
	local edgeNormals = {} :: { [number]: number }
	local verticies = {} :: { Vector3 }
	local normals = {} :: { Vector3 }
	local edges = {} :: { number }
	local faces = {} :: { Face }

	for i, line in obj:split('\n') do
		local token = line:gmatch("[^%s/]+")
		local command = token()

		if command == 'f' then
			-- face
			local normal = Vector3.zero
			local verts = {} :: { number }
			
			repeat
				local vertex = token()
				if not vertex then
					-- early break incase
					break
				end

				local _uv = tonumber(token())
				normal += normals[tonumber(token()) :: number]
				table.insert(verts, tonumber(vertex) :: number)
			until not vertex

			local faceId = #faces + 1
			local length = #verts
			normal = (normal / length).Unit
			
			local k = length
			for j = 1, length do
				-- add an edge
				local edge = bit32.lshift(verts[k], 16) + verts[j]
				edgeNormals[edge] = faceId
				
				table.insert(edges, edge)
				k = j
			end

			table.insert(faces, {
				verticies = verts,
				normal = normal,
			})
		elseif command == 'v' then
			-- vertex
			table.insert(verticies, Vector3.new(
				tonumber(token()) :: number,
				tonumber(token()) :: number,
				tonumber(token()) :: number
			))
		elseif command == 'vn' then
			-- vertex normal
			table.insert(normals, Vector3.new(
				tonumber(token()) :: number,
				tonumber(token()) :: number,
				tonumber(token()) :: number
			))
		end
	end
	
	local centroid, mass = getCenterVolume(verticies, faces)
	local inertia = getInertiaTensor(verticies, faces, centroid, mass)
	
	return table.freeze({
		support = createSupportQuery(verticies),
		center = createCentroid(verticies),
		normals = edgeNormals,
		
		centroid = centroid,
		inertia = inertia,
		mass = mass,
		
		verticies = verticies,
		edges = edges,
		faces = faces,
	}) :: Obj
end

return Hull