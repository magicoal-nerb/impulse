--!native
--!strict

local EPSILON = 1e-3

local Support = require("./Support")
local Gjk = require("./Gjk")

local Hull = {}
Hull.__index = Hull

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

local function empty(v: Vector3): Vector3
	-- empty support vector for
	-- optimization purposes
	return Vector3.zero
end

local function areArcsIntersecting(
	a: Vector3,
	b: Vector3,
	c: Vector3,
	d: Vector3
): boolean
	-- this is another separating axis test, but now we check
	-- if two lines on a plane collide
	
	local bxa = b:Cross(a)
	local dxc = d:Cross(c)
	
	-- checks if the arc spanning a->b and c->d are
	-- colliding. if it is, then we can test this pair
	local cba = c:Dot(bxa)
	local dba = d:Dot(bxa)
	local adc = a:Dot(dxc)
	local bdc = b:Dot(dxc)
	
	return cba*dba <= 0.0 and adc*bdc <= 0.0 and 0.0 <= cba*bdc
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
		-- the ray is parallel to the triangle
		return 1, Vector3.zero, Vector3.zero
	end

	local invDet = 1.0 / det
	local s = origin - a
	local u = invDet * s:Dot(rayAC)

	if (u > 1 and math.abs(u - 1) > EPSILON)
	or (u < 0 and math.abs(u) > EPSILON) then
		-- first case, u is out of bounds
		return 1, Vector3.zero, Vector3.zero
	end

	local se = s:Cross(ab)
	local v = invDet*direction:Dot(se)
	if (u + v > 1 and math.abs(u + v - 1) > EPSILON)
	or (v < 0 and math.abs(v) > EPSILON) then
		-- second case, v is out of bounds
		return 1, Vector3.zero, Vector3.zero
	end

	local t = invDet*ac:Dot(se)	
	if t > EPSILON then
		-- the ray hits the triangle
		return t,
			origin+direction*t,
			ab:Cross(ac).Unit
	else
		-- the ray does not hit the triangle
		return 1, Vector3.zero, Vector3.zero
	end
end

local function createCentroid(verticies: { Vector3 }): Vector3
	-- calculate center from point cloud
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

local function queryEdge(
	verticies: { Vector3 },
	edge: number
): (Vector3, Vector3)
	-- query edges of the verticies please
	return verticies[bit32.band(edge, 0xFFFF)],
		verticies[bit32.rshift(edge, 16)]
end

function Hull.queryEdge(hullA: Hull, edge: number): (Vector3, Vector3)
	-- query edges of the verticies please
	return hullA.verticies[bit32.band(edge, 0xFFFF)],
		hullA.verticies[bit32.rshift(edge, 16)]
end

local function twin(edge: number): number
	local a = bit32.rshift(edge, 16)
	local b = bit32.band(edge, 0xFFFF)
	
	return bit32.lshift(b, 16) + a
end

function Hull.queryEdgeDirections(hullA: Hull, hullB: Hull): (Vector3, number, number, number)
	-- check all possible edge combinations of A and B as potential separating
	-- axes. only happens in 3D
	local facesA = hullA.faces
	local facesB = hullB.faces
	
	local vertsA = hullA.verticies
	local vertsB = hullB.verticies
	
	local normsA = hullA.normals
	local normsB = hullB.normals

	local bestDirection = Vector3.zero
	local bestDistance = -math.huge
	local bestEdgeA = 0
	local bestEdgeB = 0

	local origin = hullA.center
	for i, edgeA in hullA.edges do
		-- get the edge from A
		local normalB = facesA[normsA[twin(edgeA)]].normal
		local normalA = facesA[normsA[edgeA]].normal
		
		local pointA, pointB = queryEdge(vertsA, edgeA)
		local dirAB = pointB - pointA

		for j, edgeB in hullB.edges do
			-- get the edge from B
			local normalD = facesB[normsB[twin(edgeB)]].normal
			local normalC = facesB[normsB[edgeB]].normal
			
			local pointC, pointD = queryEdge(vertsB, edgeB)
			local dirCD = pointD - pointC

			-- compute the separating axis
			local axis = dirAB:Cross(dirCD)
			if axis:Dot(axis) < 1e-3 then
				-- these two edges are parallel
				continue
			elseif axis:Dot(pointA - pointC) < 0.0 then
				-- flip
				axis = -axis
			end

			-- We remove points whose arcs
			-- don't collide, as we know it will just
			-- repeat!!
			if not areArcsIntersecting(
				normalA,
				normalB,
				-normalC,
				-normalD
			) then
				continue
			end

			-- Check the support point of B onto A
			local direction = axis.Unit
			local distance = (pointC - pointA):Dot(direction)
						
			if bestDistance < distance then
				-- Mark our direction and edges
				bestDirection = direction
				bestDistance = distance
				bestEdgeA = edgeA
				bestEdgeB = edgeB
			end
		end
	end

	assert(bestEdgeA ~= 0, "no edge found for a")
	assert(bestEdgeB ~= 0, "no edge found for b")

	return bestDirection, bestDistance,
		bestEdgeA, bestEdgeB
end

function Hull.queryFaceDirections(hullA: Hull, hullB: Hull): (Face, number)
	local bestDistance = -math.huge
	local bestFace = 0

	-- It is known for a convex shape, support(face.normal) lies on
	-- the current face
	local vertsA = hullA.verticies	
	for i, face in hullA.faces do
		-- query the support point
		local normal = face.normal
		local vertexA = vertsA[face.verticies[1]]
		local vertexB = hullB.support(-normal)
		
		local distance = (vertexB - vertexA):Dot(normal)
		if bestDistance < distance then
			bestDistance = distance
			bestFace = i
		end
	end

	-- return a record of the current face
	assert(bestFace ~= 0, "hull had no faces!")
	return hullA.faces[bestFace], bestDistance
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
		-- create our own vertex since it changed
		local point = cframe:PointToWorldSpace(vertex * size)
		centroid += point
		verticies[i] = point
	end

	local invSize = Vector3.one / size
	for i, face in ref.faces do
		-- make a new normal but preserve
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
	
	return table.freeze({
		support = createSupportQuery(verticies),
		center = createCentroid(verticies),
		normals = edgeNormals,
		
		verticies = verticies,
		edges = edges,
		faces = faces,
	})
end

function Hull.collides(self: Hull, with: Hull): boolean
	-- Helper function to check if two objects
	-- collide through GJK
	return Gjk.isColliding(
		self.support,
		with.support,
		with.center - self.center
	)
end

return Hull
