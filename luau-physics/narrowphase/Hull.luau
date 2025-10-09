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
	normal: vector,
}

export type Obj = {
	-- used for querying gjk/sat
	support: Support,

	-- center of mass
	center: vector,

	-- list of verticies
	verticies: { vector },

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
	center: vector,

	-- list of verticies
	verticies: { vector },

	-- edge and normal
	normals: { [number]: number },

	-- edges are defined are 2 byte numbers
	edges: { number },

	-- list of faces, `w` = p.n
	faces: { Face },	
	
	-- base shape
	ref: Obj,
}, Hull))

local function empty(v: vector): vector
	-- empty support vector for
	-- optimization purposes
	return vector.zero
end

local function areArcsIntersecting(
	a: vector,
	b: vector,
	c: vector,
	d: vector
): boolean
	-- this is another separating axis test, but now we check
	-- if two lines on a plane collide
	
	local bxa = vector.cross(b,a)
	local dxc = vector.cross(d,c)
	
	-- checks if the arc spanning a->b and c->d are
	-- colliding. if it is, then we can test this pair
	local cba = vector.dot(c, bxa)
	local dba = vector.dot(d, bxa)
	local adc = vector.dot(a, dxc)
	local bdc = vector.dot(b, dxc)
	
	return cba*dba <= 0.0 and adc*bdc <= 0.0 and 0.0 <= cba*bdc
end

local function raycastTriangle(
	tri: { vector },
	origin: vector,
	direction: vector
): (number, vector, vector)
	-- https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
	local a = tri[1]
	local b = tri[2]
	local c = tri[3]
	
	local ab = b - a
	local ac = c - a

	local rayAC = vector.cross(direction, ac)
	local det = vector.dot(ab, rayAC)
	if math.abs(det) < EPSILON then
		-- the ray is parallel to the triangle
		return 1, vector.zero, vector.zero
	end

	local invDet = 1.0 / det
	local s = origin - a
	local u = invDet * vector.dot(s, rayAC)

	if (u > 1 and math.abs(u - 1) > EPSILON)
	or (u < 0 and math.abs(u) > EPSILON) then
		-- first case, u is out of bounds
		return 1, vector.zero, vector.zero
	end

	local se = vector.cross(s, ab)
	local v = invDet*vector.dot(direction, se)
	if (u + v > 1 and math.abs(u + v - 1) > EPSILON)
	or (v < 0 and math.abs(v) > EPSILON) then
		-- second case, v is out of bounds
		return 1, vector.zero, vector.zero
	end

	local t = invDet*vector.dot(ac, se)	
	if t > EPSILON then
		-- the ray hits the triangle
		return t,
			origin+direction*t,
			vector.normalize(vector.cross(ab, ac))
	else
		-- the ray does not hit the triangle
		return 1, vector.zero, vector.zero
	end
end

local function createCentroid(verticies: { vector }): vector
	-- calculate center from point cloud
	local centroid = vector.zero
	for i, vertex in verticies do
		centroid += vertex
	end

	return centroid / #verticies
end

local function castVector(vec: Vector3): vector
	-- roblox should just make a cast here, but we have
	-- to do this explicitly
	return vector.create(vec.X, vec.Y, vec.Z)
end

local function createSupportQuery(verticies: { vector }): Support
	return function(direction: vector)
		-- get the vertex with the highest dot
		-- product with the direction
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in verticies do
			local dot = vector.dot(point, direction)
			if bestDot < dot then
				bestDot = dot
				bestItr = i
			end
		end

		return verticies[bestItr]
	end
end

local function queryEdge(
	verticies: { vector },
	edge: number
): (vector, vector)
	-- query edges of the verticies please
	return verticies[bit32.band(edge, 0xFFFF)],
		verticies[bit32.rshift(edge, 16)]
end

function Hull.queryEdge(hullA: Hull, edge: number): (vector, vector)
	-- query edges of the verticies please
	return hullA.verticies[bit32.band(edge, 0xFFFF)],
		hullA.verticies[bit32.rshift(edge, 16)]
end

local function twin(edge: number): number
	local a = bit32.rshift(edge, 16)
	local b = bit32.band(edge, 0xFFFF)
	
	return bit32.lshift(b, 16) + a
end

function Hull.queryEdgeDirections(hullA: Hull, hullB: Hull): (vector, number, number, number)
	-- check all possible edge combinations of A and B as potential separating
	-- axes. only happens in 3D
	local facesA = hullA.faces
	local facesB = hullB.faces
	
	local vertsA = hullA.verticies
	local vertsB = hullB.verticies
	
	local normsA = hullA.normals
	local normsB = hullB.normals

	local bestDirection = vector.zero
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
			local axis = vector.cross(dirAB, dirCD)
			if vector.dot(axis, axis) < 1e-3 then
				-- these two edges are parallel
				continue
			elseif vector.dot(axis, pointA - pointC) < 0.0 then
				-- flip
				axis = -axis
			end

			-- we remove points whose arcs
			-- don't collide, as they are not
			-- a valid separating axis
			if not areArcsIntersecting(
				normalA,
				normalB,
				-normalC,
				-normalD
			) then
				continue
			end

			-- check the support point of B onto A
			local direction = vector.normalize(axis)
			local distance = vector.dot(pointC - pointA, direction)
						
			if bestDistance < distance then
				-- mark our direction and edges
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

	-- it is known for a convex shape, S(face.normal) lies on
	-- the current face
	local vertsA = hullA.verticies	
	for i, face in hullA.faces do
		-- query the support point
		local normal = face.normal
		local vertexA = vertsA[face.verticies[1]]
		local vertexB = hullB.support(-normal)
		
		local distance = vector.dot(vertexB - vertexA, normal)
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
	size: vector
)
	-- clones a base hull, good for parts
	local ref = self.ref

	local centroid = vector.zero
	local verticies = self.verticies
	local faces = self.faces
	
	for i, vertex in ref.verticies do
		-- create our own vertex since it changed
		local point = castVector(cframe:PointToWorldSpace(vertex * size))
		centroid += point
		verticies[i] = point
	end

	local invSize = vector.one / size
	for i, face in ref.faces do
		-- make a new normal but preserve
		-- verticies table
		local normal = castVector(cframe:VectorToWorldSpace(face.normal * invSize))
		faces[i].normal = vector.normalize(normal)
	end
	
	self.center = castVector(cframe.Position)
end

function Hull.raycast(
	self: Hull,
	origin: vector,
	direction: vector
): (number, vector, vector)
	local verticies = self.verticies
	local temporary = table.create(3, vector.zero)

	local bestNormal = vector.zero
	local bestT = 1.0

	for i, face in self.faces do
		local verts = face.verticies
		for j = 1, #verts - 2 do
			-- we assume that the convex face creates
			-- a triangle fan, so we just keep on testing
			-- if our ray hits the triangle
			temporary[1] = verticies[verts[j + 0]]
			temporary[2] = verticies[verts[j + 1]]
			temporary[3] = verticies[verts[j + 2]]

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
	size: vector
): Hull
	local faces = {}
	local refFaces = ref.faces
	for i = 1, #ref.faces do
		faces[i] = {
			verticies = refFaces[i].verticies,
			normal = vector.zero,
		}
	end
	
	local verticies = table.create(#ref.verticies, vector.zero) :: { vector }
	local self = setmetatable({
		ref = ref,
		support = empty,
		center = castVector(cframe.Position),
		verticies = verticies,
		normals = ref.normals,
		edges = ref.edges,
		faces = faces :: { Face },
	}, Hull)
	
	function self.support(direction: vector): vector
		-- get the vertex with the highest dot
		-- product with the direction
		local bestDot = -math.huge
		local bestItr = 0

		for i, point in verticies do
			local dot = vector.dot(point, direction)
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
	-- creates a hull from an obj so yeah
	local edgeNormals = {} :: { [number]: number }
	local verticies = {} :: { vector }
	local normals = {} :: { vector }
	local edges = {} :: { number }
	local faces = {} :: { Face }

	for i, line in obj:split('\n') do
		local token = line:gmatch("[^%s/]+")
		local command = token()

		if command == 'f' then
			-- face
			local normal = vector.zero
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
			normal = vector.normalize(normal / length)
			
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
			table.insert(verticies, vector.create(
				tonumber(token()) :: number,
				tonumber(token()) :: number,
				tonumber(token()) :: number
			))
		elseif command == 'vn' then
			-- vertex normal
			table.insert(normals, vector.create(
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
	return Gjk.isColliding(
		self.support,
		with.support,
		with.center - self.center
	)
end

return Hull
