--!strict

--[[
	The EPA algorithm is an addition to the GJK algorithm where
	we can extract the minimal translation vector from a terminating
	GJK simplex. This is possible because(in my case atleast) GJK
	enclosing a point results in the simplex being a tetrahedron.

	The job of the EPA algorithm is to keep on sampling points that
	will eventually hit the surface of the shape. If we found a triangle
	closest to the origin that is on the surface, then we give back that triangle's
	position, and its normal is our MTV normal!
]]

type Support = (Vector3) -> (Vector3)

local J = { 2, 3, 1 }

local function encode(a: number, b: number): number
	-- Encodes the verticies
	return bit32.lshift(a, 16) + b
end

local function decode(a: number): (number, number)
	-- Gives back the verticies
	return bit32.rshift(a, 16), bit32.band(a, 0xFFFF)
end

local function dual(a: number): number
	-- Gets the dual of the edge
	return bit32.lshift(bit32.band(a, 0xFFFF), 16) + bit32.rshift(a, 16)
end

return function(
	simplex: { Vector3 },
	suppA: Support,
	suppB: Support
): (Vector3, number)
	assert(#simplex == 4, "simplex should be a tetrahedron")
	
	local normals = table.create(4, Vector3.zero)
	local mesh: {{ number }} = {
		-- Create a basic tetrahedron mesh
		-- from our GJK termination condition
		{1, 2, 4},
		{1, 3, 4},
		{2, 3, 4},
		{1, 2, 3},
	}
	
	for i, triangle in mesh do
		local a = simplex[triangle[1]]
		local b = simplex[triangle[2]]
		local c = simplex[triangle[3]]
		
		local normal = (c - a):Cross(b - a)
		if normal:Dot(a) < 0 then
			-- Flip normal if the vertex
			-- is pointing outside of it
			normal *= -1
		end
		
		normals[i] = normal.Unit
	end
	
	for i = 1, 8 do
		-- First, we check for the closest triangle
		-- that encloses this simplex
		local bestDistance = normals[1]:Dot(simplex[mesh[1][1]])
		local bestTriangle = 1
		
		for j = 2, #normals do
			local normal = normals[j]
			local point = simplex[mesh[j][1]]
			
			local distance = point:Dot(normal)
			if bestDistance > distance then
				bestDistance = distance
				bestTriangle = j
			end
		end
		
		-- If the current triangle is on the surface
		-- of the mesh, then we can safely terminate
		local normal = normals[bestTriangle]
		local point = suppA(normal) - suppB(-normal)
		
		local distance = math.abs(normal:Dot(point) - bestDistance)
		if math.abs(distance) < 1e-2 then
			-- We return because the closest triangle
			-- in our mesh is on the surface of the shape
			return -normal, point:Dot(normal)
		end
		
		-- Otherwise, we will add this new point by creating
		-- a triangle fan around the point. We do this by removing
		-- triangles facing it, and then triangulating around the boundary
		-- edges
		local newNormals: { Vector3 } = {}
		local newMesh: {{ number }} = {}
		local edgelist: { number } = {}
		
		for j, triangle in mesh do
			local a = triangle[1]
			if normals[j]:Dot(normal) <= 0 then
				-- It is behind the simplex point, so we
				-- can keep this
				table.insert(newMesh, triangle)
				table.insert(newNormals, normals[j])
				
				continue
			end
			
			-- Otherwise, add its entries to the edgelist
			local b = triangle[2]
			local c = triangle[3]

			edgelist[encode(a, b)] = 1
			edgelist[encode(b, c)] = 1
			edgelist[encode(c, a)] = 1
		end
		
		-- Repair by building a triangle fan
		-- around this discovered point
		table.insert(simplex, point)
		
		local id = #simplex
		for edge in edgelist do
			if edgelist[dual(edge)] then
				-- This cannot be a boundary edge
				-- because the dual of it exists
				continue
			end
			
			local a, b = decode(edge)
			local va = simplex[a]
			local vb = simplex[b]
			
			local normal = (point - va):Cross(vb - va)
			if normal:Dot(normal) < 1e-3 then
				-- The triangle is degenerate, so do not
				-- use it.
				continue
			elseif normal:Dot(point) < 0 then
				-- The triangle is pointing away the point
				table.insert(newNormals, -normal.Unit)
			else
				-- The triangle is pointing towards the point
				table.insert(newNormals, normal.Unit)
			end
			
			table.insert(newMesh, { a, b, id })
		end
		
		-- Update our current state
		normals = newNormals
		mesh = newMesh
	end
	
	-- Otherwise, we check for the closest triangle
	-- that encloses this simplex
	local bestDistance = normals[1]:Dot(simplex[mesh[1][1]])
	local bestTriangle = 1

	for j = 2, #normals do
		local normal = normals[j]
		local point = simplex[mesh[j][1]]

		local distance = point:Dot(normal)
		if bestDistance > distance then
			bestDistance = distance
			bestTriangle = j
		end
	end

	-- We will assume this triangle is on the surface
	-- in order to terminate the algorithm. This may not be
	-- entirely correct, but we can fix it next tick.

	local normal = normals[bestTriangle]
	local point = suppA(normal) - suppB(-normal)
	return -normal, point:Dot(normal)
end