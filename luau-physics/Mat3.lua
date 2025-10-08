--!strict
--!native

-- used to do matrix operations

local Mat3 = {}
Mat3.__index = Mat3

export type Mat3 = typeof(setmetatable({} :: {{ number }}, Mat3))

Mat3.zero = setmetatable({
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0},
}, Mat3)

Mat3.identity = setmetatable({
	{1, 0, 0},
	{0, 1, 0},
	{0, 0, 1},
}, Mat3)

function Mat3.fromCFrame(cframe: CFrame): Mat3
	local _, _, _, r00, r01, r02, r10, r11, r12, r20, r21, r22 = cframe:GetComponents()
	return setmetatable({
		{ r00, r01, r02 },
		{ r10, r11, r12 },
		{ r20, r21, r22 },
	}, Mat3)
end

function Mat3.fromVector(vec: vector): Mat3
	return setmetatable({
		{vec.x, 0, 0},
		{0, vec.y, 0},
		{0, 0, vec.z},
	}, Mat3)
end

function Mat3.outerProduct(vec: vector): Mat3
	-- v^T * v
	local x = vec.x
	local y = vec.y
	local z = vec.z
	
	return setmetatable({
		{x*x, x*y, x*z},
		{y*x, y*y, y*z},
		{z*x, z*y, z*z},
	}, Mat3)
end

function Mat3.numMul(self: Mat3, x: number): Mat3
	-- scalar multiplication
	return setmetatable({
		{self[1][1] * x, self[1][2] * x, self[1][3] * x},
		{self[2][1] * x, self[2][2] * x, self[2][3] * x},
		{self[3][1] * x, self[3][2] * x, self[3][3] * x},
	}, Mat3)
end

function Mat3.vecMul(self: Mat3, what: vector): vector
	-- multiply a mat3 by a vector
	local x = what.x
	local y = what.y
	local z = what.z
	return vector.create(
		self[1][1] * x + self[2][1] * y + self[3][1] * z,
		self[1][2] * x + self[2][2] * y + self[3][2] * z,
		self[1][3] * x + self[2][3] * y + self[3][3] * z
	)
end

function Mat3.matMul(self: Mat3, what: Mat3): Mat3
	-- we're a matrix
	local product = setmetatable({
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	}, Mat3)
	
	for i = 1, 3 do
		for j = 1, 3 do
			product[i][j] = self[i][1] * what[1][j]
				+ self[i][2] * what[2][j]
				+ self[i][3] * what[3][j]
		end
	end

	return product :: Mat3
end

function Mat3.vecAdd(self: Mat3, what: vector): Mat3
	-- it's a vector
	local x = what.x
	local y = what.y
	local z = what.z
	return setmetatable({
		{self[1][1] + x, self[1][2], self[1][3]},
		{self[2][1], self[2][2] + y, self[2][3]},
		{self[3][1], self[3][2], self[3][3] + z},
	}, Mat3)
end

function Mat3.matSub(self: Mat3, what: Mat3): Mat3
	-- we're a matrix
	local product = setmetatable({
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	}, Mat3)
	
	for i = 1, 3 do
		for j = 1, 3 do
			product[i][j] = self[i][j] - what[i][j]
		end
	end

	return product
end

function Mat3.matAdd(self: Mat3, what: Mat3): Mat3
	-- we're a matrix
	local product = setmetatable({
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	}, Mat3)
	
	for i = 1, 3 do
		for j = 1, 3 do
			product[i][j] = self[i][j] + what[i][j]
		end
	end

	return product
end

function Mat3.inverse(self: Mat3): Mat3
	-- A^-1
	local i11 = self[2][2] * self[3][3] - self[2][3] * self[3][2]
	local i21 = self[2][3] * self[3][1] - self[2][1] * self[3][3]
	local i31 = self[2][1] * self[3][2] - self[2][2] * self[3][1]
	
	-- precompute determinant
	local det = self[1][1] * i11
		+ self[1][2] * i21
		+ self[1][3] * i31
	
	if math.abs(det) < 1e-3 then
		error("lol singular matrix")
	end
	
	-- calculates it through cofactors + determinant expansion
	-- i dont like it
	local out = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	}
	
	local invDet = 1.0 / det
	out[1][1] = i11 * invDet
	out[1][2] = (self[1][3] * self[3][2] - self[1][2] * self[3][3]) * invDet
	out[1][3] = (self[1][2] * self[2][3] - self[1][3] * self[2][2]) * invDet
	out[2][1] = i21 * invDet
	out[2][2] = (self[1][1] * self[3][3] - self[1][3] * self[3][1]) * invDet
	out[2][3] = (self[1][3] * self[2][1] - self[1][1] * self[2][3]) * invDet
	out[3][1] = i31 * invDet
	out[3][2] = (self[1][2] * self[3][1] - self[1][1] * self[3][2]) * invDet
	out[3][3] = (self[1][1] * self[2][2] - self[1][2] * self[2][1]) * invDet
	
	return setmetatable(out, Mat3)
end

function Mat3.new(
	a: number, b: number, c: number,
	d: number, e: number, f: number,
	g: number, h: number, i: number
): Mat3
	return setmetatable({
		{ a, b, c },
		{ d, e, f },
		{ g, h, i },
	}, Mat3)
end

function Mat3.toCFrame(self: Mat3): CFrame
	-- converts to a cframe for slightly faster
	-- computation
	return CFrame.new(
		0, 0, 0,
		self[1][1], self[1][2], self[1][3],
		self[2][1], self[2][2], self[2][3],
		self[3][1], self[3][2], self[3][3]
	)
end

function Mat3.transpose(self: Mat3): Mat3
	-- A^T
	return setmetatable({
		{ self[1][1], self[2][1], self[3][1]},
		{ self[1][2], self[2][2], self[3][2]},
		{ self[1][3], self[2][3], self[3][3]}
	}, Mat3)
end

function Mat3.__mul(self: Mat3, what: number | Mat3): Mat3
	if typeof(what) == "number" then
		-- scalar multiply
		return Mat3.numMul(self, what)
	else
		-- matrix multiply
		return Mat3.matMul(self, what)
	end
end

Mat3.__sub = Mat3.matSub
Mat3.__add = Mat3.matAdd

return Mat3