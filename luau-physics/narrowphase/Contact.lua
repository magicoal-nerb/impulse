--!native
--!nocheck
--!strict

local Body = require("../Body")

--[[
	Jv + b = 0
	M(v_2 - v_1) + b = L*lambda

	L is J^T because it is parallel to the constraint
	plane

	M = [ m_a 0 0 0 ]
	    [ 0 i_a 0 0 ]
	    [ 0 0 m_b 0 ]
	    [ 0 0 0 i_b ]
		    
	M(v_2 - v_1) = J^T * lambda
	v_2 = M^-1 (J^T * lambda) + v_1
	
	v = v_2 (new state)
	J(M^-1 * J^T * lambda + v_1) + b = 0
	J * M^-1 * J^T * lambda = -(J*v_1 + b)
	
	lambda = -(J*v_1 + b) / (J * M^-1 * J^T)
	M_eff = 1.0 / (J * M^-1 * J^T)
]]

local FLAG_NORMAL = 0x1
local FLAG_TANGENT = 0x2

local Jacobian = {}
local Contact = {}
Contact.__index = Contact

export type Body = Body.Body
export type Contact = typeof(setmetatable({} :: {
	bodyA: Body,
	bodyB: Body,
	
	-- normal
	friction: number,
	normal: vector,
	
	-- offsets
	rA: vector,
	rB: vector,
	
	-- jacobians
	jN: Jacobian,
	jT: Jacobian,
	jB: Jacobian,
}, Contact))

export type Jacobian = {	
	-- state variables
	flag: number,
	jva: vector,
	jwa: vector,
	jvb: vector,
	jwb: vector,
	
	-- final bias
	bias: number,
	effMass: number,
	totLambda: number,
}

local function castVec3(vec: vector): Vector3
	-- casts a vector to a vector3 >_>
	return Vector3.new(vec.x, vec.y, vec.z)
end

local function solveJacobian(
	self: Jacobian,
	contact: Contact,
	dt: number
)
	-- lambda = -(J*v_1 + b) / (J * M^-1 * J^T)
	-- calculate J*v_1
	local a = contact.bodyA
	local b = contact.bodyB
	local jv = vector.dot(self.jva, a.linearVelocity)
		+ vector.dot(self.jwa, a.angularVelocity)
		+ vector.dot(self.jvb, b.linearVelocity)
		+ vector.dot(self.jwb, b.angularVelocity)

	-- ensure that sum lambda_i >= 0, cannot clamp
	-- lambda on its own. lambda is the difference between
	-- the clamped and previous lambda
	local effMass = self.effMass
	
	local lambda = -(jv + self.bias) * effMass
	local previousLambda = self.totLambda
	if self.flag == FLAG_TANGENT then
		-- apply friction
		local friction = contact.friction * contact.jN.totLambda
		self.totLambda = math.clamp(self.totLambda + lambda, -friction, friction)
	else
		-- clamp lambda >= 0
		self.totLambda = math.max(self.totLambda + lambda, 0.0)
	end

	lambda = self.totLambda - previousLambda
	a.linearVelocity += a.invMass * self.jva * lambda
	a.angularVelocity += a.invInertia * self.jwa * lambda

	b.linearVelocity += b.invMass * self.jvb * lambda
	b.angularVelocity += b.invInertia * self.jwb * lambda
end

function Jacobian.new(
	a: Body,
	b: Body,
	
	normal: vector,
	rA: vector,
	rB: vector,
	flag: number,
	depth: number
): Jacobian
	-- state variables
	local jva = -normal
	local jwa = -vector.cross(rA, normal)
	local jvb = normal
	local jwb = vector.cross(rB, normal)
	
	--[[
		C: (b - a) . n >= 0
		C': ((v_b + w_b x r_b) - (v_a + w_a x r_a)). n >= 0
		C': ((v_b - r_b x w_b) - (v_a - r_a x w_a)). n >= 0

		know: a x b . c = c x a . b
		C': -(r_b x w_b).n + (r_a x w_a).n + (v_b - v_a).n
		C': -(n x r_b).w_b + (n x r_a).w_a + (v_b - v_a).n

		want: C' = JV + b
		J = [ -n^T (n x r_a)^T n^T -(n x r_b)^T ]
		J = [ -n^T -(r_a x n)^T n^T (r_b x n)^T ]
		V = [ v_a ]
			[ w_a ]
			[ v_b ]
			[ w_b ]
		    
		M_eff = 1.0 / (J * M^-1 * J^T => J_i.(M_i * J_i))
	]]
	
	local effMass = a.invMass
		+ vector.dot(a.invInertia * jwa, jwa)
		+ b.invMass
		+ vector.dot(b.invInertia * jwb, jwb)
	
	local bias = 0.0
	if flag == FLAG_NORMAL then
		--[[
			doing the bias term
			C: (b - a) . n >= 0
			
			baumgarte stabilization term(error = C)
			b = -B/dt * C
			b = -B/dt * (b-a).n
			
			restitution(dependant on C')
			c = C'.n
		]]

		local velocityConstraint = -a.linearVelocity
			- vector.cross(a.angularVelocity, rA)
			+ b.linearVelocity
			+ vector.cross(b.angularVelocity, rB)
		
		local projected = vector.dot(velocityConstraint, normal)
		local restitution = a.restitution * b.restitution
		local beta = a.beta * b.beta
		
		bias = restitution * projected - 120 * beta * depth
	end
	
	return {		
		-- state variables
		flag = flag,
		jva = jva,
		jwa = jwa,
		jvb = jvb,
		jwb = jwb,
		
		-- warm starting
		bias = bias,
		totLambda = 0.0,
		effMass = 1.0 / effMass,
	}
end

local VEC_X = vector.create(1, 0, 0)
local VEC_Z = vector.create(0, 0, 1)
local function getOrthogonal(normal: vector): (vector, vector)
	if math.abs(normal.y) > 0.99 then
		-- its up, so just pick the other two
		-- basis vectors
		return VEC_X, VEC_Z
	else
		-- tangent = y x normal
		-- bitangent = tangent x normal
		local tangent = vector.create(normal.z, 0, -normal.x)
		local bitangent = vector.cross(tangent, normal)
		return vector.normalize(tangent), vector.normalize(bitangent)
	end
end

function Contact.new(
	bodyA: Body,
	bodyB: Body,

	normal: vector,
	rA: vector,
	rB: vector,
	
	depth: number
): Contact
	local tangent, bitangent = getOrthogonal(normal)
	return setmetatable({
		bodyA = bodyA,
		bodyB = bodyB,
		
		friction = bodyA.friction * bodyB.friction,
		normal = normal,
		
		rA = rA,
		rB = rB,

		jN = Jacobian.new(
			-- state
			bodyA, bodyB,
			normal,
			rA, rB,
			
			-- props
			FLAG_NORMAL, depth
		),
		
		jT = Jacobian.new(
			-- state
			bodyA, bodyB,
			tangent,
			rA, rB,
			
			-- props
			FLAG_TANGENT, depth
		),
		
		jB = Jacobian.new(
			-- state
			bodyA, bodyB,
			bitangent,
			rA, rB,
			
			-- props
			FLAG_TANGENT, depth
		),
	}, Contact)
end

function Contact.solve(self: Contact, dt: number)
	-- solve the jacobian
	solveJacobian(self.jN, self, dt)
	solveJacobian(self.jT, self, dt)
	solveJacobian(self.jB, self, dt)
end

return Contact