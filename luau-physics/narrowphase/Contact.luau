--!native
--!strict

local Body = require("../Body")

--[[
	The high level overview of what I'm doing here is
	basically making our old state vector[ie. the previous velocities of A and B]
	no longer violate a constraint. A constraint is satisfied when it is 0, or if it
	is a conditional constraint, when that condition is satisfied.

	We derive the impulse model through the following:
		Jv + b = 0 
		M(v_2 - v_1) = L*lambda

	where J is the Jacobian matrix, and b is the baumgarte stabilization term.
	B accounts for positional error, and J defines what the constraints for the state
	velocities are.

	A key insight is that L = J^T, because Jv + b defines some hyperplane. And, we know
	that the direction of most increase in perpendicular to the plane, which is towards the
	normal of the plane. Thus, we get:
		(v_2 - v_1) = M^-1 * (J^T * lambda)
		v_2 = M^-1 * (J^T * lambda) + v_1

	After finding the next state vector, we essentially find a new state vector that
	does satisfy the constraint plane. From backsubstitution, we get:
		J(M^-1 * J^T * lambda + v_1) + b = 0
		lambda = -(Jv_1 + b) / (J * M^-1 * J^T)

	A crucial insight is that J a row vector that represents where we want our system state
	to move to(so it's 1 x N). Moreover, M^-1 is actually in the inverse mass matrix, ie:
		M = [ m_a 0 0 0 ]
		    [ 0 i_a 0 0 ]
		    [ 0 0 m_b 0 ]
		    [ 0 0 0 i_b ]
		    
	which is (N x N). Thus, if we did:
		J * (M^-1 * J^T) <=> [1 x N] x [(N x N) x (N x 1)]
		<=> [1 x 1], a scalar!

	We call this scalar the effective mass matrix.
	Since we can now compute lambda, our next state vector is:
		v_2 = M^-1 * (J^T * lambda) + v_1
]]

local FLAG_NORMAL = 0x1
local FLAG_TANGENT = 0x2

local ROLL_DAMPEN = 0.05

local Jacobian = {}
local Contact = {}
Contact.__index = Contact

export type Body = Body.Body
export type Contact = typeof(setmetatable({} :: {
	bodyA: Body,
	bodyB: Body,
	
	-- Normal
	friction: number,
	normal: Vector3,
	
	-- Offsets
	rA: Vector3,
	rB: Vector3,
	
	-- Relative
	relativeA: Vector3,
	relativeB: Vector3,
	
	-- Jacobians
	jN: Jacobian,
	jT: Jacobian,
	jB: Jacobian,
}, Contact))

export type Jacobian = {	
	-- State variables
	flag: number,
	jva: Vector3,
	jwa: Vector3,
	jvb: Vector3,
	jwb: Vector3,
	
	-- Final bias
	bias: number,
	effMass: number,
	totLambda: number,
}

local function getOrthogonal(normal: Vector3): (Vector3, Vector3)
	if math.abs(normal.Y) > 0.99 then
		-- It's up, so just pick the other two
		-- basis vectors.
		return Vector3.xAxis, Vector3.zAxis
	else
		-- tangent = y x normal
		-- bitangent = tangent x normal
		local tangent = Vector3.new(normal.Z, 0, -normal.X)
		local bitangent = tangent:Cross(normal)
		return tangent.Unit, bitangent.Unit
	end
end

local function safeUnit(v: Vector3): Vector3
	if v:Dot(v) < 1e-3 then
		return Vector3.zero
	else
		return v.Unit
	end
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
	local jv = self.jva:Dot(a.linearVelocity)
		+ self.jwa:Dot(a.angularVelocity)
		+ self.jvb:Dot(b.linearVelocity)
		+ self.jwb:Dot(b.angularVelocity)

	-- Ensure that sum lambda_i >= 0, cannot clamp
	-- lambda on its own. Lambda is the difference between
	-- the clamped and previous lambda
	local effMass = self.effMass
	
	local lambda = -(jv + self.bias) * effMass
	local previousLambda = self.totLambda
	if self.flag == FLAG_TANGENT then
		-- apply friction
		local friction = contact.friction * contact.jN.totLambda
		self.totLambda = math.clamp(self.totLambda + lambda, -friction, friction)
		a.angularVelocity -= a.invInertia * safeUnit(a.angularVelocity) * ROLL_DAMPEN * friction
		b.angularVelocity -= b.invInertia * safeUnit(b.angularVelocity) * ROLL_DAMPEN * friction
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

local function createJacobian(): Jacobian
	return {
		-- state variables
		flag = 0,
		jva = Vector3.zero,
		jwa = Vector3.zero,
		jvb = Vector3.zero,
		jwb = Vector3.zero,

		-- warm starting
		bias = 0.0,
		totLambda = 0.0,
		effMass = 0.0,
	}
end

local function initJacobian(
	jacobian: Jacobian,
	a: Body,
	b: Body,
	
	normal: Vector3,
	rA: Vector3,
	rB: Vector3,
	flag: number,
	depth: number
)
	-- state variables
	local jva = -normal
	local jwa = -rA:Cross(normal)
	local jvb = normal
	local jwb = rB:Cross(normal)
	
	--[[
		C: (b - a) . n >= 0
		C': ((v_b + w_b x r_b) - (v_a + w_a x r_a)). n >= 0
		C': ((v_b - r_b x w_b) - (v_a - r_a x w_a)). n >= 0

		Know: a x b . c = c x a . b
		C': -(r_b x w_b).n + (r_a x w_a).n + (v_b - v_a).n
		C': -(n x r_b).w_b + (n x r_a).w_a + (v_b - v_a).n

		Want: C' = JV + b
		J = [ -n^T (n x r_a)^T n^T -(n x r_b)^T ]
		J = [ -n^T -(r_a x n)^T n^T (r_b x n)^T ]
		V = [ v_a ]
			[ w_a ]
			[ v_b ]
			[ w_b ]
		    
		M_eff = 1.0 / (J * M^-1 * J^T => J_i.(M_i * J_i))
	]]
	
	local effMass = a.invMass
		+ (a.invInertia * jwa):Dot(jwa)
		+ b.invMass
		+ (b.invInertia * jwb):Dot(jwb)
	
	local bias = 0.0
	if flag == FLAG_NORMAL then
		--[[
			Doing the bias term
			C: (b - a) . n >= 0
			
			Baumgarte stabilization term(error = C)
			b = -B/dt * C
			b = -B/dt * (b-a).n
			
			Restitution(dependant on C')
			c = C'.n
		]]

		local velocityConstraint = -a.linearVelocity
			- a.angularVelocity:Cross(rA)
			+ b.linearVelocity
			+ b.angularVelocity:Cross(rB)
		
		local projected = velocityConstraint:Dot(normal)
		local restitution = a.restitution * b.restitution
		local beta = a.beta * b.beta
		
		bias = restitution * projected - 120 * beta * depth
	end

	-- state variables
	jacobian.flag = flag
	jacobian.jva = jva
	jacobian.jwa = jwa
	jacobian.jvb = jvb
	jacobian.jwb = jwb
		
	-- warm starting
	jacobian.bias = bias
	jacobian.totLambda = 0.0
	jacobian.effMass = 1.0 / effMass
end

function Contact.update(
	self: Contact,
	rA: Vector3,
	rB: Vector3,
	normal: Vector3,
	depth: number
)
	local tangent, bitangent = getOrthogonal(normal)
	local bodyA = self.bodyA
	local bodyB = self.bodyB
	
	-- Set state
	self.relativeA = bodyA.cframe:VectorToObjectSpace(rA)
	self.relativeB = bodyB.cframe:VectorToObjectSpace(rB)
	self.normal = normal
	self.rA = rA
	self.rB = rB
	
	initJacobian(self.jN, bodyA, bodyB, normal, rA, rB, FLAG_NORMAL, depth)
	initJacobian(self.jB, bodyA, bodyB, bitangent, rA, rB, FLAG_TANGENT, depth)
	initJacobian(self.jT, bodyA, bodyB, tangent, rA, rB, FLAG_TANGENT, depth)
end

function Contact.new(
	bodyA: Body,
	bodyB: Body,

	normal: Vector3,
	rA: Vector3,
	rB: Vector3,
	
	depth: number
): Contact
	local tangent, bitangent = getOrthogonal(normal)
	local jN = createJacobian()
	local jT = createJacobian()
	local jB = createJacobian()
	
	initJacobian(jB, bodyA, bodyB, bitangent, rA, rB, FLAG_TANGENT, depth)
	initJacobian(jT, bodyA, bodyB, tangent, rA, rB, FLAG_TANGENT, depth)
	initJacobian(jN, bodyA, bodyB, normal, rA, rB, FLAG_NORMAL, depth)
	
	return setmetatable({
		relativeA = bodyA.cframe:VectorToObjectSpace(rA),
		relativeB = bodyB.cframe:VectorToObjectSpace(rB),
		
		bodyA = bodyA,
		bodyB = bodyB,
		
		friction = bodyA.friction * bodyB.friction,
		normal = normal,
		
		rA = rA,
		rB = rB,

		jN = jN,
		jB = jB,
		jT = jT,
	}, Contact)
end

function Contact.solve(self: Contact, dt: number)
	-- Solve the jacobian
	solveJacobian(self.jN, self, dt)
	solveJacobian(self.jT, self, dt)
	solveJacobian(self.jB, self, dt)
end

return Contact