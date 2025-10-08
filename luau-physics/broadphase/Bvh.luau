--!strict

local BvhBonsaiPrune = require("./builders/BvhBonsaiPrune")
local BvhBinnedSah = require("./builders/BvhBinnedSah")

local Queue = require("./Queue")

local VEC_INF = vector.one * math.huge

-- Truthfully we only care if a node's id
-- is solely FLAG_BRANCH, because that is an internal
-- node made by the bvh. Flags that don't follow this
-- are leaves by default.
local FLAG_BRANCH = 0x1
local FLAG_LEAF = 0x2

local Bvh = {}
Bvh.__index = Bvh

export type Bin = Box & { count: number }
export type Box = {
	center: vector,
	min: vector,
	max: vector,
}

local function sah(box: Box): number
	-- the main cost function of our tree
	local delta = box.max - box.min
	return 2*(delta.x*delta.y + delta.z*delta.x + delta.z*delta.y)
end

local function unionCost(
	box0: Box,
	box1: Box
): number
	-- gets the cost of combinine two boxes
	-- together. the cost function is just the surface
	-- area of the boxes.
	local delta = vector.max(box0.max, box1.max) - vector.min(box0.min, box1.min)
	return 2*(delta.x*delta.y + delta.z*delta.x + delta.z*delta.y)
end

local function doesAABBCollidePoint(
	min: vector,
	max: vector,
	point0: vector
): boolean
	-- checks if a bounding box and point collides.
	-- consider the case:
	-- A___P___A

	-- in this case, A_min < P < A_max. basically this just
	-- asks if P is bounded within A_min and A_max. also, we're
	-- still using the number line analogy, so this will generalize
	-- to all 3 axes.
	return min.x < point0.x and point0.x < max.x
		and min.y < point0.y and point0.y < max.y
		and min.z < point0.z and point0.z < max.z
end

local function doesAABBCollideAABB(aabb0: Box, aabb1: Box): boolean
	-- checks if two aabbs collide
	local min0 = aabb0.min
	local max0 = aabb0.max

	local min1 = aabb1.min
	local max1 = aabb1.max

	return min0.x <= max1.x and max0.x >= min1.x
		and min0.y <= max1.y and max0.y >= min1.y
		and min0.z <= max1.z and max0.z >= min1.z
end

local function doesAABBCollideRay(
	node: Box,
	origin: vector, 
	invDir: vector, 
	size: vector
): number
	-- We know about a + bt = c, where t = (c - a)b
	-- We use this relationship to calculate vMin, vMax
	-- and we assume the T value is within [0, 1]
	local halfSize = size * 0.5
	local min = node.min - halfSize
	local max = node.max + halfSize
	if doesAABBCollidePoint(min, max, origin) then
		-- Premature check
		return 0.0
	end

	local vMin = (min - origin) * invDir
	local vMax = (max - origin) * invDir

	-- Quick branchless optimization
	local rMin = vector.min(vMin,vMax)
	local rMax = vector.max(vMax,vMin)

	local tMin = math.max(rMin.x, rMin.y, rMin.z)
	local tMax = math.min(rMax.y, rMax.y, rMax.z)
	return tMax >= 0 
		and tMin <= tMax 
		and tMin <= 1 
		and tMin >= 0
		and tMin
		or -1
end

export type BvhNode<T> = Box & T & {
	flag: number,
	left: number,
	right: number,
	parent: number,
	free: number,
}

export type Bvh<T> = typeof(setmetatable({} :: {
	queue: Queue.Queue<number>,
	freeIndex: number,
	nodes: { BvhNode<T> },
	count: number,
	root: number,
}, Bvh))

function Bvh.new<T>(): Bvh<T>
	return setmetatable({
		queue = Queue.new(16, 0),
		freeIndex = 0,
		nodes = {},
		count = 0,
		root = 1,
	}, Bvh)
end

local BvhProxy: Box = {
	center = vector.zero,
	min = vector.one,
	max = vector.one,
}

function Bvh.query<T>(
	self: Bvh<T>,
	min: vector,
	max: vector
): { number }
	local nodes = self.nodes
	if not nodes[self.root] then
		-- nothing
		return {}
	end

	local output = {}
	local queue = self.queue
	queue:clear()
	queue:enqueue(self.root)

	BvhProxy.min = min
	BvhProxy.max = max
	while not queue:empty() do
		local id = queue:dequeue()
		local node = nodes[id]
		if node.flag ~= FLAG_BRANCH then
			-- leaf node => output
			if doesAABBCollideAABB(node, BvhProxy) then
				-- add node
				table.insert(output, id)
			end
		else
			-- branch node => queue
			local l = node.left
			if l ~= 0 and doesAABBCollideAABB(nodes[l], BvhProxy) then
				-- add left node
				queue:enqueue(l)
			end

			local r = node.right
			if r ~= 0 and doesAABBCollideAABB(nodes[r], BvhProxy) then
				-- add right node
				queue:enqueue(r)
			end
		end
	end

	return output
end

function Bvh.trace<T>(
	self: Bvh<T>,
	origin: vector,
	direction: vector,
	size: vector
): { number }
	-- box raycast operation
	local invDir = vector.one / direction
	local nodes = self.nodes
	if not nodes[self.root] then
		-- Don't query.
		return {}
	end

	debug.profilebegin("bvh::trace")

	local queue = self.queue
	queue:clear()
	queue:enqueue(self.root)

	local output = {}
	while not queue:empty() do
		local id = queue:dequeue()
		local node = nodes[id]
		if node.flag ~= FLAG_BRANCH then
			-- leaf node => output
			local t = doesAABBCollideRay(node, origin, invDir, size)			
			if 0 <= t and t <= 1 then
				-- add left node
				table.insert(output, id)
			end
		else
			-- branch node
			local t0 = doesAABBCollideRay(nodes[node.left], origin, invDir, size)
			if 0 <= t0 and t0 <= 1 then
				-- add left node
				queue:enqueue(node.left)
			end

			local t1 = doesAABBCollideRay(nodes[node.right], origin, invDir, size)			
			if 0 <= t1 and t1 <= 1 then
				-- add right node
				queue:enqueue(node.right)
			end
		end
	end

	debug.profileend()

	return output
end

function Bvh.rotate<T>(self: Bvh<T>, index: number)
	if index == self.root then
		-- dont rotate the root
		-- node
		return
	end

	-- collect nodes
	local nodes = self.nodes
	local node = nodes[index]

	local parent = node.parent
	local right = node.right
	local left = node.left

	local parentNode = nodes[parent]
	local rightNode = nodes[right]
	local leftNode = nodes[left]

	local sibling	
	if parentNode.left == index then
		sibling = parentNode.right
	else
		sibling = parentNode.left
	end

	local siblingNode = nodes[sibling]	
	local leftArea = unionCost(siblingNode, leftNode)
	local rightArea = unionCost(siblingNode, rightNode)
	local area = unionCost(leftNode, rightNode)
	if area <= leftArea and area <= rightArea then
		-- dont rotate
		return
	end

	if leftArea < rightArea then
		-- swap right and sibling
		if parentNode.left == index then
			parentNode.right = right
		else
			parentNode.left = right
		end

		node.right = sibling
		rightNode.parent = parent
	else
		-- swap left and sibling
		if parentNode.left == index then
			parentNode.right = left
		else
			parentNode.left = left
		end

		node.left = sibling
		leftNode.parent = parent
	end

	siblingNode.parent = index
end

function Bvh.refit<T>(self: Bvh<T>, index: number)
	-- stage 3: refit
	local nodes = self.nodes
	while index ~= 0 do
		-- rotate
		local node = nodes[index]
		self:rotate(index)

		local leftNode = nodes[node.left]
		local rightNode = nodes[node.right]
		node.min = vector.min(leftNode.min, rightNode.min)
		node.max = vector.max(leftNode.max, rightNode.max)

		index = node.parent
	end
end

function Bvh.remove<T>(self: Bvh<T>, node: BvhNode<T>)
	-- removes a node
	local nodes = self.nodes
	local parent = node.parent
	local parentNode = nodes[parent]
	
	if not parentNode then
		-- add to freelist
		local index = table.find(nodes, node) :: number
		assert(index ~= self.root)
		node.free = self.freeIndex
		node.parent = 0
		
		self.freeIndex = index
		return
	end
	
	local sibling	
	local index

	-- obtain the sibling index
	if nodes[parentNode.left] == node then
		sibling = parentNode.right
		index = parentNode.left
	else
		sibling = parentNode.left
		index = parentNode.right
	end

	local newParent = parentNode.parent
	if newParent ~= 0 then
		-- grandparent is a branch node
		local newParentNode = nodes[newParent]
		if newParentNode.left == parent then
			newParentNode.left = sibling
		else
			newParentNode.right = sibling
		end

		-- refit if possible
		-- set sibling parent
		nodes[sibling].parent = newParent
		nodes[parent] = nil
		
		-- stage 3: refit
		local nodes = self.nodes
		local cursor = newParent
		while cursor ~= 0 do
			-- rotate
			local node = nodes[cursor]
			local leftNode = nodes[node.left]
			local rightNode = nodes[node.right]
			node.min = vector.min(leftNode.min, rightNode.min)
			node.max = vector.max(leftNode.max, rightNode.max)

			cursor = node.parent
		end
	else
		-- sibling becomes root node
		-- set sibling parent
		nodes[sibling].parent = 0
		self.root = sibling
	end

	-- add to freelist
	node.free = self.freeIndex
	self.freeIndex = index
end

function Bvh.getFreeIndex<T>(self: Bvh<T>): number
	-- we use this so we dont
	-- need to keep on creating
	-- redundant data. this acts similarly
	-- to a freelist
	local id = self.freeIndex
	if id ~= 0 then
		self.freeIndex = self.nodes[self.freeIndex].free
		return id
	else
		self.count += 1
		return self.count
	end
end

function Bvh.getObject<T>(self: Bvh<T>, id: number): BvhNode<T>
	-- provides an object given the id
	return self.nodes[id]
end

function Bvh.insert<T>(self: Bvh<T>, leaf: BvhNode<T>): number
	-- Stage 1: find best sibling
	-- based off of erin catto's dynamic aabb tree
	-- insertion method, where we just find a good enough
	-- sibling node to insert into
	local siblingIndex = self.root
	local nodes = self.nodes

	local leafIndex = self:getFreeIndex()
	leaf.free = 0

	if not nodes[siblingIndex] then
		leaf.parent = 0
		self.root = leafIndex

		nodes[self.root] = leaf
		return leafIndex
	end

	local area = sah(leaf)
	while siblingIndex do
		local sibling = nodes[siblingIndex]
		if sibling.flag ~= FLAG_BRANCH then
			break
		end

		-- the inherit cost is the surface area added
		-- when attempting to insert a node here.
		local cost = 2*sah(sibling)
		local inheritCost = 2*unionCost(sibling, leaf) - cost
		local r = nodes[sibling.right]
		local l = nodes[sibling.left]	

		local leftCost = unionCost(leaf, l) + inheritCost
		if l.flag == FLAG_BRANCH then
			leftCost -= sah(l)
		end

		local rightCost = unionCost(leaf, r) + inheritCost
		if r.flag == FLAG_BRANCH then
			rightCost -= sah(r)
		end

		if cost <= leftCost and cost <= rightCost then
			break
		end

		if leftCost < rightCost then
			siblingIndex = sibling.left
		else
			siblingIndex = sibling.right
		end
	end

	-- stage 2: create new parent
	local sibling = nodes[siblingIndex]
	local parentIndex = sibling.parent
	local branchIndex = self:getFreeIndex()

	-- create aabb
	local branch = {
		min = VEC_INF,
		max = -VEC_INF,
		flag = FLAG_BRANCH,
		right = leafIndex,
		left = siblingIndex,
		parent = parentIndex,
		free = 0,
	} :: BvhNode<T>

	sibling.parent = branchIndex
	leaf.parent = branchIndex

	nodes[branchIndex] = branch
	nodes[leafIndex] = leaf

	if parentIndex ~= 0 then
		-- parent is an existing node
		local parent = nodes[parentIndex]
		if parent.left == siblingIndex then
			parent.left = branchIndex
		else
			parent.right = branchIndex
		end

		self:refit(branchIndex)
	else
		-- parent was the root node
		branch.min = vector.min(leaf.min, sibling.min)
		branch.max = vector.max(leaf.max, sibling.max)
		self.root = branchIndex
	end

	return leafIndex
end

export type BvhWork = {
	nodeIndex: number,
	parent: number,
	leftIndex: number,
	rightIndex: number,
}

function Bvh.build<T>(self: Bvh<T>, leaves: { BvhNode<T> })
	-- Do this when the bvh is empty.
	local nodes = {}
	local size = 1

	local buildQueue: Queue.Queue<BvhWork> = Queue.new(16, {} :: BvhWork)
	buildQueue:clear()
	buildQueue:enqueue({
		nodeIndex = 1,
		parent = 0,

		leftIndex = 1,
		rightIndex = #leaves,
	})

	while not buildQueue:empty() do
		local pair = buildQueue:dequeue()
		local count = (pair.rightIndex - pair.leftIndex) + 1
		if count == 1 then
			-- leaf case
			local node = leaves[pair.leftIndex]
			node.flag = FLAG_LEAF
			node.parent = pair.parent
			nodes[pair.nodeIndex] = node
			continue
		elseif count == 2 then
			-- branch case
			local right = leaves[pair.rightIndex]
			local left = leaves[pair.leftIndex]

			local min = vector.min(left.min, right.min)
			local max = vector.max(left.max, right.max)
			local branch = {
				min = min,
				max = max,
				center = (min + max) * 0.5,
				parent = pair.parent,
				left = size + 1,
				right = size + 2,
				flag = FLAG_BRANCH,
			} :: BvhNode<T>

			right.parent = pair.nodeIndex
			left.parent = pair.nodeIndex

			right.flag = FLAG_LEAF
			left.flag = FLAG_LEAF

			nodes[pair.nodeIndex] = branch
			nodes[size + 1] = left
			nodes[size + 2] = right

			size += 2
			continue
		elseif count < 2 then
			-- invalid case (count = 0)
			error("bvh! invalid partition lol")
		end
		
		-- create a branch node
		local branch = {
			max = -VEC_INF,
			min = VEC_INF,
			center = vector.zero,
			parent = pair.parent,
			left = size + 1,
			right = size + 2,
			flag = FLAG_BRANCH,
		} :: BvhNode<T>
		
		-- find the split, which in this case
		-- we'll use a binned sah
		local partition = BvhBinnedSah(
			leaves, 
			branch, 
			pair.leftIndex, 
			pair.rightIndex
		)
		
		-- add to work
		buildQueue:enqueue({
			nodeIndex = size + 1,
			parent = pair.nodeIndex,

			leftIndex = pair.leftIndex,
			rightIndex = partition - 1,
		})

		buildQueue:enqueue({
			nodeIndex = size + 2,
			parent = pair.nodeIndex,

			leftIndex = partition,
			rightIndex = pair.rightIndex,
		})

		size += 2
		nodes[pair.nodeIndex] = branch
	end

	assert(#nodes == size, "bvh! num nodes != size")
	self.nodes = nodes
	self.count = size
	self.root = 1

	-- do the bonsai prune to improve static tree cost
	BvhBonsaiPrune(self :: any, nodes, self.queue)
end

function Bvh.getCost<T>(self: Bvh<T>): number
	-- get the bvh cost
	local nodes = self.nodes
	local queue = self.queue
	queue:clear()
	queue:enqueue(self.root)

	local cost = 0.0
	while #queue > 0 do
		local pop = nodes[queue:dequeue()]
		if pop and pop.flag == FLAG_BRANCH then
			queue:enqueue(pop.left)
			queue:enqueue(pop.right)
			cost += sah(pop)
		end
	end

	return cost
end

local dbg = Instance.new("Folder", workspace)
function Bvh.draw<T>(self: Bvh<T>)
	-- debugger
	local nodes = self.nodes
	local queue = self.queue
	queue:enqueue(self.root)
	dbg:ClearAllChildren()
	
	while not queue:empty() do
		local id = queue:dequeue()
		local element = nodes[id]
		if not element then
			continue
		end
		
		local min = element.min
		local max = element.max

		local part = Instance.new("Part")
		part.Anchored = true
		part.CFrame = CFrame.new((min+max)*0.5)
		part.Size = max - min
		part.CanCollide = false
		part.CanQuery = false
		part.CanTouch = false
		part.Transparency = 1.0

		local sel = Instance.new("SelectionBox")
		sel.LineThickness = 0.05
		sel.Adornee = part
		sel.Parent = part
		part.Parent = dbg
		
		if element.flag == FLAG_BRANCH then
			queue:enqueue(element.left)
			queue:enqueue(element.right)
		end
	end
end

local fatAABB = vector.one

function Bvh.update<T>(
	self: Bvh<T>,
	node: BvhNode<T>,
	min0: vector,
	max0: vector
)
	local min1 = node.min - fatAABB
	local max1 = node.max + fatAABB

	if min1.x <= min0.x and min1.y <= min0.y and min1.z <= min0.z 
		and max0.x <= max1.x and max0.y <= max1.y and max0.z <= max1.z then
		-- no need to update because the fat aabb still persists
		return
	end
	
	-- reinsert because it is outside of the aabb range
	node.min = min0 - fatAABB
	node.max = max0 + fatAABB
	
	self:remove(node)
	self:insert(node)
end

return Bvh