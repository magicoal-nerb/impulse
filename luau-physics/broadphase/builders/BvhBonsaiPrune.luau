--!strict

-- optimization phase during construction
-- where we just rotate through the entire tree
-- and make pruned mini trees. good for static stuff

local Queue = require("../Queue")

local FLAG_LEAF = 0x2

export type Box = {
	flag: number,
	center: Vector3,
	min: Vector3,
	max: Vector3,
	free: number,
	left: number,
	right: number,
	parent: number,
}

local function sah(box: Box): number
	-- the main cost function of our tree
	local delta = box.max - box.min
	return delta.X*delta.Y + delta.Z*delta.X + delta.Z*delta.Y
end

export type Bvh = typeof(setmetatable({} :: {
	root: number,
	freeIndex: number,
}, {
	__index = {} :: {	
		rotate: (self: Bvh, id: number) -> (),
		insert: (self: Bvh, node: Box) -> (),
	}
}))

return function<T>(
	bvh: Bvh,
	nodes: { Box },
	queue: Queue.Queue<number>
)
	-- do rotations from the entire tree, bottom-up
	local function dfs(id: number)
		local node = nodes[id]
		if node.flag == FLAG_LEAF then
			return
		end

		dfs(node.left)
		dfs(node.right)

		bvh:rotate(id)

		local leftNode = nodes[node.left]
		local rightNode = nodes[node.right]
		node.min = leftNode.min:Min(rightNode.min)
		node.max = leftNode.max:Max(rightNode.max)
	end

	dfs(bvh.root)

	-- create a queue
	local root = bvh.root
	local threshold = sah(nodes[root]) * 0.05
	local mini = {} :: { number }

	while not queue:empty() do
		local id = queue:dequeue()
		local node = nodes[id]
		assert(node)

		if node.flag == FLAG_LEAF then
			-- dont continue if its a leaf node
			-- but do track the parent
			table.insert(mini, id)
			continue
		elseif sah(node) > threshold then
			-- branch node violates the property. continue
			-- dfs
			queue:enqueue(node.right)
			queue:enqueue(node.left)
		else
			-- branch node that does not violate property
			-- use this as your new mini tree
			local parent = node.parent
			while nodes[parent] do
				local parentNode = nodes[parent]
				nodes[parent] = nil

				-- add to freelist
				parentNode.free = bvh.freeIndex
				bvh.freeIndex = parent

				parent = parentNode.parent
			end
			table.insert(mini, id)
		end
	end

	for i, nodeId in mini do
		-- Insert a tiny subset of nodes
		-- in there.
		local node = nodes[nodeId]
		bvh:insert(node)
	end
end