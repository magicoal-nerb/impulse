--!strict

-- binned sah builder ejemplo

local VEC_INF = Vector3.one * math.huge
local BINS = 8

export type Bin = Node & { count: number }
export type Node = {
	center: Vector3,
	min: Vector3,
	max: Vector3,
}

local BvhSahAxes: { Vector3 } = table.freeze({
	Vector3.new(1, 0, 0),
	Vector3.new(0, 1, 0),
	Vector3.new(0, 0, 1),
})

local function getNodeCost(node: Bin): number
	-- gets the cost of a node, if it has no leaves
	-- then it has the highest cost
	if node.count == 0 then
		return math.huge
	end

	local delta = node.max - node.min
	return 2*(delta.X*delta.Y + delta.Z*delta.X + delta.Z*delta.Y)*node.count
end

local function createEmptyBbox(): Bin
	return {
		center = Vector3.zero,
		count = 0,
		max = -VEC_INF,
		min = VEC_INF,
	}
end

return function(
	leaves: { Node },
	box: Node,
	leftIndex: number,
	rightIndex: number
): number
	for i = leftIndex, rightIndex do
		local node = leaves[i]
		box.min = box.min:Min(node.min)
		box.max = box.max:Max(node.max)
	end

	local delta = box.max - box.min
	local costs = {}

	local bestCost = math.huge
	local bestAxis = 1
	local bestBin = 1

	for i, axis in BvhSahAxes do
		-- Create bins
		local dot = delta:Dot(axis)
		if dot < 1e-2 then
			-- Don't use this.
			continue
		end

		local invSplitT = (BINS - 1) / dot
		local bins = {} :: { Bin }
		for j = 1, BINS do
			bins[j] = createEmptyBbox()
		end

		-- Populate bins
		local minBounds = box.min:Dot(axis)
		for j = leftIndex, rightIndex do
			local node = leaves[j]
			local index = (node.center:Dot(axis) - minBounds) * invSplitT
			index = (index // 1) + 1

			local bin = bins[index]
			bin.min = bin.min:Min(node.min)
			bin.max = bin.max:Max(node.max)
			bin.count += 1
		end

		-- Populate costs
		local rightBox = createEmptyBbox()
		local leftBox = createEmptyBbox()
		for j = 1, BINS do
			local leftBin = bins[j]
			if leftBin.count > 0 then
				leftBox.min = leftBox.min:Min(leftBin.min)
				leftBox.max = leftBox.max:Max(leftBin.max)
				leftBox.count += leftBin.count
			end

			costs[j] = getNodeCost(leftBox)
		end

		-- Add costs
		for j = BINS, 1, -1 do
			costs[j] += getNodeCost(rightBox)

			local rightBin = bins[j]
			if rightBin.count > 0 then
				rightBox.min = rightBox.min:Min(rightBin.min)
				rightBox.max = rightBox.max:Max(rightBin.max)
				rightBox.count += rightBin.count
			end
		end

		-- Get minimum cost
		for j = 1, BINS do
			local cost = costs[j]
			if cost < bestCost then
				bestCost = cost
				bestAxis = i
				bestBin = j
			end
		end
	end

	local axis = BvhSahAxes[bestAxis]	
	assert(bestBin)

	-- Partition data along the split position through swaps.
	local splitPosition = (box.min + (bestBin / (BINS + 1)) * delta):Dot(axis)
	local left = leftIndex
	local right = rightIndex
	while left < right do
		if leaves[left].center:Dot(axis) >= splitPosition then
			while left < right and leaves[right].center:Dot(axis) >= splitPosition do
				right -= 1
			end

			-- swap candidates
			leaves[left], leaves[right] = leaves[right], leaves[left]
		end

		left += 1
	end

	if left <= leftIndex + 1 or left >= rightIndex - 1 then
		return (leftIndex + rightIndex) // 2
	else
		return left
	end
end