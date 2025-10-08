--!strict

-- binned sah builder ejemplo

local VEC_INF = vector.one * math.huge
local BINS = 8

export type Bin = Node & { count: number }
export type Node = {
	center: vector,
	min: vector,
	max: vector,
}

local BvhSahAxes: { vector } = table.freeze({
	vector.create(1, 0, 0),
	vector.create(0, 1, 0),
	vector.create(0, 0, 1),
})

local function getNodeCost(node: Bin): number
	-- gets the cost of a node, if it has no leaves
	-- then it has the highest cost
	if node.count == 0 then
		return math.huge
	end

	local delta = node.max - node.min
	return 2*(delta.x*delta.y + delta.z*delta.x + delta.z*delta.y)*node.count
end

local function createEmptyBbox(): Bin
	return {
		center = vector.zero,
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
		box.min = vector.min(box.min, node.min)
		box.max = vector.max(box.max, node.max)
	end

	local delta = box.max - box.min
	local costs = {}

	local bestCost = math.huge
	local bestAxis = 1
	local bestBin = 1

	for i, axis in BvhSahAxes do
		-- Create bins
		local dot = vector.dot(delta, axis)
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
		local minBounds = vector.dot(box.min, axis)
		for j = leftIndex, rightIndex do
			local node = leaves[j]
			local index = (vector.dot(node.center, axis) - minBounds) * invSplitT
			index = (index // 1) + 1

			local bin = bins[index]
			bin.min = vector.min(bin.min, node.min)
			bin.max = vector.max(bin.max, node.max)
			bin.count += 1
		end

		-- Populate costs
		local rightBox = createEmptyBbox()
		local leftBox = createEmptyBbox()
		for j = 1, BINS do
			local leftBin = bins[j]
			if leftBin.count > 0 then
				leftBox.min = vector.min(leftBox.min, leftBin.min)
				leftBox.max = vector.max(leftBox.max, leftBin.max)
				leftBox.count += leftBin.count
			end

			costs[j] = getNodeCost(leftBox)
		end

		-- Add costs
		for j = BINS, 1, -1 do
			costs[j] += getNodeCost(rightBox)

			local rightBin = bins[j]
			if rightBin.count > 0 then
				rightBox.min = vector.min(rightBox.min, rightBin.min)
				rightBox.max = vector.max(rightBox.max, rightBin.max)
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
	local splitPosition = vector.dot(box.min + (bestBin / (BINS + 1)) * delta, axis)
	local left = leftIndex
	local right = rightIndex
	while left < right do
		if vector.dot(leaves[left].center, axis) >= splitPosition then
			while left < right and vector.dot(leaves[right].center, axis) >= splitPosition do
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