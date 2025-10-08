--!strict

-- sweep sah has a nlogn penality because of
-- sorting the data. however, it minimizes costs the most
-- using bonsai prune

local VEC_INF = vector.one * math.huge

export type Node = {
	center: vector,
	min: vector,
	max: vector,
}

local function sah(delta: vector): number
	-- the main cost function of our tree
	return delta.x*delta.y + delta.z*delta.x + delta.z*delta.y
end

local function quickSort(
	array: { Node },
	l: number,
	r: number,
	axis: vector
)
	if l >= r then
		return
	end

	-- i chose the middle partition
	local pivot = vector.dot(array[(l+r)//2].center, axis)
	local eq = l
	local lt = l
	local gt = r

	while eq <= gt do
		local dot = vector.dot(array[eq].center, axis)
		if dot < pivot then
			-- left hand side
			array[eq], array[lt] = array[lt], array[eq]
			lt += 1
			eq += 1
		elseif dot > pivot then
			-- right hand side
			array[eq], array[gt] = array[gt], array[eq]
			gt -= 1
		else
			-- middle
			eq += 1
		end
	end

	quickSort(array, l, lt-1, axis)
	quickSort(array, gt+1, r, axis)
end

local BvhAxes = {
	vector.create(1, 0, 0),
	vector.create(0, 1, 0),
	vector.create(0, 0, 1),
}

export type Box = {
	min: vector,
	max: vector,
}

return function(
	box: Box, 
	leaves: { Node },
	start: number,
	finish: number
): number
	for i = start, finish do
		-- union box
		local leaf = leaves[i]
		box.min = vector.min(box.min, leaf.min)
		box.max = vector.max(box.max, leaf.max)
	end

	local bestIndex = -1
	local bestCost = math.huge
	local bestAxis

	local length = (finish - start)+1
	local costs = table.create(length + 1, 0)
	for j, axis in BvhAxes do
		-- sort indicies
		quickSort(
			leaves,
			start,
			finish,
			axis
		)

		local boxMax = -VEC_INF
		local boxMin = VEC_INF
		for i = 1, length do
			-- Go from the right to the left side
			local leaf = leaves[finish + 1 - i]
			boxMin = vector.min(boxMin, leaf.min)
			boxMax = vector.max(boxMax, leaf.max)

			costs[length - i + 1] = sah(boxMax - boxMin) * i
		end

		boxMax = -VEC_INF
		boxMin = VEC_INF
		for i = 1, length - 1 do
			-- go from the left to the right side
			local leaf = leaves[start + i - 1]
			boxMin = vector.min(boxMin, leaf.min)
			boxMax = vector.max(boxMax, leaf.max)

			local cost = sah(boxMax - boxMin)*i + costs[i+1]
			if cost < bestCost then
				bestCost = cost
				bestAxis = axis
				bestIndex = vector.dot(leaf.center, axis)
			end
		end
	end

	-- unstable partition
	local ptr = start
	while ptr < finish and vector.dot(leaves[ptr].center, bestAxis) <= bestIndex do
		ptr += 1
	end

	for i = ptr + 1, finish do
		if vector.dot(leaves[i].center, bestAxis) <= bestIndex then
			-- swap to preserve partition property
			-- that is, <=[start, ptr-1] >[ptr, finish]
			leaves[i], leaves[ptr] = leaves[ptr], leaves[i]
			ptr += 1
		end
	end

	-- 	LHS			RHS
	-- [start l-1] [l finish]
	return math.clamp(ptr-1, start, finish-1)
end