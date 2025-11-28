--!native

-- really basic simulation logic
-- for the humanoid and world

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")
local Players = game:GetService("Players")

local LocalPlayer = assert(Players.LocalPlayer)

local LuauHumanoid = require(ReplicatedStorage["luau-humanoid"])
local LuauPhysics = require(ReplicatedStorage["luau-physics"])

local what = Instance.new("Hint", workspace)

local Humanoid = LuauHumanoid.Humanoid
local World = LuauPhysics.World

local CurrentHumanoid: LuauHumanoid.Humanoid
local RealHumanoid: Humanoid

local function characterAdded()
	local character = LocalPlayer.Character
	RealHumanoid = character:WaitForChild("Humanoid") :: Humanoid
	
	local rootPart = RealHumanoid.RootPart
	RealHumanoid:GetPropertyChangedSignal("MoveDirection")
		:Connect(function()
			CurrentHumanoid.moveDirection = RealHumanoid.MoveDirection
		end)
	
	RealHumanoid:GetPropertyChangedSignal("Jump")
		:Connect(function()
			CurrentHumanoid.jump = RealHumanoid.Jump
		end)
	
	-- create the humanoid from the root part
	RealHumanoid.EvaluateStateMachine = false
	RealHumanoid:ChangeState(Enum.HumanoidStateType.Running)
	rootPart.Anchored = true
	
	local baseHeight = if RealHumanoid.RigType == Enum.HumanoidRigType.R15
		then 0.15
		else 3
	
	CurrentHumanoid = Humanoid.new(rootPart)
	CurrentHumanoid.hipHeight = RealHumanoid.HipHeight + baseHeight
end

local HZ = 240

local previous = os.clock()
local function heartbeat(dt: number)
	local clock = os.clock()
	if not CurrentHumanoid then
		previous = clock
		return
	end
	
	local ticks = (clock - previous) * HZ
	--World.bvh:draw()

	local t0 = 0.0
	local t1 = 0.0
	
	-- set state, whatever this is
	local state = CurrentHumanoid.state
	for i = 1, ticks do
		CurrentHumanoid:update(1 / HZ)
		if CurrentHumanoid.state ~= state then
			RealHumanoid:ChangeState(CurrentHumanoid.state)
		end
		
		World.step(1 / HZ)
	end

	local remainder = (ticks % 1) / HZ
	CurrentHumanoid:update(remainder)
	if CurrentHumanoid.state ~= state then
		RealHumanoid:ChangeState(CurrentHumanoid.state)
	end

	t0 = os.clock()
	World.step(remainder)
	t1 = os.clock()
	
	what.Text = `{((t1-t0)*1e6)//1}us, {World.numActive} awake`

	previous = clock
end

if LocalPlayer.Character then
	characterAdded()
end

LocalPlayer.CharacterAdded:Connect(characterAdded)
RunService.Heartbeat:Connect(heartbeat)

-- initialize the world
for _, v in workspace.static:GetDescendants() do
	if v:IsA("BasePart") then
		World.addStaticBody(v)
	end
end

for i, v in workspace.dynamic:GetChildren() do
	World.addDynamicBody(v)
end