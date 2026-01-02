--!native

-- really basic simulation logic
-- for the humanoid and world
-- but, i advise you to make your own!!

assert(not workspace.StreamingEnabled, "this simulation script assumes StreamingEnabled = false")

local Pausing = false
local HZ = 240

local ContextActionService = game:GetService("ContextActionService")
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
	rootPart.CFrame = workspace.tp.CFrame
	
	local baseHeight = if RealHumanoid.RigType == Enum.HumanoidRigType.R15
		then 0.15
		else 3
	
	
	CurrentHumanoid = Humanoid.new(rootPart)
	CurrentHumanoid.hipHeight = RealHumanoid.HipHeight + baseHeight
end

-- debugging purposes
ContextActionService:BindAction(
	"pause-physics",
	function(_, state: Enum.UserInputState)
		if state == Enum.UserInputState.Begin then
			Pausing = not Pausing
		end
	end,
	false,
	Enum.KeyCode.Q
)

ContextActionService:BindAction(
	"step-physics",
	function(_, state: Enum.UserInputState)
		if state == Enum.UserInputState.Begin then
			workspace.Wireframe:Clear()
			
			local state = CurrentHumanoid.state
			CurrentHumanoid:update(1 / HZ)
			if CurrentHumanoid.state ~= state then
				RealHumanoid:ChangeState(CurrentHumanoid.state)
			end
			
			World.step(1 / HZ)
		end
	end,
	false,
	Enum.KeyCode.E
)

local previous = os.clock()
local function heartbeat(dt: number)
	
	local clock = os.clock()
	if not CurrentHumanoid or Pausing then
		previous = clock
		return
	end
	
	local ticks = (clock - previous) * HZ
	workspace.Wireframe:Clear()

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

	local remainder = ((ticks % 1) / HZ)
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

task.wait(0.5)
previous = os.clock()
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