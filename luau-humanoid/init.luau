-- luau humanoid reaction :3
-- magicoal_nerb

local Humanoid = require("@self/Humanoid")

export type Humanoid = Humanoid.Humanoid
export type State = Humanoid.State

return {
	-- main humanoid
	Humanoid = require("@self/Humanoid"),
	
	-- states
	Freefall = require("@self/states/Freefall"),
	Climbing = require("@self/states/Climbing"),
	Jumping = require("@self/states/Jumping"),
	Running = require("@self/states/Running"),
}