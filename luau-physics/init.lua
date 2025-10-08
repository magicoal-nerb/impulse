local World = require("@self/World")

export type World = World.World
export type Body = World.Body
export type Bvh = World.Bvh

return {
	World = require("@self/World"),
}