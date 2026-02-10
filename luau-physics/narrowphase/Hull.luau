--!strict

local SupportHull = require("./hulls/SupportHull")
local MeshHull = require("./hulls/MeshHull")

export type Hull = {
	-- Hull abstract base class :3
	new: (...any) -> Hull,
	update: (Hull, CFrame, Vector3) -> (),
	raycast: (Hull, Vector3, Vector3) -> (number, Vector3, Vector3),
	
	-- fields
	support: (Vector3) -> Vector3,
	center: Vector3,
}

export type Support = SupportHull.Support
export type Face = MeshHull.Face

export type SupportHull = SupportHull.SupportHull
export type MeshHull = MeshHull.MeshHull

return {
	SupportHull = SupportHull,
	MeshHull = MeshHull,
}