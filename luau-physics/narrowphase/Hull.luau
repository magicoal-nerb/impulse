--!strict

local SupportHull = require("./hulls/SupportHull")
local MeshHull = require("./hulls/MeshHull")

export type Hull = {
	-- Hull abstract base class :3
	new: (...any) -> Hull,
	update: (Hull, CFrame, Vector3) -> (),
	raycast: (Hull, number, Vector3, Vector3) -> (),

	support: Support,
}

export type Support = SupportHull.Support
export type Face = MeshHull.Face

export type SupportHull = SupportHull.SupportHull & Hull
export type MeshHull = MeshHull.MeshHull & Hull

return {
	SupportHull = SupportHull,
	MeshHull = MeshHull,
}