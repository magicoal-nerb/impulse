# impulse
Luau physics engine that works off of sequential impulses.   
It also aims to fully reimplement Roblox's humanoid physics   

## Usage
This project uses [Roblox Studio](https://github.com/lune-org/lune) to visualize, although any environment that uses [Luau](https://luau.org/) will work.
This project can be built through [Rojo](https://rojo.space)

```bash
git clone "https://github.com/magicoal-nerb/luau-physics.git"
cd lua-luau
rojo serve
```

## Features
- One shot contact manifold generation
- Uses sequential impulses to solve contact constraints
- Optimized broadphase using dynamic BVHs that use tree rotations
- Warm starting
- Sleeping
- Stable solver(check the tests below)
- Baumgarte Stabilization
- Built-in collision tests such as GJK/SAT
- Works with any convex shape, physics models can be specified with a wavefront
- General shapecasting, raycasting, and blockcasting(beats Roblox's query performance suprisingly...)

## Goals
- Creating an efficient physics engine in Luau
- Making a fine-tunable physics engine that is heavily customizable for developers' needs
	- reliable touches
	- a steppable world without needing an external plugin or headless studio client
	- hopefully a valuable resource for hobbyists creating their own physics engines
- Future proofing Roblox's Humanoid physics, hopefully to the point where it can be used as a substitute humanoid for obbying or server authorative movement

## Planned features
- Dynamic contact islands
- Experimenting with block solving contact points under PGS/direct solves if sufficient
- Other constraints such as hinges, prismatic constraints, etc...
- Islands that group up other bodies(similar to how Welds work)
- Incremental manifold building

## Showcase
https://github.com/user-attachments/assets/2ed574bd-29b7-4d0d-9e25-926b3e5598dd


## Contributing
Any contributions are greatly appreciated! I might alter pull requests for consistent coding style and performance, but please try keeping code clean!!   

Shield: [![CC BY 4.0][cc-by-shield]][cc-by]

This work is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png
[cc-by-shield]: https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg
