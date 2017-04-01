# AnimMorph

## Intro
Project for Professor Justin Solomon's MIT class, Shape Analysis (6.838).

Implementation of Chen, Weber, Keren, and Ben-Chen's "Planar Shape Interpolation with Bounded Distortion."

Some planned contributions:
	1. Smooth B-spline mesh interpolations
	2. Intuitive UI
	3. Animation tools

- Jared Counts

## Todo
* Add toggleable 2D mode to Camera
* Would be nice if we shared libraries with my other project (e.g., Camera, Mouseable, etc. were copy pasted from elsewhere)
* Separate window logic from loop logic
* Mesh loading
* Texturing
* Convert texture to mesh
* Embed mesh so it has the same relative orientation as original mesh.
* Clean up Shape morph code
	* Separate out some kind of MeshHelper for efficiently mapping vertices to edges, edges to triangles, etc.
	* Optimizations?
		* For instance, all topological math can be done only once.
		* Profiling (memory and time)
* B-spline mesh interpolation
* Mesh posing
* Mesh animating