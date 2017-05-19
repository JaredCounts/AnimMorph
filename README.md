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
* Texturing
* Convert texture to mesh
* Separate topology representation from Mesh class
* Clean up Shape morph code
	* Optimizations?
		* For instance, all topological math can be done only once.
		* Profiling (memory and time)
* UI
	* Improve mesh posing
	* Add handles for editing the Bezier motion
	