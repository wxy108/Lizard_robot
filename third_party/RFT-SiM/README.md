# RFT-SiM for legged :robot:s
***built on Python 3.12.10 and MuJoCo 3.3.8***
# [RFT_SiM_Example_Crabwalk.py](RFT_SiM_Example_Crabwalk.py)
- this runs an example :crab: robot simulation in the RFT-SiM pipeline.
  - outputs: robot position over time, video of simulation

# [sim_fxn_lib.py](sim_fxn_lib.py)
- houses helper functions and RFT force code

# [Rigid_crabwalking](Rigid_crabwalking_01.ipynb) 
- example robot simulation without sand ground

# asset
- contains meshes and textures for model

# Guide
To run RFT-SiM, the model xml and meshes need to be prepared in a specific way.
Please see [Robot_sand.xml](Robot_Sand.xml) for an example model xml.

- The control signals can be modified in the script. (see control_pos_mid_fr in [RFT_SiM_Example_Crabwalk.py](RFT_SiM_Example_Crabwalk.py))
- To determine INERTIAS in the xml, I recommend pulling the diagonal matrix from a SolidWorks part model with material selected.
- zeta = 3.75 is the number for Quikrete Play Sand. Other sands will need a different number which can be found via the method in SM of Terradynamics of Legged Locomotion on Granular Media by Li et al.

- the nitty-gritty;
  - RFT-SiM is very particular about meshes. Each body in a robot model should have its own mesh (if two bodies are identical, the same mesh can be used with proper indexing of names).
  - Each of these meshes should have an identical number of triangles
    - To do this, bring the CAD part or stl file into a program like Blender or Fusion 360. Once the part is brought in, center it in the workspace. Then, use the meshing tools (remesh, reduce) to create a mesh with evenly spaced, or concentrated the bottom, triangles along the surface. 500-1000 triangles is recommended. More triangles creates a slower, more accurate simulation. 
  - The xml file needs to have X sites for each body, where X is the number of triangles in the mesh. These should be named in a fashion similar to [Robot_sand.xml](Robot_Sand.xml).
    - This will make the xml very long. It is recommended to use a python script to generate these site lists and then paste/append it in the xml.

# LICENSE
MIT License

Copyright (c) 2026 Ryan Walker Brown

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
