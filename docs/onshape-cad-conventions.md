# Onshape CAD Modeling Conventions

## Frame orientation

**World frame** (shared reference for the assembly):

- <span style="color: #F44336; font-weight: bold;">X</span> — forward  
- <span style="color: #4CAF50; font-weight: bold;">Y</span> — left  
- <span style="color: #2196F3; font-weight: bold;">Z</span> — up  

**Local frames** (per body / link): In Onshape, joint rotation is defined about the local <span style="color: #2196F3; font-weight: bold;">Z</span> axis, so we cannot simply copy the world axes onto every part. Use this priority order when placing each body’s frame:

1. <span style="color: #2196F3; font-weight: bold;">Z</span> — joint rotation axis; align with +<span style="color: #4CAF50; font-weight: bold;">Y</span> of the world frame (left) when geometry allows.  
2. <span style="color: #4CAF50; font-weight: bold;">Y</span> — along the link, from parent toward the child.  
3. <span style="color: #F44336; font-weight: bold;">X</span> — forward (consistent with world +<span style="color: #F44336; font-weight: bold;">X</span> when possible).  

Apply the rules in order: satisfy <span style="color: #2196F3; font-weight: bold;">Z</span> first; if that is impossible (for example, the natural rotation axis is vertical or faces forward instead of left), relax <span style="color: #2196F3; font-weight: bold;">Z</span> and use the next rules to fix <span style="color: #4CAF50; font-weight: bold;">Y</span> and <span style="color: #F44336; font-weight: bold;">X</span> as well as the geometry permits.
