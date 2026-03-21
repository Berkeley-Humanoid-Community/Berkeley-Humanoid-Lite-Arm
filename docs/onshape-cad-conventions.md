# Onshape CAD Modeling Conventions

## Frame orientation

**World frame** (shared reference for the assembly):

- <span style="color: #F44336; font-weight: bold;">X</span> — forward  
- <span style="color: #4CAF50; font-weight: bold;">Y</span> — left  
- <span style="color: #2196F3; font-weight: bold;">Z</span> — up  

**Local frames** (per body / link): In Onshape, joint rotation is defined about the local <span style="color: #2196F3; font-weight: bold;">Z</span> axis, so we cannot simply copy the world axes onto every part. Use this priority order when placing each body’s frame:

1. <span style="color: #2196F3; font-weight: bold;">Z</span> — joint rotation axis; align with +axes of the world frame.
2. <span style="color: #4CAF50; font-weight: bold;">Y</span> — along the link, from parent toward the child.  
3. <span style="color: #F44336; font-weight: bold;">X</span> — forward (consistent with world +<span style="color: #F44336; font-weight: bold;">X</span> when possible).  

Apply the rules in order: satisfy <span style="color: #2196F3; font-weight: bold;">Z</span> first, and <span style="color: #4CAF50; font-weight: bold;">Y</span>, lastly <span style="color: #F44336; font-weight: bold;">X</span>. If in some condition it is impossible to achieve (for example, the rotation axis aligns with parent-child axis), relax <span style="color: #4CAF50; font-weight: bold;">Y</span> and use the next rules to fix <span style="color: #F44336; font-weight: bold;">X</span> as well as the geometry permits.
