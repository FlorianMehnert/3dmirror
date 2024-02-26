### 1. find out min and max range of distorted x/y corrdinates (distorted ~ as they come out of the kinect)
   - either based on counting all values, average out a reasonable window and return range based on pointcloud OR just basic slider for depth range
   - UNSURE ABOUT: either finding the area in which the distorted x/y coordinates are OR simply find the deepest distorted and the nearest distorted pixel
#### initially compute max and minimal values for points
- where are points passed 
#### current problems
- texture could be analysed from the cpu (bit slow but could be outsourced to compute shader - no succesful application yet)
- but this only returns the maximum and the minimum
2. *compute surface normals* (optional)
   - sliding window over distorted x/y coordinates 

3. compute kinect frustum and render with two triangles per side or with triangle strips (going with triangle strips)
   - **dynamically:** new renderer (just alter the cube renderer) origin is obvious (0,0,0) -> 4 coordinates for the outer image pixels
   - Question: how do I get the coordinates from the in GPU distorted pixels to the cpu (for the new renderer), does this add delay (since this is another renderer not related to pcl processing)
   - **statically:** (0,0,0) is still origin and frustum is maybe static calcuclation of max extension of frame

4. ==pass position in kinect coordidate system to fragment shader==

   - **question** (it was mentioned to pass position as texture ~ like a normal map): what position am I supposed to pass to the fragment shader?
     - position of kinect is (0,0,0) in its coordindate system
     - depth values is given by the coarse depth texture
     - **probably eye position is supposed to go to the fragment shader** -> where do we get the eye position from (look myself)
   - **question:** how do I convert the eye position into the kinect coordindate system -> fixed?

5. init previous sample type to no surface
   - start of ray marching: initialize a “ray” with no surface (which will later march and find either surface or no surface)

6. do marching - maybe missunderstood by me:  (look in github azure kinect)

   - undistort sample (depth value) ~ already done?

   - check whether uv is outside of domain (then type is no surface)

     - am I supposed to cut off everything outside of the square like this?

     <img src="./images/image-20240222131512402.png" alt="image-20240222131512402" style="zoom:25%;" />

     - check depth value exists for this UV (check for the undistorted color in the depth map)




fix the eye positions first

pass the depth image to the gpu

use the eye position as center and trace from each eye through the depth texture



- holo disp glf von holo raycast benutzen -> hat funktion der man eine eye position gibt rayorigin und ray direction (auge schaut immer gerade auf das display)