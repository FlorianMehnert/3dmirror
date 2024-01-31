# how to init
- workgroups should not depend on each other
- invokations can communicate with each other 

1. create shader object
2. feeding the source code
3. compiling
4. create proga
5. attach compute shader
6. link it

# how to run the compute  shader in OpenGl
```cpp
glUseProgram(computeProgram)
glDispatchCompute(1920, 1080, 1) // drückt aus wieviele einheiten insgesamt gebraucht werden -> wenn ein mit 1920x1080 bearbeitet werden soll auf pixelbasis, dann 1920, 1080, 1
// sobald das layout > 1,1,1 im compute shader sollte hier aber 1920/8, 1080/4, 1 stehen

// ALTERNATIVES dispatch compte für anderes layout
glDispatchCompute(ceil(1920/8), ceil(1080/4), 1)
glMemoryBarrier(GL_ALL_BARRIER_BITS) // specify to wait until finished -> gl_all_barrier is the most safe
```

# layout im compute shader
- für nvidia wavefront = 32 -> layout in im compute shader sollte ein vielfaches davon sein
```glsl
layout(local_size_x = 8, local_size_y = 4, local_size_z = 1) in;
layout(rgba32f, binding = 0) uniform image2D screen;
```
- sobald local_size != `(1,1,1)` sollte gl_GlobalInvocationID.xy verwendet werden anstelle von gl_GlobalWorkgroupID, da ID dreidimensional ist und Index linear -> index 10000 in 1920x1080 bsp vs id.xy von 100, 100, 1 
- `glGlobalInvocationID = gl_WorkgroupID * gl_WorkGroupSize + gl_LocalInvocationID`

# initialization in cgv framework
- **either** compute shader or pipeline shaders can be in the .pj file
```cpp
// create compute program using in built build_program -> creates, links etc.
if (!compute_prog.is_created())
	compute_prog.build_program(ctx, "compute_test.glpr", true);
glGenBuffers(1, &input_buffer);
glBindBuffer(GL_SHADER_STORAGE_BUFFER, input_buffer);
std::iota(std::begin(data), std::end(data), 1);
glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(data), data, GL_DYNAMIC_COPY);
glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, input_buffer);
```
## [`glGenBuffers(GLsize n, GLuint * buffers)`](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glGenBuffers.xhtml)
following command creates **n** buffer objects and stores them in the **buffers** array
```cpp
glGenBuffers(1, &input_buffer);
```

## [`glBufferData`]((https://registry.khronos.org/OpenGL-Refpages/gl4/html/glBufferData.xhtml))*`(GLenum target, GLsizeiptr size, const void * data, GLenum usage)`*
following command creates and initializes a buffer object's data store
```cpp
glBufferData(GL_SHADER_STORAGE_BUFFER, size_of_data, data, GL_STATIC_DRAW);
```
# sending data to be computed in the shader
```cpp
compute_prog.enable(ctx);
glDispatchCompute(sizeof(data), 1, 1);
glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
int* updatedData = (int*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(data), GL_MAP_READ_BIT);
glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
compute_prog.disable(ctx);
```

# retrieving data from compute shaders
## [`void *glMapNamedBufferRange( 	GLuint buffer, GLintptr offset, GLsizeiptr length,	GLbitfield access);`](https://registry.khronos.org/OpenGL-Refpages/gl4/html/glMapBufferRange.xhtml)
map all or part of a buffer object's data store into the client's address space

# working with multiple buffers
1. looks like we have to bind a buffer
2. map it into our adress space on the CPU
3. do something with data which is accessed at the pointer received from glMapBuffer
4. bind the buffer again
5. unmap the buffer
```cpp
glBindBuffer (GL_ARRAY_BUFFER, b0);
void *data0 = glMapBuffer (GL_ARRAY_BUFFER, GL_WRITE_ONLY);
glBindBuffer (GL_ARRAY_BUFFER, b1);
void *data1 = glMapBuffer (GL_ARRAY_BUFFER, GL_WRITE_ONLY);
glBindBuffer (GL_ARRAY_BUFFER, 0);

// do interesting and/or exciting stuff with data0 and data1

glBindBuffer (GL_ARRAY_BUFFER, b0);
glUnmapBuffer (GL_ARRAY_BUFFER);
glBindBuffer (GL_ARRAY_BUFFER, b1);
glUnmapBuffer (GL_ARRAY_BUFFER);
glBindBuffer (GL_ARRAY_BUFFER, 0);
```