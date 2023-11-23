#pragma once
#include <cgv/render/shader_program.h>


std::vector<cgv::render::render_types::ivec3> M_TRIANGLES;
std::vector<cgv::render::render_types::vec3> M_POINTS;
std::vector<cgv::render::render_types::vec2> M_UV;

// shader
cgv::render::shader_program rgbd_prog;

// texture, shaders and display lists
cgv::render::texture color;