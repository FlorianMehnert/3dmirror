#version 330 core

/*
The following interface is implemented in this shader:
// ***** begin interface of mirror3D.glsl *****************************
vec3 ray_trace_pixel(vec3 ro, vec3 rd, out float depth);
mat4 kinect_projection_matrix(float fx, float fy, float cx, float cy, int image_height, float near_clip, float far_clip);
mat4 get_model_view_matrix_translation(vec3 ro);
void march_along(vec3 ro, vec3 rd, out vec4 color, out float depth);
vec4 kinect_to_world_space(vec2 uv);
// ***** end interface of mirror3D.glsl *****************************
*/

// ***** begin interface of camera.glsl *******************************
#define CONVERGENCE 0
#define SUCCESS 0
uniform float inversion_epsilon;
struct calibration
{
	int w;
	int h;
	float max_radius_for_projection;
	vec2 dc;
	float k[6];
	float p[2];
	float skew;
	vec2 c, s;
};
vec2 image_to_pixel_coordinates(in vec2 x, in calibration calib);
vec2 pixel_to_image_coordinates(in vec2 p, in calibration calib);
vec2 texture_to_pixel_coordinates(in vec2 t, in calibration calib);
vec2 pixel_to_texture_coordinates(in vec2 p, in calibration calib);
int apply_distortion_model(in vec2 xd, out vec2 xu, out mat2 J, in calibration calib);
int apply_distortion_model(in vec2 xd, out vec2 xu, out mat2 J, float epsilon, in calibration calib);
int invert_distortion_model(in vec2 xu, inout vec2 xd, bool use_xd_as_initial_guess, in calibration calib);
// ***** end interface of camera.glsl *********************************

// ***** begin interface of view.glsl *********************************
mat4 get_modelview_matrix();
mat4 get_projection_matrix();
mat4 get_inverse_projection_matrix();
mat4 get_modelview_projection_matrix();
mat4 get_inverse_modelview_matrix();
mat4 get_inverse_modelview_projection_matrix();
mat3 get_normal_matrix();
mat3 get_inverse_normal_matrix();
// ***** end interface of view.glsl ***********************************

// ***** begin interface of rgbd.glsl *********************************
uint get_depth_width();
uint get_depth_height();
uint lookup_depth(ivec2 xp);
bool construct_point(in vec2 xp, in float depth, out vec3 p);
bool lookup_color(vec3 p, out vec4 c);
bool lookup_color(vec3 p, float eps, out vec4 c);
// ***** end interface of rgbd.glsl ***********************************

// ***** begin interface of holo_disp.glfs ****************************
void compute_sub_pixel_rays(out vec3 ro[3], out vec3 rd[3]);
bool finalize_sub_pixel_fragment(in vec3 rgb, in vec3 depth);
void stereo_translate_modelview_matrix(in float eye, in out mat4 M);
float view_from_fragment_component (const vec2 frag, const int component);
// ***** end interface of holo_disp.glfs ******************************

uniform sampler2D depth_image;
uniform sampler2D color_image;
uniform calibration depth_calib;
uniform calibration color_calib;
uniform vec2 viewport_dims;
uniform mat3 color_rotation;
uniform vec3 color_translation;
uniform float depth_scale;
uniform int bf_size;

mat4 get_modelview_matrix_view(int color_channel){
		// reusing holo_disp for correct matrices MV and P
		vec2 uv = gl_FragCoord.xy / viewport_dims;
		float view = view_from_fragment_component(uv, color_channel);
		mat4 MV = get_modelview_matrix();
		stereo_translate_modelview_matrix(view, MV);
		return MV;
}

// ***** start of intersecting pixel ***** 
float point_in_or_on(vec3 p1, vec3 p2, vec3 a, vec3 b){
	vec3 cp1 = cross(b-a, p1 - a);
	vec3 cp2 = cross(b-a, p2 - a);
	return step(0.0, dot(cp1, cp2));
}

bool point_in_triangle(vec3 px, vec3 p0, vec3 p1, vec3 p2){
	return bool(point_in_or_on(px, p0, p1, p2) * point_in_or_on(px, p1, p2, p0) * point_in_or_on(px, p2, p0, p1));
}

struct Ray{
	vec3 origin;
	vec3 direction;
};

vec3 intersect_plane(Ray ray, vec3 p0, vec3 p1, vec3 p2){
	vec3 d = ray.direction;
	vec3 n = cross(p1-p0,p2-p0);
	vec3 x = ray.origin + d*dot(p0-ray.origin, n)/dot(d,n);
	return x;
}

bool triangle_intersection(Ray ray, vec3 p0, vec3 p1, vec3 p2){
	vec3 x = intersect_plane(ray, p0, p1, p2);
	return point_in_triangle(x, p0, p1,p2);
}

bool intersect_quad(Ray ray, vec3 p0, vec3 p1, vec3 p2, vec3 p3){
	return triangle_intersection(ray, p0, p2, p1) || triangle_intersection(ray, p0, p3, p2);
}

float get_depth(ivec2 xp){
	return 65535.0 * texture(depth_image, pixel_to_texture_coordinates(xp, depth_calib)).r;
}

// costruct pixel point
vec3 cpp(ivec2 xp){
	float depth = get_depth(xp);
	vec3 p;
	construct_point(xp, depth, p);
	return p;
}

bool intersect_pixel(Ray ray, ivec2 xp){
	return intersect_quad(ray, cpp(xp), cpp(xp+ivec2(0,1)), cpp(xp+ivec2(1,1)), cpp(xp+ivec2(1,0)));;
}
// ***** end of intersecting pixel ***** 

// brute force approach - no raymarching pure iterating over depth image
vec4 intersect_depth_image_bf(){
	vec3 ro[3], rd[3];
	vec4 final_color = vec4(1,0,1,1);
	compute_sub_pixel_rays(ro, rd);
	for (int c = 0; c < 3; c++){
		for (int i = 512/2 - bf_size; i<512/2 + bf_size; i++){
			for (int j = 512/2 - bf_size; j<512/2 + bf_size; j++){
				if (intersect_pixel(Ray(ro[c],rd[c]), ivec2(j,i))){
					lookup_color(cpp(ivec2(j,i)), final_color);
					final_color[c] = final_color[c];
				}
			}
		}
	}
	return final_color;
}

vec3 vec3_ray(Ray ray) {
	return ray.origin + ray.direction;
}

void set_raylength(Ray inray, out Ray outray, int s, float step_width) {
	outray.origin = inray.origin;
	outray.direction = inray.direction * s * step_width;
}

vec4 pixel_coordinates_to_color(vec2 xp) {
	vec3 p;
	vec4 color;
	construct_point(ivec2(xp), lookup_depth(ivec2(xp)), p);
	lookup_color(p, color);
	return color;
}

mat2 rotate_2d(float rotation) {
	return mat2(cos(rotation), -sin(rotation),
		sin(rotation), cos(rotation));
}

bool inverse_construct_point(in vec3 p, out vec2 xp, out float depth) {
	mat2 J;
	vec2 xu, xd;
	xp = p.xy;
	xu = xp;
	int result = apply_distortion_model(xp, xu, J, depth_calib);
	xd = image_to_pixel_coordinates(xp, depth_calib);
	if (result != SUCCESS) {
		return false;
	}
	float depth_m = p.z/depth_scale;
	vec3 kinect_point = vec3(xu / depth_m, depth_m);
	xp = kinect_point.xy;
	depth = kinect_point.z;
	return true;
}

bool inverse_construct_point_v2(in vec3 p, out vec2 xp)
{
	mat2 J;
	vec2 xu, xd;
	int result;
	float depth_m;
	
	depth_m = p.z;
	xd = p.xy / depth_m;
	result = apply_distortion_model(xd, xu, J, depth_calib);
	if (result != SUCCESS)
		return false;
	xp = image_to_pixel_coordinates(xu, depth_calib);
	return true;
}
