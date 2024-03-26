#version 330 core

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

vec3 ray_trace_pixel(vec3 ro, vec3 rd, out float depth) {
	vec3 pos, nor; // hit position and normal
	vec3 col;
	int mat_id; // material ID
	//float t = intersect(ro, rd, pos, nor, mat_id);
	float t = 0.1;
	col = vec3(1.0, 0, 1.0);
	if (mat_id != -1) {
		// get color and texture coordinates depending on material
		vec4 material = vec4(1.0, 0.1, 0.1, 0.05); // add actual color calculation probabily just take given color

		/*
		somehow find out what the color is we collided with
		vec2 uv = get_texcoords(pos, mat_id);
		*/

		vec3 albedo = material.rgb;

		// lighting - probably not to be hardcoded :)
		//color = compute_illumination(pos, nor, rd, shadow_factor, albedo);
		col = vec3(1.0, 0.1, 0.1);
	}

	vec4 pos_clip = get_modelview_projection_matrix() * vec4(pos, 1.0);
	depth = 0.5 * (pos_clip.z / pos_clip.w) + 0.5;
	return col;
}

mat4 kinect_projection_matrix(float fx, float fy, float cx, float cy, int image_height, float near_clip, float far_clip) {

	float fovy, f, z0, z1;
	fovy = 2.0 * atan(0.5 * image_height / fy);

	f = 1.0 / tan(fovy / 2.0);
	z0 = -(far_clip + near_clip) / (far_clip - near_clip);
	z1 = -2.0 * far_clip * near_clip / (far_clip - near_clip);

	return mat4(
		f, 0, 0, 0,
		0, f, 0, 0,
		0, 0, z0, z1,
		0, 0, -1, 0
	);
}

// returns modelview matrix for only rotational transforms e.g. eye positions
mat4 get_model_view_matrix_translation(vec3 ro) {
	return mat4(
		vec4(1.0, 0.0, 0.0, 0.0),
		vec4(0.0, 1.0, 0.0, 0.0),
		vec4(0.0, 0.0, 1.0, 0.0),
		vec4(ro, 1.0)
	);
}