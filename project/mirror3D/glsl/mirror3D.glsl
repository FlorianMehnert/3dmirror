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

uniform sampler2D depth_image;
uniform calibration depth_calib;

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

vec4 kinect_to_world_space(vec2 uv) {
	// Get the depth value from the texture
	float depth = texture(depth_image, uv).r;

	// Convert UV coordinates to normalized device coordinates
	vec4 ndc = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);

	// Apply inverse projection matrix to get camera space position
	vec4 cameraPos = inverse(kinect_projection_matrix(521.492, 251.568, 255.251, 258.7, 512, 0, 20000)) * ndc;

	// Normalize camera space position
	cameraPos.xyz = cameraPos.xyz / cameraPos.w;

	// Convert camera space position to world space
	vec4 worldPos = cameraPos;

	// Output world position
	return vec4(worldPos.xyz, 1.0);
}

// takes world position and returns normalized device coordinates in kinect space
// WORLD -> NDC (Kinect) -> UNDISTORT
vec2 world_to_kinect_space(vec3 world_pos) {
	vec4 position_kinect, position_ndc;
	position_kinect = vec4(world_pos, 1.0);; // apply inverse modelview aka extrinsic parameters (static since kinect is origin of world_space)
	position_ndc = inverse(kinect_projection_matrix(521.492, 251.568, 255.251, 258.7, 512, 0, 20000)) * position_kinect; // apply inverse projection_matrix - aka intrinsic parameters
	vec2 position_undistorted;
	mat2 J;
	apply_distortion_model(position_ndc.xy, position_undistorted, J, depth_calib);

	// TODO handle cases where distortion model fails
	return position_undistorted;
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

// map the ray sample to the uv of the depth texture output by the kinect using undistortion
bool map_sample_to_uv(in vec3 ro, out vec2 xd) {

	mat2 J;
	vec2 xu;
	// take distorted and output undistorted with jacobian matrix aswell as return code of either SUCCESS, OUT_OF_BOUNDS or DIVISION_BY_ZERO
	int dir;
	dir = apply_distortion_model(xd, xu, J, depth_calib);
	return dir == SUCCESS;
}

// perform marching in kinect space
void march_along(vec3 ro, vec3 rd, out vec4 color, out float depth) {
	const int max_steps = 100;
	const float max_distance = 100.0;
	vec3 sample_point, p;
	float projected_ray_depth, ray_depth;
	bool prev_sample_type = false; // means no surface
	bool curr_sample_type = false;
	bool is_behind = false;

	float depth_threshold = texture(depth_image, gl_FragCoord.xy / vec2(textureSize(depth_image, 0))).r;
	float t = 0.0; // stepsize
	depth = 0.0;

	for (int i = 0; i < max_steps; i++) {
		sample_point = ro + rd * t; // ro is in kinect coordinate system
		p = vec3(0.0);

		if (sample_point.x < -1 || sample_point.x > 1 || sample_point.y < -1 || sample_point.y > 1) { // 2. check if out of bounds
			color = vec4(1.0f, 0, 1.0f, 1.0f);
			depth = 0.0;
			curr_sample_type = false;
		}
		else if (construct_point(sample_point.xy, lookup_depth(ivec2(sample_point.xy)), p)) { // 3. check if depth is invalid
			// lookup_color
			if (!lookup_color(p, color)) {
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}
			curr_sample_type = false;
		}

		projected_ray_depth = lookup_depth(ivec2(sample_point.xy));
		ray_depth = sample_point.z; // maybe multiplied by the depth distance

		if (ray_depth > projected_ray_depth) { // 4. marched under depth texture
			is_behind = true;
		}

		// 5.still need to check if prev type and current type differ and both are not no surface

		if (depth < depth_threshold || t > max_distance) {
			break;
		}

		// 6. check if still in frustum ? - check frustum intersection from 0,0,0 in kinect space with 0,0 to 512,512 with calculated depth

		t += 0.01; // maybe adjust step size
	}

	if (!lookup_color(sample_point.xyz, color)) {
		color = vec4(1.0, 0.0, 0.0, 1.0);
	}
	// 10. still compute corrected depth value with respect to eye
	// 11. output depth and color value and exit fragment shader
}