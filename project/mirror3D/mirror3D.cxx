#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/texture.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/dialog.h>
#include <rgbd_capture/rgbd_input.h>
#include <rgbd_render/rgbd_render.h>
#include <rgbd_render/rgbd_starter.h>
#include <rgbd_render/rgbd_point_renderer.h>
#include <rgbd_capture/rgbd_device.h>
#include <cgv_gl/box_wire_renderer.h>
#include <cgv_gl/rectangle_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/utils/pointer_test.h>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <holo_disp/shader_display_calibration.h>
#include <cgv/defines/quote.h>

// only temporary for variance of depth frame
#include <iostream>
#include <vector>
#include <cmath>

// specifically for mirror3D plugin
#include <frame.h>

// constructing pointclouds
#include "point_cloud/point_cloud.h"

// timer event for camera
#include <future>

using namespace cgv::render;
using namespace cgv;

struct Vertex {
	float x;
	float y;
	float z;
	unsigned int colors;
};

struct PointIndexBuffer {
	int i, j;
};

class rgbd_mesh_renderer :
	public rgbd::rgbd_point_renderer
{
public:
	bool build_shader_program(cgv::render::context& ctx, cgv::render::shader_program& prog, const cgv::render::shader_define_map& defines) override;
};

bool rgbd_mesh_renderer::build_shader_program(cgv::render::context& ctx, cgv::render::shader_program& prog, const cgv::render::shader_define_map& defines) {
		return prog.build_program(ctx, "mirror3D.glpr", true, defines);
}

class mirror3D :
	public rgbd::rgbd_starter<cgv::base::node>,
	public cgv::render::drawable,
	public cgv::gui::event_handler
{
	bool calib_outofdate = true;
protected:
	bool debug_frame_timing = false;
	cgv::render::stereo_view* stereo_view_ptr = nullptr;
	float view_test = 0.0f;
	bool debug_matrices = false;

	// render data
	std::vector<usvec3> sP;
	std::vector<rgb8> sC;
	
	// visualize calculated frustum extent
	vec3 lu, bd;
	float depth_calculated_frustum = 1.0f;

	// visualize truncated frustum
	float cam_x = 0.0f;
	float cam_y = 0.0f;
	float cam_color_offset = 0.0f;

	// show spheres that represent the left and right most
	bool visualize_eye_positions = true;

	// color warped to depth image is only needed in case CPU is used to lookup colors
	rgbd::frame_type warped_color_frame;

	// rendering configuration
	cgv::render::point_render_style prs;
	cgv::render::texture depth_tex, color_tex;
	cgv::render::box_wire_render_style box_wire_style;

	cgv::render::view* view_ptr = 0;

	// init intermediate pointcloud
	point_cloud &intermediate_pcl = point_cloud();
	
	// init current pointcloud
	point_cloud &current_pcl = point_cloud();

	// color and depth frames from kinect
	rgbd::frame_type depth_frame;
	rgbd::frame_type color_frame;

	// shader initializer
	cgv::render::shader_program prog;
	cgv::vec3 light_direction;
	holo_disp::holographic_display_calibration disp_calib;
	holo_disp::shader_display_calibration shader_calib;

	// texture, shaders and display lists
	cgv::render::texture color;

	rgbd_mesh_renderer pr;

	// regarding frustum rendering
	attribute_array_manager aam_frustum;
	std::vector<cgv::box3> boxes;
	std::vector<cgv::rgba> box_colors;
	//box_render_style brs;
	box_wire_render_style brs;
	float box_size = -0.999f;

	shader_program compute_prog;
	shader_program raycast_prog;

	// somthing for the timer event from vr_rgbd
	std::future<size_t> future_handle;

	// buffer to hold all vertices for the compute shader
	GLuint input_buffer;
	GLuint points;
	vec3 vertex_array[262144]; // 512 x 512
	uvec3 vres;
	int data[128];

	// compute shader buffers
	Vertex vertices[262144];

	// raycast compute shader
	GLuint vertexBuffer;
	GLuint resultBuffer;
	GLuint results2Buffer;

	// index buffer
	PointIndexBuffer PIB[262144];

	// element buffer object
	GLuint ebo;

	// shader shared buffer object creation
	GLuint buffer_id = 0;

	bool construct_quads = true;
	float distance = 5.0;
	float discard = 0.02f;
	float fdepth = 1.0f;
	bool render_quads = true;
	bool depth_lookup = false;
	bool flip_y = true;
	bool show_camera = false;
	bool show_calculated_frustum_size = false;
	bool do_raytracing = false;
	float view = 0;
	bool undistort_first = true;
	bool do_lookup_depth = false;
	mat23 iMV, MV;
	int bf_size = 10;

	float step_size = 0.1;
	int step = 0;

	enum ColorMode {
		COLOR_TEX_SM, NORMAL, BRUTE_FORCE, MIRROR, EXPERIMENT
	};
	ColorMode coloring = EXPERIMENT;

	// dispatch each time the button state is toggled
	bool one_tap_press = false;
	bool one_tap_flag = false;
	
	std::array<float, 4> camera_edges;
	std::array<cgv::vec3, 4> camera_corners;

public:
	mirror3D() : color_tex("uint8[R,G,B]"), depth_tex("uint16[R]")
	{	
		set_name("mirror3D");
		pr.set_distortion_map_usage(true);
		pr.set_geometry_less_rendering(true);
		pr.configure_invalid_color_handling(true, rgba(1, 0, 1, 1));
		prs.point_size = 5;
		prs.blend_width_in_pixel = 0;

		// mirror config
		light_direction = normalize(cgv::vec3(1.0f, 1.5f, 1.0f));
		if (!disp_calib.read(QUOTE_SYMBOL_VALUE(INPUT_DIR) "/visual.json"))
			std::cerr << "could not read holographic display calibration form <"
			<< QUOTE_SYMBOL_VALUE(INPUT_DIR) "/visual.json>" << std::endl;
		shader_calib.compute(disp_calib);

	}
	void on_set(void* member_ptr)
	{
		cgv::utils::pointer_test pt(member_ptr);
		//if (pt.one_of(..., ..., ...)) {
		//	...
		//}
		//else
		on_set_base(member_ptr, *this);

		update_member(member_ptr);
		post_redraw();
	}
	std::string get_type_name() const { return "mirror3D"; }
	
	void on_attach() {
		std::cout << "attached to " << rgbd_inp.get_serial() << std::endl;
	}

	void on_start() {
		std::cout << "started device " << rgbd_inp.get_serial() << std::endl;
		std::cout << "\x1b[1;31mcolor\x1b[0m" << std::endl;
		pr.set_calibration(calib);
		std::cout << calib.depth_scale << " ";
		std::cout << calib.depth.h << " ";
		std::cout << calib.depth.w << " ";
		std::cout << std::endl;
	}

	void on_new_frame(double t, rgbd::InputStreams new_frames) {
		auto now_h = std::chrono::high_resolution_clock::now();
		// construct copy of new frames
		if ((new_frames & rgbd::IS_COLOR) == rgbd::IS_COLOR)
			color_frame = rgbd_starter<node>::color_frame;
		if ((new_frames & rgbd::IS_DEPTH) == rgbd::IS_DEPTH)
			depth_frame = rgbd_starter<node>::depth_frame;
		if (debug_frame_timing) {
			static long long col_dev_ts = 0;
			static long long dev_ts = 0;
			std::cout << "frame: ";
			auto nw = std::chrono::duration_cast<std::chrono::nanoseconds>(now_h.time_since_epoch()).count();
			std::cout << ", color=" << (nw - color_frame.system_time_stamp) * 1e-6;
			//std::cout << "(" << color_frame.device_time_stamp << ")";
			std::cout << "(" << (color_frame.device_time_stamp - col_dev_ts) * 1e-6 << ")";
			col_dev_ts = color_frame.device_time_stamp;
			std::cout << ", depth=" << (nw - depth_frame.system_time_stamp) * 1e-6;
			//std::cout << "(" << (depth_frame.device_time_stamp- color_frame.device_time_stamp)*1e-6 << ")";
			std::cout << "(" << (depth_frame.device_time_stamp - dev_ts) * 1e-6 << ")";
			dev_ts = depth_frame.device_time_stamp;
			std::cout << " -> " << (depth_frame.device_time_stamp - color_frame.device_time_stamp) * 1e-6;
			std::cout << "depth frame ";
			std::cout << std::endl;

			// color_frame is using 4 bytes per pixel with 2048x1536 in BGR32 format - which wastes one color channel
			// depth_frame is using 2 bytes per pixel DEP16 - probably double precision integer
			std::cout << "color_frame_size:" << color_frame.buffer_size << "color_frame_resolution:" << color_frame << " depthframesize: " << depth_frame.buffer_size << "depth_frame_resolution:" << depth_frame << std::endl;
		}
		post_redraw();
	}

	void on_stop() {
		std::cout << "stopped device " << rgbd_inp.get_serial() << std::endl;
	}

	void stream_help(std::ostream& os)
	{
		os << "mirror3D: toggle capture with <Space>" << std::endl;
	}
	
	bool handle(cgv::gui::event& e)
	{
		if (e.get_kind() != cgv::gui::EID_KEY)
			return false;
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		const auto ka = ke.get_action();
		if (ke.get_action() == cgv::gui::KA_RELEASE)
			return false;
		switch (ke.get_key()) {
			case cgv::gui::KEY_Space:
				if (ke.get_modifiers() == 0) {
					on_set(&(is_running = !is_running));
					return true;
				}
				break;
			case 'C': 
				coloring = (int) coloring < 4 ? (ColorMode)(((int) coloring)+1) : COLOR_TEX_SM;
				return true;
			case 'X':
				coloring = (int) coloring > 0 ? (ColorMode)(((int)coloring) - 1) : EXPERIMENT;
				return true;
			case cgv::gui::KEY_Left: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
				shader_calib.eye_separation_factor -= 0.0625f;
				std::cout << "eye_separation_factor: " << shader_calib.eye_separation_factor << std::endl;
				on_set(&shader_calib.eye_separation_factor);
				return true;
			}
			case cgv::gui::KEY_Right: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
				shader_calib.eye_separation_factor += 0.0625f;
				std::cout << "eye_separation_factor: " << shader_calib.eye_separation_factor << std::endl;
				on_set(&shader_calib.eye_separation_factor);
				return true;
			}
		}

		return false;
	}

	void create_gui()
	{
		add_decorator("mirror3D", "heading", "level=1");
		add_member_control(this, "debug_frame_timing", debug_frame_timing, "check");
		add_member_control(this, "distance", distance, "value_slider", "min=0;max=10");
		add_member_control(this, "discard_dst", discard, "value_slider", "min=0;max=1;step=0.01");
		add_member_control(this, "frustum depth", fdepth, "value_slider", "min=0;max=10;step=0.01");
		add_member_control(this, "cam_x", cam_x, "value_slider", "min=0;max=2;step=0.01");
		add_member_control(this, "cam_y", cam_y, "value_slider", "min=0;max=2;step=0.01");
		add_member_control(this, "depth of frustum", depth_calculated_frustum, "value_slider", "min=0;max=8;step=0.01");
		add_member_control(this, "construct quads", construct_quads, "check");
		add_member_control(this, "one time execution", one_tap_press, "toggle");
		add_member_control(this, "render quads", render_quads, "check");
		add_member_control(this, "show camera position", show_camera, "check");
		add_member_control(this, "show calculated frustum size", show_calculated_frustum_size, "check");
		add_member_control(this, "cull mode", coloring, "dropdown", "enums='color,normals,brute-force,mirror, experiment'");
		add_member_control(this, "undistort_first", undistort_first, "check");
		add_member_control(this, "do lookup depth", do_lookup_depth, "check");
		add_member_control(this, "Eye Separation Factor", shader_calib.eye_separation_factor, "value_slider", "min=0;max=20;ticks=true");
		add_member_control(this, "Debug Matrices", debug_matrices, "check");
		add_member_control(this, "Interpolate View Matrix", shader_calib.interpolate_view_matrix, "check");
		add_member_control(this, "View Eye Positions", visualize_eye_positions, "check");
		add_member_control(this, "bf_size", bf_size, "value_slider", "min=0;max=100;step=1");
		add_member_control(this, "rc step size", step_size, "value_slider", "min=0;max=2;step=0.01");
		add_member_control(this, "rc iterations", step, "value_slider", "min=0;max=20;step=1");
		add_member_control(this, "current_view", view, "value_slider", "min=-1;max=1;step=.01");

		if (begin_tree_node("capture", is_running)) {
			align("\a");
			create_gui_base(this, *this);
			align("\b");
			end_tree_node(is_running);
		}
		if (begin_tree_node("point style", prs)) {
			align("\a");
			pr.create_gui(this, *this);
			add_gui("point style", prs);
			align("\a");
			end_tree_node(prs);
		}

		if (begin_tree_node("box style", brs)) {
			align("\a");
			create_gui_base(this, *this);
			add_gui("box style", brs);
			align("\a");
			end_tree_node(prs);
		}
		
	}
	bool init(cgv::render::context& ctx)
	{
		start_first_device();
		ctx.set_bg_clr_idx(4);
		aam_frustum.init(ctx);

		// render points
		cgv::render::ref_point_renderer(ctx, 1);
		cgv::render::ref_box_wire_renderer(ctx, 1);
		cgv::render::ref_rectangle_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);
		
		init_dummy_compute_shader(ctx);
		//init_raycast_compute_shader(ctx);

		return pr.init(ctx);
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, -1);
		cgv::render::ref_box_wire_renderer(ctx, -1);
		cgv::render::ref_rectangle_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);

		pr.clear(ctx);
		if (is_running) {
			is_running = false;
			on_set_base(&is_running, *this);
		}
		aam_frustum.destruct(ctx);
		glDeleteBuffers(1, &input_buffer);
		glDeleteBuffers(1, &points);
		glDeleteBuffers(1, &vertexBuffer);
		glDeleteBuffers(1, &resultBuffer);
		glDeleteBuffers(1, &results2Buffer);
	}

	// prints errors in debug builds if shader code is wrong
		void shaderCheckError(cgv::render::shader_program & prog, const char name[]) {
		#ifndef NDEBUG
		if (prog.last_error.size() > 0) {
			std::cerr << "error in " << name << "\n" << prog.last_error << '\n';
			prog.last_error = "";
		}
		#endif // #ifdef NDEBUG
	}

	void setup_compute_shader(cgv::render::context& ctx)
	{
		// enable, configure, run and disable program (see gradient_viewer.cxx in example plugins)

		// see cgv_gl clod_point_renderer
		if (!prog.build_program(ctx, "mirror3D.glpr", true)) {
			std::cerr << "ERROR in setup_compute_shader::init() ... could not build program" << std::endl;
		}
	}

	void init_dummy_compute_shader(context& ctx) {
		// TODO: delete Buffer
		if (!compute_prog.is_created())
			compute_prog.build_program(ctx, "compute_test.glpr", true);
		glGenBuffers(1, &input_buffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, input_buffer);
		std::iota(std::begin(data), std::end(data), 1);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(data), data, GL_DYNAMIC_COPY);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, input_buffer);
	}

	void update_dummy_compute_shader(cgv::render::context& ctx) {
		
		// calculate
			compute_prog.enable(ctx);
		glDispatchCompute(sizeof(data), 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

		// retrieve
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, input_buffer);
		int* updatedData = (int*)glMapNamedBufferRange(input_buffer, 0, sizeof(data), GL_MAP_READ_BIT);
		compute_prog.disable(ctx);
		for (int i = 0; i < 5; ++i) {
			int value = updatedData[i];
			std::cout << "Element " << i << ": " << value << std::endl;
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	void truncated_pyramid(cgv::render::context& ctx, float base_x, float base_y, float color_cam_offset, float height) {
		int segments = 4;
		float angle_step = 2.0f * M_PI / segments;
		std::array<vec3,8> vertices;

		for (int i = 0; i < segments; ++i) {
			float angle = i * angle_step;
			vertices[i] = vec3(0);
		}

		// generate vertices for top extent and rotate by 45 degrees
		for (int i = 0; i < segments; ++i) {
			float angle = i * angle_step;
			float x = cos(angle);
			float y = sin(angle);
			vertices[i + 4] = vec3(x * cos(M_PI / 4.0f) - y * sin(M_PI / 4.0f), x * sin(M_PI / 4.0f) + y * cos(M_PI / 4.0f), height);
		}
		vec3 x_vector = vertices[5] - vertices[4];
		vec3 y_vector = vertices[6] - vertices[5];

		vertices[4] = vertices[4] - x_vector * base_x - y_vector * base_y + y_vector * color_cam_offset;
		vertices[5] = vertices[5] + x_vector * base_x - y_vector * base_y + y_vector * color_cam_offset;
		vertices[6] = vertices[6] + x_vector * base_x + y_vector * base_y + y_vector * color_cam_offset;
		vertices[7] = vertices[7] - x_vector * base_x + y_vector * base_y + y_vector * color_cam_offset;

		static int F[4 * 4] = {
			0, 1, 5, 4,
			1, 2, 6, 5,
			2, 3, 7, 6,
			3, 0, 4, 7,
		};

		static int FN[6 * 4];
		static float N[6 * 3];
		
		float V[8 * 3] = {
			vertices[0][0], vertices[0][1], vertices[0][2],
			vertices[1][0], vertices[1][1], vertices[1][2],
			vertices[2][0], vertices[2][1], vertices[2][2],
			vertices[3][0], vertices[3][1], vertices[3][2],
			vertices[4][0], vertices[4][1], vertices[4][2],
			vertices[5][0], vertices[5][1], vertices[5][2],
			vertices[6][0], vertices[6][1], vertices[6][2],
			vertices[7][0], vertices[7][1], vertices[7][2],
		};

		ctx.draw_edges_of_faces(V, N, 0, F, FN, 0, 4, 4, false);
	}

	 bool calculate_inverse_modelview_matrix_rgbd_depth() {
		// get: MV matrix of depth camera
		mat23 MV_depth = calib.depth.get_camera_matrix();

		// Check if the matrix is invertible (determinant not equal to zero) + inverse calculation (there exist functions for that in GPU)
		float detA = MV_depth(0, 0) * MV_depth(1, 1) - MV_depth(0, 1) * MV_depth(1, 0);
		if (detA != 0) {
			iMV(0, 0) = MV_depth(1, 1) / detA;
			iMV(0, 1) = -MV_depth(0, 1) / detA;
			iMV(0, 2) = (MV_depth(0, 1) * MV_depth(1, 2) - MV_depth(1, 1) * MV_depth(0, 2)) / detA;
			iMV(1, 0) = -MV_depth(1, 0) / detA;
			iMV(1, 1) = MV_depth(0, 0) / detA;
			iMV(1, 2) = -(MV_depth(0, 0) * MV_depth(1, 2) - MV_depth(1, 0) * MV_depth(0, 2)) / detA;
			return true;
		}
		return false;
	}

	 // since pose - external calibration is identity matrix the projection matrix which is EM * IM is equivalent to IM
	 mat4 projection_matrix_kinect_depth() {
		 mat4 IM = calib.depth.get_homogeneous_camera_matrix();
		 return IM;
	 }

	void init_frame(cgv::render::context& ctx)
	{
			if (!view_ptr)
				view_ptr = find_view_as_node();
			if (color_frame_changed) {
				create_or_update_texture_from_frame(ctx, color_tex, color_frame);
				color_frame_changed = false;
			}
			if (depth_frame_changed) {
				create_or_update_texture_from_frame(ctx, depth_tex, depth_frame);
			}
			if (!pr.do_geometry_less_rendering()) {
				if (!pr.do_lookup_color()) {
					rgbd_inp.map_color_to_depth(depth_frame, color_frame, warped_color_frame);
					rgbd::construct_rgbd_render_data_with_color(depth_frame, warped_color_frame, sP, sC);
				}
				else
					// TODO: use own function to avoid pushback : alternatively use own construct function (without pushback)
					rgbd::construct_rgbd_render_data(depth_frame, sP);
			}
			if (!stereo_view_ptr)
				stereo_view_ptr = dynamic_cast<cgv::render::stereo_view*>(find_view_as_node());
			
			// show frustum
			rgbd_inp.map_depth_to_point(0, 0, depth_calculated_frustum*1000, lu);
			rgbd_inp.map_depth_to_point(512, 512, depth_calculated_frustum*1000, bd);

	}
	// I have a low resolution geometry created from a low res depth texture.I also have a color texture available which I want to use to color.The color texture has a higher resolution.How do I map the color in the fragment shader ?
	void draw(cgv::render::context& ctx)
	{
		if (!stereo_view_ptr)
			return;
		if (one_tap_press != one_tap_flag) {
			update_dummy_compute_shader(ctx);
			if (sP.size() > 0) {
				std::cout << "sizeof elements in sP " << sizeof(sP[0]) << ", and size of sP: " << sP.size() << std::endl;
				std::cout << "sizeof Vertex " << sizeof(Vertex) << std::endl;
			}
			one_tap_flag = one_tap_press;
		}
		if (!pr.ref_prog().is_linked())
			return;
		if (!depth_tex.is_created())
			return;

		glDisable(GL_CULL_FACE);
		if (calib_outofdate) {
			pr.set_calibration(calib);
			calib_outofdate = false;
		}

		ctx.ref_surface_shader_program().enable(ctx);
		ctx.push_modelview_matrix();
		vec3 point_between_eyes = shader_calib.left_eye + vec3(0.5) * (shader_calib.right_eye - shader_calib.left_eye);
		ctx.mul_modelview_matrix(cgv::math::translate4(point_between_eyes));
		ctx.pop_modelview_matrix();
		ctx.ref_surface_shader_program().disable(ctx);

		if (view_ptr)
			pr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		pr.set_render_style(prs);
		if (!pr.do_geometry_less_rendering())
			pr.set_position_array(ctx, sP);
		if (!pr.do_lookup_color())
			pr.set_color_array(ctx, sC);
		if (pr.validate_and_enable(ctx)) {
			depth_tex.enable(ctx, 0);
			if (pr.do_lookup_color())
				color_tex.enable(ctx, 1);
			//if (pr.do_geometry_less_rendering());
			pr.ref_prog().set_uniform(ctx, "depth_image", 0);
			pr.ref_prog().set_uniform(ctx, "color_image", 1);
			pr.ref_prog().set_uniform(ctx, "max_distance", distance);
			pr.ref_prog().set_uniform(ctx, "discard_dst", discard);
			pr.ref_prog().set_uniform(ctx, "construct_quads", construct_quads);
			pr.ref_prog().set_uniform(ctx, "render_quads", render_quads);
			pr.ref_prog().set_uniform(ctx, "coloring", (int)coloring);
			pr.ref_prog().set_uniform(ctx, "depth_in_which_to_lookup", fdepth);
			pr.ref_prog().set_uniform(ctx, "do_lookup_depth", do_lookup_depth);
			pr.ref_prog().set_uniform(ctx, "undistort_first", undistort_first);
			shader_calib.set_uniforms(ctx, pr.ref_prog(), *stereo_view_ptr);
			calculate_inverse_modelview_matrix_rgbd_depth();
			pr.ref_prog().set_uniform(ctx, "P_kinect", projection_matrix_kinect_depth());
			pr.ref_prog().set_uniform(ctx, "eye_separation", shader_calib.eye_separation_factor);
			pr.ref_prog().set_uniform(ctx, "bf_size", bf_size);
			pr.draw(ctx, 0, sP.size()); // only using sP size with geometryless rendering
			pr.disable(ctx);
			if (pr.do_lookup_color())
				color_tex.disable(ctx);
			if (pr.do_geometry_less_rendering())
				depth_tex.disable(ctx); 
			}

		if (show_camera) {
			ctx.ref_surface_shader_program().enable(ctx);
			ctx.push_modelview_matrix();
			ctx.set_color(cgv::rgb(0, 1, 0.2f));
			truncated_pyramid(ctx, cam_x, cam_y, cam_color_offset / 10, fdepth);
			ctx.pop_modelview_matrix();
			ctx.ref_surface_shader_program().disable(ctx);
		}
		if (show_calculated_frustum_size) {
			auto& bwr = ref_box_wire_renderer(ctx);
			vec3 p1;
			vec3 p2;

			cgv::box3 box(lu, bd);
			bwr.set_box(ctx, box);
			box_wire_style.default_color = rgb(0, 0, 0);
			bwr.set_render_style(box_wire_style);
			bwr.render(ctx, 0, 1);
		}

		if (visualize_eye_positions) {
			auto& sr1 = ref_sphere_renderer(ctx);
			auto& sr2 = ref_sphere_renderer(ctx);
			sr1.set_radius(ctx,.05f);
			sr1.set_position(ctx, shader_calib.left_eye/2);
			sr1.render(ctx, 0, 1);

			sr2.set_position(ctx, shader_calib.right_eye / 2);
			sr2.set_radius(ctx, .05f);

			sr2.render(ctx, 0, 1);

			// define sample (which would be ro + rd)
			vec3 sample = cgv::math::lerp(shader_calib.left_eye/2, shader_calib.right_eye/2, view) + step * step_size * vec3(0, 0, 1.0f);
			dvec2 mapped_sample;
			
			// map sample into space of depth camera: iMV * sample -> apply_distortion_model()
			calib.depth.apply_distortion_model(dvec2(iMV * sample), mapped_sample);

			if (mapped_sample[0] < -1.0 || mapped_sample[0] > 1.0 || mapped_sample[1] > 1.0 || mapped_sample[1] < -1.0) {
			}
			else {
				auto& sr3 = ref_sphere_renderer(ctx);
				auto& sr4 = ref_sphere_renderer(ctx);
				uint16_t depth = reinterpret_cast<const uint16_t&>(depth_frame.frame_data[((mapped_sample[1]*256+256) * depth_frame.width + (mapped_sample[0]*256+256)) * depth_frame.get_nr_bytes_per_pixel()]);
				sr3.set_position(ctx, vec3(sample));
				rgb8 looked_up_color = rgb8(255, 0,0);
				bool inside_frame = rgbd::lookup_color(vec3(mapped_sample, calib.depth_scale* depth), looked_up_color, color_frame, calib);
				std::vector<rgb8> colors;
				colors.push_back(looked_up_color);
				sr3.set_color_array(ctx, colors);
				sr3.set_radius(ctx, .05f);
				sr3.render(ctx, 0, 1);
				sr4.set_position(ctx, vec3(mapped_sample, step*step_size));
				sr4.set_radius(ctx, .05f);
				sr4.render(ctx, 0, 1);
				projection_matrix_kinect_depth();
			}
		}
			glEnable(GL_CULL_FACE);
		}
		
};

#include <cgv/base/register.h>

cgv::base::object_registration<mirror3D> mirror_reg("");
