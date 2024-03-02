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
#include <cgv_gl/box_renderer.h>
#include <cgv/utils/pointer_test.h>
#include <chrono>
#include <numeric>
#include <algorithm>

// only temporary for variance of depth frame
#include <iostream>
#include <vector>
#include <cmath>

// specifically for mirror3D plugin
#include <mirror3D.h>
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
	
	// render data
	std::vector<usvec3> sP;
	std::vector<rgb8> sC;

	// color warped to depth image is only needed in case CPU is used to lookup colors
	rgbd::frame_type warped_color_frame;

	// rendering configuration
	cgv::render::point_render_style prs;
	cgv::render::texture depth_tex, color_tex;

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

	// texture, shaders and display lists
	cgv::render::texture color;

	rgbd_mesh_renderer pr;
	box_renderer br;

	// regarding frustum rendering
	attribute_array_manager aam_frustum;
	std::vector<cgv::box3> boxes;
	std::vector<cgv::rgba> box_colors;
	box_render_style brs;
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

	bool shader_demo = true;
	bool construct_quads = true;
	float distance = 5.0;
	float discard = 0.02f;
	bool render_quads = true;
	bool depth_lookup = false;
	bool flip_y = true;
	bool simple_cube = true;

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
		if (ke.get_action() == cgv::gui::KA_RELEASE)
			return false;
		switch (ke.get_key()) {
		case cgv::gui::KEY_Space:
			if (ke.get_modifiers() == 0) {
				on_set(&(is_running = !is_running));
				return true;
			}
			break;
		}
		return false;
	}

	void create_gui()
	{
		add_decorator("mirror3D", "heading", "level=1");
		add_member_control(this, "debug_frame_timing", debug_frame_timing, "check");
		add_member_control(this, "shader demo", shader_demo, "check");
		add_member_control(this, "distance", distance, "value_slider", "min=0;max=10");
		add_member_control(this, "discard_dst", discard, "value_slider", "min=0;max=1;step=0.01");
		add_member_control(this, "construct quads", construct_quads, "check");
		//add_member_control(this, "flip y", flip_y, "toggle", "w=66", " "); // should be doable in config file
		add_member_control(this, "one time execution", one_tap_press, "toggle");
		add_member_control(this, "render quads", render_quads, "check");
		add_member_control(this, "show camera position", simple_cube, "check");
		
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
		auto &R = cgv::render::ref_box_renderer(ctx, 1);
		
		init_dummy_compute_shader(ctx);
		//init_raycast_compute_shader(ctx);

		return pr.init(ctx) && br.init(ctx);
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);

		pr.clear(ctx);
		br.clear(ctx);
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
	void init_raycast_compute_shader(context& ctx) {
		if (!raycast_prog.is_created())
			raycast_prog.build_program(ctx, "azure_raycast_test.glpr", true);
		std::cout << (raycast_prog.is_created() ? "raycast_prog is created" : "raycast_prog is not created") << std::endl;
		std::cout << (raycast_prog.is_linked() ? "raycast_prog is linked" : "raycast_prog is not linked") << std::endl;
		
		std::cout << "after build" << std::endl;

		glGenBuffers(1, &vertexBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, vertexBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Vertex) * 512 * 512, nullptr, GL_STATIC_DRAW);

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, vertexBuffer);

		glGenBuffers(1, &resultBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, resultBuffer);

		// 8192 is 512 * 512 / 32 for depth image size divided by workload
		// only read since this is a result buffer
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Vertex)*2048, nullptr, GL_DYNAMIC_READ);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, resultBuffer);
		glGenBuffers(1, &results2Buffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, results2Buffer);
		// uint + float * work group
		glBufferData(GL_SHADER_STORAGE_BUFFER, (sizeof(unsigned int) + sizeof(float)) * 2048, nullptr, GL_DYNAMIC_READ);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, results2Buffer);
	}

	void talk_to_raycast_shader(cgv::render::context& ctx) {
		raycast_prog.enable(ctx);
		// Set uniform values
		raycast_prog.set_uniform(ctx, "sphere_radius", 0.02);
		raycast_prog.set_uniform(ctx, "ray_direction", vec4(0, 1, 0, 1)); // pls change to vertex buffer containing constructed points
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, vertexBuffer);
		glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(vertices), vertices);

		glDispatchCompute(512 * 512, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

		// retrieve data from buffer
		int* updatedData0 = (int*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(vertices), GL_MAP_READ_BIT);
		std::cout << "\x1b[1;31mtrying to retrieve contents of vertexBuffer\x1b[0m" << std::endl;
		if (updatedData0) {
			for (int i = 0; i < 10; ++i) {
				int value = updatedData0[i];
				std::cout << "vertexBuffer element " << i << ": " << value << std::endl;
			}
		}
		else {
			std::cout << "\x1b[1;31mdid not successfully retrieve elements of results2Buffer\x1b[0m" << std::endl;
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

		glBindBuffer(GL_SHADER_STORAGE_BUFFER, results2Buffer);
		int* updatedData = (int*)glMapNamedBufferRange(results2Buffer, 0, (sizeof(unsigned int) + sizeof(float)) * 2048, GL_MAP_READ_BIT);
		std::cout << "\x1b[1;31mtrying to retrieve contents of results2Buffer\x1b[0m" << std::endl;
		if (updatedData) {
			for (int i = 0; i < 10; ++i) {
				int value = updatedData[i];
				std::cout << "results2Buffer element " << i << ": " << value << std::endl;
			}
		}
		else {
			std::cout << "\x1b[1;31mdid not successfully retrieve elements of results2Buffer\x1b[0m" << std::endl;
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

		glBindBuffer(GL_SHADER_STORAGE_BUFFER, resultBuffer);
		int* updatedData2 = (int*)glMapNamedBufferRange(resultBuffer, 0, sizeof(Vertex) * 2048, GL_MAP_READ_BIT);
		
		std::cout << "\x1b[1;31mtrying to retrieve contents of resultBuffer\x1b[0m" << std::endl;
		if (updatedData2) {
			for (int i = 0; i < 2; ++i) {
				int value = updatedData2[i];
				std::cout << "resultBuffer element " << i << ": " << value << std::endl;
			}
		}
		else {
			std::cout << "\x1b[1;31mdid not successfully retrieve elements of resultBuffer\x1b[0m" << std::endl;
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
		raycast_prog.disable(ctx);
	}

	// return range of depth values
	std::pair<unsigned short, unsigned short> find_extent() {
		unsigned int smallest_non_zero = 127;
		unsigned int largest_finite = 1;

		for (unsigned short depth : depth_frame.frame_data) {
			// Check if depth is not zero and update smallestNonZero
			if (depth > 1 && depth < smallest_non_zero) {
				smallest_non_zero = depth;
			}

			// Check if depth is finite and update largestFinite
			if (depth < 127 && depth > largest_finite) {
				largest_finite = depth;
			}
		}

		return std::make_pair(smallest_non_zero, largest_finite);
	}

	const int TEXTURE_SIZE = 512;
	int getIndex(int x, int y) {
		return y * (TEXTURE_SIZE-1) + x;
	}

	// iterate over texture from left to right, top to bottom and resepctively for right, top and bottom extent
	void find_area_extent() {
		int start = 0;
		int end = TEXTURE_SIZE - 1;

		// north, east, south, west
		float extents[4] = { 0,0,0,0 };
		
		// Check top edge
		unsigned short i = 0, j = 0;
		float kinect_point[3];
		bool found_point = false;
		for (j = 0; j < end; ++j) {
			for (i = 0; i < end; ++i) {
				this->rgbd_inp.map_depth_to_point(i,j, depth_frame.frame_data[getIndex(i,j)], kinect_point);
				if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
					extents[0] = kinect_point[1];
					break;
				}		
			}
			if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
				extents[0] = kinect_point[1];
				break;
			}
		}

		// Check right edge
		for (i = end; i > 0; --i) {
			for (j = 0; j < end; ++j) {
				this->rgbd_inp.map_depth_to_point(i, j, depth_frame.frame_data[getIndex(i, j)], kinect_point);
				if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
					extents[1] = kinect_point[0];
					break;
				}
			}
			if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
				extents[0] = kinect_point[1];
				break;
			}
		}

		// Check bottom edge
		for (j = end; j > 0; --j) {
			for (i = end; i > 0; --i) {
				this->rgbd_inp.map_depth_to_point(i, j, depth_frame.frame_data[getIndex(i, j)], kinect_point);
				if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
					extents[2] = kinect_point[1];
					break;
				}
			}
			if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
				extents[0] = kinect_point[1];
				break;
			}
		}

		// Check left edge
		for (int i = 0; i < end; ++i) {
			for (int j = 0; j < end; ++j) {
				this->rgbd_inp.map_depth_to_point(i, j, depth_frame.frame_data[getIndex(i, j)], kinect_point);
				if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
					extents[3] = kinect_point[0];
					break;
				}
			}
			if (kinect_point[0] || kinect_point[1] || kinect_point[2]) {
				extents[0] = kinect_point[1];
				break;
			}
		}
		
		for (i = 0; i < 4; ++i) {
			std::cout << extents[i] << " ";
		}
		std::cout << std::endl;
		for (int i = 0; i < 4; ++i)
			this->camera_edges[i] = extents[i];
	}

	void get_outer_points() {
		float top = this->camera_edges[0];
		float right = this->camera_edges[1];
		float bottom = this->camera_edges[2];
		float left = this->camera_edges[3];

		this->camera_corners[0] = cgv::vec3(left, top, 1); // top left
		this->camera_corners[1] = cgv::vec3(right, top, 1); // top right
		this->camera_corners[2] = cgv::vec3(left, bottom, 1); // bottom left
		this->camera_corners[3] = cgv::vec3(right, bottom, 1); // bottom right
	}

	int* get_outer_pixel_depth() {
		int corners[4];
		corners[0] = depth_frame.frame_data[0];
		corners[1] = depth_frame.frame_data[511];
		corners[2] = depth_frame.frame_data[511*511];
		corners[3] = depth_frame.frame_data[depth_frame.buffer_size-1];
		std::cout << corners[0] << " " << corners[1] << " " << corners[2] << " " << corners[3] << std::endl;
		return corners;
	}

	void iterate_over_depth_frame() {
		for (int i = 0; i < 512; i++) {
			for (int j = 0; i < 512; j++) {
				unsigned int data = depth_frame.frame_data[i * 512 + j];
				if (data != 0) {
					std::cout << data << " ";
				} 
			}
		}
	}

	void map_depth_to_point(int x,int y) {
		float kinect_point[3];
		this->rgbd_inp.map_depth_to_point(x, y, depth_frame.frame_data[getIndex(x,y)], kinect_point);
		std::cout << "kinect point with x= " << x << " and y= " << y << " ";
		for (float d : kinect_point)
		{
			std::cout << d << " ";
		}
		std::cout << std::endl;
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
				//iterate_over_depth_frame();
				find_extent();
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
	}

	void draw(cgv::render::context& ctx)
	{
		if (one_tap_press != one_tap_flag) {
			//executed_compute_shader = true;
			update_dummy_compute_shader(ctx);
			//talk_to_raycast_shader(ctx);
			if (sP.size() > 0) {
				// slow approach - sP only updated without geometry_less_rendering 
				// 99% of the time there are not 512*512 entries in sP; also size of elements in sP is 6 bytes
				// usvec3 vector-> actual size of the array with : 512*512 * 6 bytes = 1572864 bytes = 1,5 mb
				// Vertex array -> size is: 512*512 * 16 bytes = 4194304 bytes = 4mb
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
			pr.draw(ctx, 0, sP.size()); // only using sP size with geometryless rendering
			pr.disable(ctx);
			if (pr.do_lookup_color())
				color_tex.disable(ctx);
			if (pr.do_geometry_less_rendering())
				depth_tex.disable(ctx); 
			}
		

		if (simple_cube) {
			//draw_cube(ctx);
			/*br.enable(ctx);
			br.disable(ctx);*/
			auto& R = cgv::render::ref_box_renderer(ctx);
			R.set_render_style(brs);
			boxes.clear();
			box_colors.clear();
			boxes.push_back(cgv::dbox3(cgv::vec3(-1, -1, -1), cgv::vec3(1, 1, box_size)));
			box_size += 0.001f;
			box_colors.push_back(cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(1.0f, 0.5f, 1.0f, 0.5f));
			R.set_box_array(ctx, boxes);
			R.set_color_array(ctx, box_colors);
			R.render(ctx, 0, boxes.size());
			
		}
		glEnable(GL_CULL_FACE);
	}
};

#include <cgv/base/register.h>

cgv::base::object_registration<mirror3D> mirror_reg("");
