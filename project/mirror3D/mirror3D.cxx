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
#include <cgv/utils/pointer_test.h>
#include <chrono>

// specifically for mirror3D plugin
#include <mirror3D.h>
#include <frame.h>

// constructing pointclouds
#include "point_cloud/point_cloud.h"

// timer event for camera
#include <future>

using namespace cgv::render;

//#include <cgv_gl/point_renderer.h>
#include <cgv_gl/surfel_renderer.h>

//#include <cgv_gl/gl/gl.h>

// from vr_rgbd
struct vertex : public cgv::render::render_types
{
	vec3 point;
	rgba8 color;
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

	// using surfel renderer to use instanced rendering
	cgv::render::surfel_render_style source_srs;

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

	// default point renderer
	//rgbd::rgbd_point_renderer pr;
	rgbd_mesh_renderer pr;

	// somthing for the timer event from vr_rgbd
	std::future<size_t> future_handle;

	// buffer to hold all vertices for the compute shader
	GLuint input_buffer = 0;
	GLuint points;
	vec3 vertex_array[262144]; // 512 x 512
	uvec3 vres;

	// index buffer
	PointIndexBuffer PIB[262144];

	// element buffer object
	GLuint ebo;

	// shader shared buffer object creation
	GLuint buffer_id = 0;

	// toggle via gui - construct pcl or use surfel renderer
	bool surfel = false;
	bool simple_cube = false;
	bool shader_demo = true;
	bool construct_quads = true;
	float distance = 5.0;
	float discard = 0.02;
	bool render_quads = true;
	bool depth_lookup = false;
	bool flip_y = true;


public:
	mirror3D() : color_tex("uint8[R,G,B]"), depth_tex("uint16[R]")
	{	
		set_name("mirror3D");
		pr.set_distortion_map_usage(true);
		pr.set_geometry_less_rendering(true);
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
		add_member_control(this, "flip y", flip_y, "toggle", "w=66", " ");
		add_member_control(this, "render quads", render_quads, "check");
		
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

		// surfel_render
		if (begin_tree_node("Surfel Rendering", source_srs)) {
			align("\a");
			//sr.create_gui(this, *this);
			add_gui("surfel_style", source_srs);
			align("\b");
			end_tree_node(source_srs);
		}
		
	}
	bool init(cgv::render::context& ctx)
	{
		start_first_device();
		ctx.set_bg_clr_idx(4);
		
		// render points
		cgv::render::ref_point_renderer(ctx, 1);
		
		// something with compute shaders
		// cgv::render::ref_clod_point_renderer(ctx, 1);
		
		// add own shader code -> build glpr in the current folder

		// https://www.khronos.org/opengl/wiki/Compute_Shader
		//setup_compute_shader(ctx);
		
		// create buffer object passed to compute shader later (depth frame is 512x512)
		//int size = 512*512*3*sizeof(GLuint);//sizeof(float);// +sizeof(GLuint);
		//std::cout << size << std::endl;
		//create_storage_buffer(1024);

		return pr.init(ctx);
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, -1);

		pr.clear(ctx);
		if (is_running) {
			is_running = false;
			on_set_base(&is_running, *this);
		}
		//glDeleteBuffers(1, &buffer_id);
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

	void create_storage_buffer(size_t size) {
		// TODO: delete Buffer
		glGenBuffers(1, &buffer_id);
		glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
		glBufferData(GL_ARRAY_BUFFER, size, &vertex_array, GL_DYNAMIC_DRAW);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, buffer_id);
	}

	void frame_to_buffer() {
		std::memset(vertex_array, 0, sizeof(vertex_array)); // set all to 0
		if (!depth_frame.frame_data.empty()) {
			const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame.frame_data.front());
			int i = 0;

			for (int y = 0; y < depth_frame.height; ++y) {
				for (int x = 0; x < depth_frame.width; ++x) {
					vec3 p;
					if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0])) {
						vertex_array[i] = depths[i];
					}
					++i;
				}
			}
			// depth frame size is 512 x 512 x 2
			std::cout << "depth_frame height: " << depth_frame.height << std::endl;
		}
		else {
			std::cout << "no frame data" << std::endl;
		}
	}

	void talk_to_compute_shader(cgv::render::context& ctx) {

		// dummy data see gradient_viewer.cxx
		unsigned int w = 192;
		unsigned int h = 128;
		prog.enable(ctx);
		shader_program* cmpt_ptr = &prog;

		cmpt_ptr->set_uniform(ctx, "width", w);
		cmpt_ptr->set_uniform(ctx, "height", h);
		// frame_to_buffer();
		glBindBufferBase(GL_ARRAY_BUFFER, 1, buffer_id); // www.khronos.org/opengl/wiki/Buffer_Object
		glDispatchCompute(16, 16, 1);
		
		// wait for the compute shader to finish
		//glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		
		prog.disable(ctx);
		//std::cout << compute_buffer << std::endl;
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
			// lookup and geometryless are checked
			// sP and sC are populated with point data
			if (!pr.do_geometry_less_rendering()) {
				if (!pr.do_lookup_color()) {
					rgbd_inp.map_color_to_depth(depth_frame, color_frame, warped_color_frame);
					rgbd::construct_rgbd_render_data_with_color(depth_frame, warped_color_frame, sP, sC);
					// flip y if needed
					
				}
				else
					// TODO: use own function to avoid pushback : alternatively use own construct function (without pushback)
					rgbd::construct_rgbd_render_data(depth_frame, sP);
			}
	}

	void draw(cgv::render::context& ctx)
	{
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
			if (pr.do_geometry_less_rendering());
			pr.ref_prog().set_uniform(ctx, "depth_image", 0);
			pr.ref_prog().set_uniform(ctx, "color_image", 1);
			pr.ref_prog().set_uniform(ctx, "max_distance", distance);
			pr.ref_prog().set_uniform(ctx, "discard_dst", discard);
			pr.ref_prog().set_uniform(ctx, "construct_quads", construct_quads);
			pr.ref_prog().set_uniform(ctx, "render_quads", render_quads);
			pr.draw(ctx, 0, sP.size());
			
			pr.disable(ctx);
			if (pr.do_lookup_color())
				color_tex.disable(ctx);
			if (pr.do_geometry_less_rendering())
				depth_tex.disable(ctx);
			}
		
		/*if (simple_cube) {
			ctx.ref_surface_shader_program().enable(ctx);
			ctx.set_material(material);
			ctx.push_modelview_matrix();
			ctx.set_color(rgb(0, 1, 0.2f));
			ctx.tesselate_unit_cube();
			ctx.push_modelview_matrix();
			ctx.pop_modelview_matrix();
			ctx.ref_surface_shader_program().disable(ctx);
		}*/
		glEnable(GL_CULL_FACE);
	}
};

#include <cgv/base/register.h>

cgv::base::object_registration<mirror3D> mirror_reg("");
