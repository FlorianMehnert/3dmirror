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
#include <cgv/utils/pointer_test.h>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <holo_disp/shader_display_calibration.h>
#include <cgv/defines/quote.h>
#include <cgv/render/shader_library.h>
#include <cgv/render/performance_monitor.h>
#include <cgv_gl/gl/gl_performance_monitor.h>
#include <fltk/events.h>


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
	public cgv::gui::event_handler,
	public cgv::render::gl::gl_performance_monitor
{
	bool calib_outofdate = true;
protected:
	bool debug_frame_timing = false;
	cgv::render::stereo_view* stereo_view_ptr = nullptr;
	float view_test = 0.0f;
	bool debug_matrices = false;

	// render data
	std::vector<usvec3> sP;
	std::array<usvec3, 512*512> aP;
	std::vector<rgb8> sC;
	
	// color warped to depth image is only needed in case CPU is used to lookup colors
	rgbd::frame_type warped_color_frame;

	// rendering configuration
	cgv::render::point_render_style prs;
	cgv::render::texture depth_tex, color_tex;

	cgv::render::view* view_ptr = 0;

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

	// somthing for the timer event from vr_rgbd
	std::future<size_t> future_handle;

	float distance = 5.0;
	float discard = 0.02f;
	bool depth_lookup = false;
	int bf_size = 10;
	bool fs_show_marched_depth = false;
	bool fs_show_sampled_depth = false;
	bool show_cpu_time = true;
	bool construct_render_data_using_array;

	float step_size = 0.05;
	int step = 100;
	performance_monitor pm = performance_monitor();

	enum ColorMode {
		COLOR_TEX_SM, NORMAL, BRUTE_FORCE, MIRROR, RAYMARCHING
	};
	ColorMode coloring = RAYMARCHING;

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
			//std::cout << "frame: ";
			//auto nw = std::chrono::duration_cast<std::chrono::nanoseconds>(now_h.time_since_epoch()).count();
			//std::cout << ", color=" << (nw - color_frame.system_time_stamp) * 1e-6;
			//std::cout << "(" << color_frame.device_time_stamp << ")";
			//std::cout << "(" << (color_frame.device_time_stamp - col_dev_ts) * 1e-6 << ")";
			//col_dev_ts = color_frame.device_time_stamp;
			//std::cout << ", depth=" << (nw - depth_frame.system_time_stamp) * 1e-6;
			//std::cout << "(" << (depth_frame.device_time_stamp- color_frame.device_time_stamp)*1e-6 << ")";
			//std::cout << "(" << (depth_frame.device_time_stamp - dev_ts) * 1e-6 << ")";
			//dev_ts = depth_frame.device_time_stamp;
			//std::cout << " -> " << (depth_frame.device_time_stamp - color_frame.device_time_stamp) * 1e-6;
			//std::cout << "depth frame ";
			//std::cout << std::endl;

			// color_frame is using 4 bytes per pixel with 2048x1536 in BGR32 format - which wastes one color channel
			// depth_frame is using 2 bytes per pixel DEP16 - probably double precision integer
			//std::cout << "color_frame_size:" << color_frame.buffer_size << "color_frame_resolution:" << color_frame << " depthframesize: " << depth_frame.buffer_size << "depth_frame_resolution:" << depth_frame << std::endl;
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
			coloring = (int)coloring < 4 ? (ColorMode)(((int)coloring) + 1) : COLOR_TEX_SM;
			on_set(&coloring);
			return true;
		case 'X':
			coloring = (int)coloring > 0 ? (ColorMode)(((int)coloring) - 1) : RAYMARCHING;
			on_set(&coloring);
			return true;
		case 'A':
			construct_render_data_using_array = !construct_render_data_using_array;
			on_set(&construct_render_data_using_array);
			return true;
		case cgv::gui::KEY_Num_8: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			if (ke.get_modifiers() == cgv::gui::EventModifier::EM_ALT) {
				step++;
				std::cout << "increase amount of iterations" << step << std::endl;
				on_set(&step);
				return true;
			}
			else {
				step_size += 0.001;
				std::cout << "decrease step length" << step_size << std::endl;
				on_set(&step_size);
				return true;
			}

		}
		case cgv::gui::KEY_Num_2: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			if (ke.get_modifiers() == cgv::gui::EventModifier::EM_ALT) {
				step--;
				std::cout << "decrease amount of iterations" << step << std::endl;
				on_set(&step);
				return true;
			}
			else {
				step_size -= 0.001;
				std::cout << "decrease ray lenght" << step_size << std::endl;
				on_set(&step_size);
				return true;
			}

		}
		case '0':
			shader_calib.eye_separation_factor = 0;
			on_set(&shader_calib.eye_separation_factor);
			return true;
		case cgv::gui::KEY_Left: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			if (ke.get_modifiers() == cgv::gui::EventModifier::EM_ALT) {
				discard -= 0.001;
				std::cout << "z tolerance for triangles: " << discard << std::endl;
				on_set(&discard);
				return true;
			}
			else {
				shader_calib.eye_separation_factor -= 0.0625f;
				std::cout << "eye_separation_factor: " << shader_calib.eye_separation_factor << std::endl;
				on_set(&shader_calib.eye_separation_factor);
				return true;
			}
			
		}
		case cgv::gui::KEY_Right: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			if (ke.get_modifiers() == cgv::gui::EventModifier::EM_ALT) {
				discard += 0.001;
				std::cout << "z tolerance for triangles: " << discard << std::endl;
				on_set(&discard);
				return true;
			}
			else {
				shader_calib.eye_separation_factor += 0.0625f;
				std::cout << "eye_separation_factor: " << shader_calib.eye_separation_factor << std::endl;
				on_set(&shader_calib.eye_separation_factor);
				return true;
			}
		}
		case cgv::gui::KEY_Up: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			distance += 0.1f;
			std::cout << "distance culling: " << distance << std::endl;
			on_set(&distance);
			return true;
		}
		case cgv::gui::KEY_Down: if (ka != cgv::gui::KeyAction::KA_RELEASE) {
			distance -= 0.1f;
			std::cout << "distance culling: " << distance << std::endl;
			on_set(&distance);
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
		add_member_control(this, "one time execution", one_tap_press, "toggle");
		add_member_control(this, "cull mode", coloring, "dropdown", "enums='color, normals, brute-force, mirror, raymarching'");
		
		// raymarching - inherited
		add_member_control(this, "Eye Separation Factor", shader_calib.eye_separation_factor, "value_slider", "min=0;max=20;ticks=true");
		add_member_control(this, "Debug Matrices", debug_matrices, "check");
		add_member_control(this, "Interpolate View Matrix", shader_calib.interpolate_view_matrix, "check");
		
		// raymarching - my params
		provider::align("\a");
		add_member_control(this, "bf_size", bf_size, "value_slider", "min=0;max=100;step=1");
		add_member_control(this, "ray step length[m]", step_size, "value_slider", "min=0;max=0.1;step=0.001");
		add_member_control(this, "max iter raymarching", step, "value_slider", "min=0;max=500;step=1");
		add_member_control(this, "debug: max ray depth", fs_show_marched_depth, "check");
		add_member_control(this, "debug: final sampled depth", fs_show_sampled_depth, "check");
		add_member_control(this, "use array for geometry", construct_render_data_using_array, "check");
		
		provider::align("\b");

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
	}

	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return rh.reflect_member("eye_separation_factor", shader_calib.eye_separation_factor);
		//return rh.reflect_member("eye_separation_factor", shader_calib.eye_separation_factor);
	}

	bool init(cgv::render::context& ctx)
	{
		start_first_device();
		ctx.set_bg_clr_idx(4);

		// render points
		cgv::render::ref_point_renderer(ctx, 1);
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

	void construct_my_render_data(
		const rgbd::frame_type& depth_frame,
		std::array<cgv::usvec3, 512*512>& sP,
		uint16_t sub_sample, uint16_t sub_line_sample)
	{
		for (uint16_t y = 0; y < depth_frame.height; y += sub_sample)
			for (uint16_t x = 0; x < depth_frame.width; x += sub_sample) {
				if (((x % sub_line_sample) != 0) && ((y % sub_line_sample) != 0))
					continue;
				uint16_t depth = reinterpret_cast<const uint16_t&>(depth_frame.frame_data[(y * depth_frame.width + x) * depth_frame.get_nr_bytes_per_pixel()]);
				if (depth == 0)
					continue;
				sP[x + depth_frame.width * y] = cgv::usvec3(x, y, depth);
			}
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
					if (construct_render_data_using_array) 
						construct_my_render_data(depth_frame, aP, 1, 1);
					else 
						rgbd::construct_rgbd_render_data(depth_frame, sP);
			}
			if (!stereo_view_ptr)
				stereo_view_ptr = dynamic_cast<cgv::render::stereo_view*>(find_view_as_node());
			
	}
	void draw(cgv::render::context& ctx)
	{
		if (!stereo_view_ptr)
			return;
		if (one_tap_press != one_tap_flag) {
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
			pr.ref_prog().set_uniform(ctx, "coloring", (int)coloring);
			shader_calib.set_uniforms(ctx, pr.ref_prog(), *stereo_view_ptr);
			pr.ref_prog().set_uniform(ctx, "eye_separation", shader_calib.eye_separation_factor/1000);
			pr.ref_prog().set_uniform(ctx, "bf_size", bf_size);
			pr.ref_prog().set_uniform(ctx, "raymarch_limit", step);
			pr.ref_prog().set_uniform(ctx, "ray_length_m", step_size);
			pr.ref_prog().set_uniform(ctx, "show_marched_depth", fs_show_marched_depth);
			pr.ref_prog().set_uniform(ctx, "show_sampled_depth", fs_show_sampled_depth);
			pr.draw(ctx, 0, sP.size()); // only using sP size with geometryless rendering
			pr.disable(ctx);
			if (pr.do_lookup_color())
				color_tex.disable(ctx);
			if (pr.do_geometry_less_rendering())
				depth_tex.disable(ctx); 
		}
		glEnable(GL_CULL_FACE);
	}
	
};

#include <cgv/base/register.h>

cgv::base::object_registration<mirror3D> mirror_reg("");
