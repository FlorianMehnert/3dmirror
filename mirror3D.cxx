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

//#include <cgv_gl/point_renderer.h>
//#include <cgv_gl/gl/gl.h>

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

	// point cloud data
	std::vector<vec3> P;
	std::vector<rgb8> C;

	// color and depth frames from kinect
	rgbd::frame_type depth_frame;
	rgbd::frame_type color_frame;

	// color warped to depth image is only needed in case CPU is used to lookup colors
	rgbd::frame_type warped_color_frame;
	
	// rendering configuration
	rgbd::rgbd_point_renderer pr;
	cgv::render::point_render_style prs;
	cgv::render::texture depth_tex, color_tex;

	cgv::render::view* view_ptr = 0;
public:
	mirror3D() : color_tex("uint8[R,G,B]"), depth_tex("uint16[R]")
	{
		set_name("mirror3D");
		pr.set_distortion_map_usage(true);
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
	bool init(cgv::render::context& ctx)
	{
		start_first_device();
		ctx.set_bg_clr_idx(4);
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
			depth_frame_changed = false;
		}
		if (!pr.do_geometry_less_rendering()) {
			if (!pr.do_lookup_color()) {
				rgbd_inp.map_color_to_depth(depth_frame, color_frame, warped_color_frame);
				rgbd::construct_rgbd_render_data_with_color(depth_frame, warped_color_frame, sP, sC);
			}
			else
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
			if (pr.do_geometry_less_rendering())
				pr.ref_prog().set_uniform(ctx, "depth_image", 0);
			pr.ref_prog().set_uniform(ctx, "color_image", 1);
			pr.draw(ctx, 0, sP.size());
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
