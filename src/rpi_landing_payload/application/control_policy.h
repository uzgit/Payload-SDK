#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

enum Mode
{
	MODE_START = 0,
	MODE_STATIC_SEARCH,
	MODE_SEARCH_DOWN,
	MODE_SEARCH_UP,
	MODE_AIM_CAMERA,
	MODE_APPROACH,
	MODE_YAW_ALIGNMENT,
	MODE_HORIZONTAL_ALIGNMENT,
	MODE_ZOOM_OUT,
	MODE_DESCENT,
	MODE_DESCENT_ZOOM_OUT,
	MODE_ASCENT,
	MODE_COMMIT,
	MODE_LANDED,
	MODE_MAX
};

enum ZoomPolicy
{
	ZOOM_NONE = 0,
	ZOOM_AUTO,
	ZOOM_IN,
	ZOOM_OUT,
	ZOOM_MAX
};

enum GimbalPolicy
{
	GIMBAL_NO_CHANGE = 0,
	GIMBAL_FORWARD,
	GIMBAL_ACTIVE,
	GIMBAL_DOWN,
	GIMBAL_MAX
};

const char* gimbal_policy_names[] = {
	"GIMBAL_NO_CHANGE",
	"GIMBAL_FORWARD",
	"GIMBAL_ACTIVE",
	"GIMBAL_DOWN",
	"GIMBAL_MAX"
};

const char* zoom_policy_names[] = {
	"ZOOM_NONE",
	"ZOOM_AUTO",
	"ZOOM_IN",
	"ZOOM_OUT",
	"ZOOM_MAX"
};

const char* mode_names[] = {
	"MODE_START",
	"MODE_STATIC_SEARCH",
	"MODE_SEARCH_DOWN",
	"MODE_SEARCH_UP",
	"MODE_AIM_CAMERA",
	"MODE_APPROACH",
	"MODE_YAW_ALIGNMENT",
	"MODE_HORIZONTAL_ALIGNMENT",
	"MODE_ZOOM_OUT",
	"MODE_DESCENT",
	"MODE_DESCENT_ZOOM_OUT",
	"MODE_ASCENT",
	"MODE_COMMIT",
	"MODE_LANDED",
	"MODE_MAX"
};

const char* short_mode_names[] = {
	"Start",
	"Static Search",
	"Search Down",
	"Search Up",
	"Aim Camera",
	"Approach",
	"Yaw Align",
	"Horiz. Align",
	"Zoom Out",
	"Descent",
	"Desc Zoom Out",
	"Asecnt",
	"Commit",
	"Landed",
	"SHORTENED_MODE_MAX"
};

const char* parameter_names[] = {
        "forward_scalar",
	"right_scalar",
	"up_scalar",
	"yaw_rate_cw",
	"gimbal_tilt_up",
        "gimbal_pan_right",
	"min_time",
	"max_time",
	"zoom"
};

struct ModeParameters
{
	double forward_scalar;
	double right_scalar;
	double up_scalar;
	double yaw_rate_cw_scalar;
	GimbalPolicy gimbal_policy;
	double gimbal_tilt_up_scalar;
	double gimbal_pan_right_scalar;
	double min_time;
	double max_time;
	ZoomPolicy zoom_policy;

	ModeParameters(){}

	ModeParameters(double forward, double right, double up, double yawRateCW, GimbalPolicy gimbal, double tiltUp, double panRight, double minTime, double maxTime, ZoomPolicy zoomSetting)
	: forward_scalar(forward),
	  right_scalar(right),
	  up_scalar(up),
	  yaw_rate_cw_scalar(yawRateCW),
	  gimbal_policy(gimbal),
	  gimbal_tilt_up_scalar(tiltUp),
	  gimbal_pan_right_scalar(panRight),
	  min_time(minTime),
	  max_time(maxTime),
	  zoom_policy(zoomSetting)
	{
		std::cout << "ModeParameters constructor called." << std::endl;
	}	
};

string parameter_headers()
{
	int mode_name_width = 13;
	int column_width = 17;

	ostringstream output;

	for(int i = 0; i < mode_name_width; i ++)
	{
		output << " ";
	}
	for(int i = 0; i < sizeof(parameter_names) / sizeof(parameter_names[0]); i ++)
	{
		output << setw(column_width) << right << parameter_names[i];
	}

	return output.str();
}

std::ostream& operator<<(std::ostream& os, const ModeParameters& params)
{
	int mode_name_width = 13;
	int columnWidth = 17;
	
    os << std::setw(columnWidth) << params.forward_scalar
       << std::setw(columnWidth) << params.right_scalar
       << std::setw(columnWidth) << params.up_scalar
       << std::setw(columnWidth) << params.yaw_rate_cw_scalar
       << std::setw(columnWidth) << params.gimbal_tilt_up_scalar
       << std::setw(columnWidth) << params.gimbal_pan_right_scalar
       << std::setw(columnWidth) << params.min_time
       << std::setw(columnWidth) << params.max_time
       << std::setw(columnWidth) << params.zoom_policy;

    return os;
}

class ControlPolicy
{
private:
	uint64_t start_time; // microseconds since epoch

	double landing_pad_detection_timeout_s = 1;
	uint64_t last_landing_pad_detection_time;
	uint64_t last_gimbal_orientation_time;
	uint64_t state_change_time;

	double theta_u;
	double theta_v;
	double gimbal_tilt;
	double zoom_factor;

	Mode current_mode;
	Mode previous_mode;

	ZoomPolicy current_zoom_policy = ZOOM_NONE;
	GimbalPolicy current_gimbal_policy = GIMBAL_NO_CHANGE;

	bool change_mode( Mode new_mode );
	double get_current_time_s();
	uint64_t get_current_time();
	double get_duration_s(uint64_t time1, uint64_t time2);
	double time_since_landing_pad_detection_s();
	double time_in_state_s();

	double constrain(double input, double min, double max);

	// event triggers
	bool state_min_time_reached();
	bool state_max_time_reached();
	bool landing_pad_detection();	// not edge, but state of being visible

	ModeParameters mode_parameters[MODE_MAX];

public:
	ControlPolicy();
	void update();
	void update_landing_pad_detection(uint64_t detection_time, double new_theta_u, double new_theta_v);
	void update_gimbal_orientation(uint64_t time, double tilt);
	void update_zoom_factor(double zoom);
	void get_gimbal_control_effort(double& tilt, double& pan);
	void get_flight_control_effort(double& forward, double& right, double& up, double& yaw_rate_cw);
	
	Mode get_mode();
	ZoomPolicy get_zoom_policy();
	GimbalPolicy get_gimbal_policy();
};

bool ControlPolicy::change_mode( Mode new_mode )
{
	bool result = false;

	if( new_mode != current_mode )
	{
		cout << "mode switch: " << mode_names[current_mode] << " -> " << mode_names[new_mode] << endl;
		
		previous_mode = current_mode;
		current_mode = new_mode;

		current_zoom_policy = mode_parameters[new_mode].zoom_policy;
		current_gimbal_policy = mode_parameters[new_mode].gimbal_policy;

		state_change_time = get_current_time();

		result = true;
	}

	return result;
}

uint64_t ControlPolicy::get_current_time()
{
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start_time;
}

double ControlPolicy::get_current_time_s()
{
	return (get_current_time() - start_time)/1e6;
}

double ControlPolicy::get_duration_s(uint64_t time1, uint64_t time2)
{
	return (time1 - time2) / 1e6;
}

double ControlPolicy::time_in_state_s()
{
	return get_duration_s( get_current_time(), state_change_time );
}

double ControlPolicy::time_since_landing_pad_detection_s()
{
	return get_duration_s( get_current_time(), last_landing_pad_detection_time );
}

ControlPolicy::ControlPolicy()
{
	previous_mode = MODE_START;
	current_mode  = MODE_START;

	double yaw_rate_search = 20;	// 20 deg/s CW
	double tilt_rate_search = 15;
	double yaw_scalar_aim = 5;
	double tilt_scalar_aim =  5;
	double pan_scalar_aim  =  5;
	double approach_forward_scalar = 10;
	double approach_right_scalar = 10;
	double approach_up_scalar = 10;
	double gimbal_down_value = -999;
	double gimbal_up_value = 999;
	double yaw_align_scalar = 5;
	double horizontal_align_scalar = 5;
	double descent_velocity = -1;
	double ascent_velocity = 1;

	mode_parameters[MODE_START] 			= {0.0,	0.0, 0.0, 0.0, GIMBAL_NO_CHANGE, 0.0,	0.0, 0.0, 0.0, ZOOM_NONE};
	mode_parameters[MODE_STATIC_SEARCH]		= {0.0,	0.0, 0.0, 0.0, GIMBAL_NO_CHANGE, 0.0,	0.0, 0.0, 2.0, ZOOM_NONE};
	mode_parameters[MODE_SEARCH_DOWN] 		= {0.0,	0.0, 0.0, yaw_rate_search, GIMBAL_ACTIVE, -tilt_rate_search, -90.0, 0.0, 7.0, ZOOM_NONE};
	mode_parameters[MODE_SEARCH_UP] 		= {0.0,	0.0, 0.0, yaw_rate_search, GIMBAL_ACTIVE, tilt_rate_search,  90.0, 0.0, 7.5, ZOOM_NONE};
	mode_parameters[MODE_AIM_CAMERA] 		= {0.0,	0.0, 0.0, yaw_scalar_aim,  GIMBAL_ACTIVE, tilt_scalar_aim, pan_scalar_aim, 0.0, -1.0, ZOOM_AUTO};
	mode_parameters[MODE_APPROACH]			= {approach_forward_scalar, approach_right_scalar, 0.0, 0.0, GIMBAL_ACTIVE, tilt_scalar_aim, pan_scalar_aim, 0.0, -1.0, ZOOM_AUTO};
	mode_parameters[MODE_YAW_ALIGNMENT]		= {0.0,	0.0, 0.0, yaw_align_scalar, GIMBAL_DOWN, 0.0, 0.0, 0.0, -1.0, ZOOM_AUTO};
	mode_parameters[MODE_HORIZONTAL_ALIGNMENT]	= {horizontal_align_scalar, horizontal_align_scalar, 0.0, 0.0, GIMBAL_DOWN, 0.0, 0.0, 0.0, -1.0, ZOOM_NONE};
	mode_parameters[MODE_ZOOM_OUT]			= {0.0,	0.0, 0.0, 0.0, GIMBAL_NO_CHANGE, 0.0, 0.0, 0.0, 5.0, ZOOM_OUT};
	mode_parameters[MODE_DESCENT]			= {0.0,	0.0, descent_velocity, 0.0, GIMBAL_DOWN, 0.0, 0.0, 0.0, 5.0, ZOOM_AUTO};
	mode_parameters[MODE_DESCENT_ZOOM_OUT]		= {0.0,	0.0, 0.0, 0.0, GIMBAL_DOWN, 0.0, 0.0, 0.0, 5.0, ZOOM_OUT};
	mode_parameters[MODE_ASCENT]			= {0.0,	0.0, ascent_velocity, 0.0, GIMBAL_DOWN, 0.0, 0.0, 0.0, 5.0, ZOOM_NONE};
	mode_parameters[MODE_COMMIT]			= {0.0,	0.0, 0.0, 0.0, GIMBAL_FORWARD, 0.0, 0.0, 0.0, -1.0, ZOOM_NONE};
	mode_parameters[MODE_LANDED]			= {0.0,	0.0, 0.0, 0.0, GIMBAL_NO_CHANGE, 0.0, 0.0, 0.0, -1.0, ZOOM_NONE};

	cout << "CONTROL POLICY SUMMARY" << endl
	     << parameter_headers()
	     << endl;
	for(int i = 0; i < MODE_LANDED; i ++)
	{
		cout << setw(13)
		     << short_mode_names[i]
		     << mode_parameters[i]
		     << endl;
	}

	start_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
	
double ControlPolicy::constrain(double input, double min, double max)
{
	double result = input;
	if( input < min )
	{
		input = min;
	}
	else if( input > max )
	{
		input = max;
	}
	return result;
}

bool ControlPolicy::state_min_time_reached()
{
	bool result = false;

	double min_time = mode_parameters[current_mode].min_time;
	if( min_time != -1 && time_in_state_s() > min_time )
	{
		result = true;
	}

	return result;
}

bool ControlPolicy::state_max_time_reached()
{
	bool result = false;

	double max_time = mode_parameters[current_mode].max_time;
	if( max_time != -1 && time_in_state_s() > max_time )
	{
		cout << "here " << time_in_state_s() << " > " << max_time << endl;
		result = true;
	}

	return result;
}

bool ControlPolicy::landing_pad_detection()
{
	return time_since_landing_pad_detection_s() < landing_pad_detection_timeout_s; 
}

void ControlPolicy::update()
{
	// initialize
	if( current_mode == MODE_START )
	{
		change_mode( MODE_STATIC_SEARCH );
	}

	// handle mode switches
	if( state_min_time_reached() )
	{
		// default/timeout transitions
		if( state_max_time_reached() )
		{
			switch( current_mode )
			{
				case MODE_STATIC_SEARCH:
					change_mode( MODE_SEARCH_DOWN );
					break;
				
				case MODE_SEARCH_DOWN:
					change_mode( MODE_SEARCH_UP );
					break;
				
				case MODE_SEARCH_UP:
					change_mode( MODE_SEARCH_DOWN );
					break;

				case MODE_AIM_CAMERA:
					break;

				case MODE_ZOOM_OUT:
					change_mode( MODE_STATIC_SEARCH );
					break;

				default:
					cout << "no default rule for mode " << current_mode << endl;
					break;
			}
		}
		// positive edge on landing pad detection
		else if( landing_pad_detection() )
		{
			switch( current_mode )
			{
				case MODE_STATIC_SEARCH:
				case MODE_SEARCH_DOWN:
				case MODE_SEARCH_UP:
					change_mode( MODE_AIM_CAMERA );
					break;

				case MODE_AIM_CAMERA:
					break;

				case MODE_ZOOM_OUT:
					change_mode( previous_mode );
					break;
				default:
					cout << "no detection rule for mode " << current_mode << endl;
					break;
			}
		}
		// negative edge on landing pad detection
		else if( ! landing_pad_detection() )
		{
			switch( current_mode )
			{
				case MODE_STATIC_SEARCH:
				case MODE_SEARCH_DOWN:
				case MODE_SEARCH_UP:
					break;

				case MODE_AIM_CAMERA:
					change_mode( MODE_ZOOM_OUT );
					break;
				default:
					cout << "no loss rule for mode " << current_mode << endl;
					break;
			}
		}
	}
}

void ControlPolicy::update_zoom_factor( double zoom )
{
	zoom_factor = zoom;
}

void ControlPolicy::update_landing_pad_detection(uint64_t detection_time, double new_theta_u, double new_theta_v)
{
	theta_u = new_theta_u;
	theta_v = new_theta_v;
	last_landing_pad_detection_time = detection_time - start_time;
}

void ControlPolicy::update_gimbal_orientation(uint64_t time, double tilt)
{
	last_gimbal_orientation_time = time;
	gimbal_tilt = tilt;
}

void ControlPolicy::get_gimbal_control_effort(double& tilt, double& pan)
{
	switch( current_mode )
	{
		case MODE_STATIC_SEARCH:
			pan  = 0;
			tilt = 0;
			break;
		case MODE_SEARCH_DOWN:
			pan  = 0;
			tilt = mode_parameters[current_mode].gimbal_tilt_up_scalar * zoom_factor;
			break;
		case MODE_SEARCH_UP:
			pan  = 0;
			tilt = mode_parameters[current_mode].gimbal_tilt_up_scalar * zoom_factor;
			break;
		case MODE_AIM_CAMERA:
			pan  =  mode_parameters[current_mode].gimbal_pan_right_scalar * theta_u * zoom_factor / 2.0;
			tilt = -mode_parameters[current_mode].gimbal_tilt_up_scalar   * theta_v * zoom_factor / 2.0;
			break;
		case MODE_ZOOM_OUT:
			pan  = 0;
			tilt = 0;
			break;
	}
}

void ControlPolicy::get_flight_control_effort(double& forward, double& right, double& up, double& yaw_rate_cw)
{
	forward = 0;
	right = 0;
	up = 0;
	yaw_rate_cw = 0;
}

Mode ControlPolicy::get_mode()
{
	return current_mode;
}

ZoomPolicy ControlPolicy::get_zoom_policy()
{
	return current_zoom_policy;
}

GimbalPolicy ControlPolicy::get_gimbal_policy()
{
	return current_gimbal_policy;
}
