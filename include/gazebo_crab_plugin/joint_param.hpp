// C++ headers
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>

// BOOST headers
#include <boost/algorithm/string.hpp>


/** @brief structure to store the parameters (including the computed error) of the joint controller. the structure provides an
 *         'error' function which returns a weighted error based on the values of vel_sq_mean_error and pos_sq_mean_error
 */
typedef struct {
	double p,i,d;				// pid gains
	double i_clamp;				// pid integral minimum/maximum
	double multiplier;			// overall multiplier (for convenience)
	double max_vel;				// maximum target velocity (before damping)
	double damping;				// velocity damping
	double vel_sq_mean_error;	// mean square error of the joint velocity
	double pos_sq_mean_error;	// mean square error of the joint position
	int input_type;				// 0=position, 1=velocity
	int update_type;			// 0=force, 1=delta-force
	
	/// @brief sets the values of the structure based on the values defined in the input string 'str'. the input format is the same as used for the log files (name=value strings, separated by spaces).
	void apply( const std::string &str ) {
		std::vector<std::string> tokens;
		std::vector<std::string> pairs;
		
		multiplier = 1.0;		// this one is usually not specified in the input string. 1.0 is the default value
		
		// split up the string
		boost::split( tokens, str, boost::is_any_of(" ") );
		for( int n=0; n<tokens.size(); n++ ) {
			// ignore empty strings
			if( tokens[n].size() < 1 )
				continue;
			
			boost::split( pairs, tokens[n], boost::is_any_of("=") );
			if( pairs.size() != 2 )
				continue;
			
			if( pairs[0] == "p" ) {
				p = atof( pairs[1].c_str() );
			} else if( pairs[0] == "i" ) {
				i = atof( pairs[1].c_str() );
			} else if( pairs[0] == "d" ) {
				d = atof( pairs[1].c_str() );
			} else if( pairs[0] == "i_clamp" ) {
				i_clamp = atof( pairs[1].c_str() );
			} else if( pairs[0] == "multiplier" ) {
				multiplier = atof( pairs[1].c_str() );
			} else if( pairs[0] == "v_max" ) {
				max_vel = atof( pairs[1].c_str() );
			} else if( pairs[0] == "damp" ) {
				damping = atof( pairs[1].c_str() );
			} else if( pairs[0] == "vel_error" ) {
				vel_sq_mean_error = atof( pairs[1].c_str() );
			} else if( pairs[0] == "pos_error" ) {
				pos_sq_mean_error = atof( pairs[1].c_str() );
			}
		}
	}
	
	/// @brief returns the weighted error
	double error() {
		// weighting the position/angle error heigher to compensate for the different scaling
		return vel_sq_mean_error + 10000*pos_sq_mean_error;
	}
} j_param_t;


/// @brief function for printing the j_param_t struct
std::ostream &dump( std::ostream &o, const j_param_t &param ) {
	return o << "  vel_error=" << param.vel_sq_mean_error
		<< " pos_error=" << param.pos_sq_mean_error
		<< " p=" << param.p
		<< " i=" << param.i
		<< " d=" << param.d
		<< " i_clamp=" << param.i_clamp
		<< " v_max=" << param.max_vel
		<< " damp=" << param.damping
		<< " input_type=" << param.input_type
		<< " update_type=" << param.update_type;
}


/// @brief operator for printing the j_param_t struct
std::ostream& operator << (std::ostream &o,const j_param_t &p){
  return dump(o,p);
}












