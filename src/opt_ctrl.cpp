// ROS headers
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <gazebo_crab_plugin/dyn_paramsConfig.h>		// auto-generated, based on ../cfg/dyn_params.cfg

// project headers
#include "../include/gazebo_crab_plugin/joint_param.hpp"

// header, as sugested in http://wiki.gazebosim.org/wiki/Tutorials/1.9/Creating_ROS_plugins_for_Gazebo
//#include <gazebo/common/Plugin.hh>

// GAZEBO headers
// headers, as sugested in http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <control_toolbox/pid.h>

#include <gazebo_crab_plugin/pid_joint_state.h>		// auto-generated by the project, based on msg/pid_joint_state.msg
#include <gazebo_crab_plugin/pid_joint_param.h>		// auto-generated by the project, based on msg/pid_joint_param.msg
#include <gazebo_crab_plugin/pid_joint_error.h>		// auto-generated by the project, based on msg/pid_joint_param.msg

// C++ headers
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <numeric>
#include <limits>
#include <ctime>		// for filename creation

// BOOST headers
#include <boost/bind.hpp>
#include <boost/math/distributions/beta.hpp>
#include <boost/algorithm/string.hpp>





namespace opt_ctrl {


/// @brief sort function for parameter sets. sorts by the velocity+position error
bool p_sort_fun( j_param_t left, j_param_t right ) {
	/*
	double error_left = left.vel_sq_mean_error + left.pos_sq_mean_error;
	double error_right = right.vel_sq_mean_error + right.pos_sq_mean_error;
	return error_left < error_right;
	*/
	return left.error() < right.error();
}



/// @brief sort function for parameter sets. sorts by the velocity+position error. currently assumes that only entry 2 and 3 are occupied
bool vec_p_sort_fun( std::vector< j_param_t > left, std::vector< j_param_t > right ) {
	return left[2].error()+left[3].error() < right[2].error()+right[3].error();
}



/// @brief copies the current time in a himan readable format into the provided string. time format: YYYY-MM-DD_hh:mm:ss
void nice_time_string( std::string &result ) {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%Y-%m-%d_%H:%M:%S",timeinfo);
	
	result = buffer;
}



/** @brief class to optimize the parameters for the joint controller. subscribes to the joints error state topic and uses the
 *         joints parameter topic to change the parameters.
 */
class OptCtrl {
	
	
	typedef gazebo_crab_plugin::pid_joint_error joint_err;
	
	typedef std::vector< ros::Subscriber > vec_sub;		// subscriber (vector over joints)
	typedef std::vector< vec_sub > vec_sub_2d;			// subscriber (vector over arms)
	typedef std::vector< vec_sub_2d > vec_sub_3d;		// subscriber (vector over bots)
	
	typedef std::vector< ros::Publisher > vec_pub;		// subscriber (vector over joints)
	typedef std::vector< vec_pub > vec_pub_2d;			// subscriber (vector over arms)
	typedef std::vector< vec_pub_2d > vec_pub_3d;		// subscriber (vector over bots)
	
	typedef std::vector< ros::Time > vec_time;
	typedef std::vector< vec_time > vec_time_2d;
	typedef std::vector< vec_time_2d > vec_time_3d;
	
	typedef std::vector< double > vec_err;
	typedef std::vector< vec_err > vec_err_2d;
	typedef std::vector< vec_err_2d > vec_err_3d;
	typedef std::vector< vec_err_3d > vec_err_4d;
	
	typedef std::vector< j_param_t > vec_params;
	typedef std::vector< vec_params > vec_params_2d;
	typedef std::vector< vec_params_2d > vec_params_3d;
	
	typedef std::vector< j_param_t > population_params;
	typedef std::vector< population_params > vec_pop_params;
	
	
	public:
		OptCtrl() {
			std::cout << "opt_ctrl started" << std::endl;
			
			log_enable_ = true;
			std::string log_path = "/opt/shared/developer/logs/arm_test/";
			std::string time_str;
			nice_time_string( time_str );
			joint_log_filename_ = log_path + "opt_ctrl.joints." + time_str + ".log";
			pop_log_filename_ = log_path + "opt_ctrl.gen_pop." + time_str + ".log";
			
			generation_ = 0;
			max_population_ = 30;
			max_generation_ = 50;
			reset_count_ = 0;
			//readPopulation();		// reads starting population from a text file (optional)
			
			// subscribe/advertise topics
			vec_sub_err_.resize( 10 );
			vec_pub_params_.resize( 10 );
			vec_time_.resize( 10 );
			vec_vel_err_.resize( 10 );
			vec_pos_err_.resize( 10 );
			vec_params_.resize( 10 );
			for( int bot_nr=1; bot_nr<=10; bot_nr++ ) {
				vec_sub_err_[bot_nr-1].resize( 1 );
				vec_pub_params_[bot_nr-1].resize( 1 );
				vec_time_[bot_nr-1].resize( 1 );
				vec_vel_err_[bot_nr-1].resize( 1 );
				vec_pos_err_[bot_nr-1].resize( 1 );
				vec_params_[bot_nr-1].resize( 1 );
				for( int leg_nr=1; leg_nr<=1; leg_nr++ ) {
					vec_sub_err_[bot_nr-1][leg_nr-1].resize( 4 );
					vec_pub_params_[bot_nr-1][leg_nr-1].resize( 4 );
					vec_time_[bot_nr-1][leg_nr-1].resize( 4 );
					vec_vel_err_[bot_nr-1][leg_nr-1].resize( 4 );
					vec_pos_err_[bot_nr-1][leg_nr-1].resize( 4 );
					vec_params_[bot_nr-1][leg_nr-1].resize( 4 );
					for( int joint_nr=2; joint_nr<=3; joint_nr++ ) {
						char path[256];
						// topic address, e.g. "/test_01/leg_1_joint_3"
						
						// subscribe to joint error topic
						snprintf( path, sizeof(path), "/test_%02i/leg_%i_joint_%i_errors", bot_nr, leg_nr, joint_nr );
						vec_sub_err_[bot_nr-1][leg_nr-1][joint_nr] = nh_.subscribe< joint_err >(
							path,
							10,	// message queue/buffer size
							boost::bind( &OptCtrl::subErrCallback, this, _1, bot_nr, leg_nr, joint_nr )
						);
						
						// advertise joint param publisher
						snprintf( path, sizeof(path), "/test_%02i/leg_%i_joint_%i_str_param", bot_nr, leg_nr, joint_nr );
						vec_pub_params_[bot_nr-1][leg_nr-1][joint_nr] = nh_.advertise< std_msgs::String >( path, 2 );
						
						// no need to initialize the ros::Time object. we will do so on the first callback call
						vec_time_[bot_nr-1][leg_nr-1][joint_nr] = ros::Time(0,0);
					}
				}
			}
			
		};
		
		/// @brief called to reset all states of the object
		void reset() {
			reset_count_++;
			std::cout << std::endl << "==================  R E S E T  (opt_ctrl, reset #" << reset_count_ << ")  ==================" << std::endl << std::endl;
			
			// set the current generation to 0
			generation_ = 0;
			
			// empty parameter pool
			vec_pop_params_.resize( 0 );
			
			// reset the time (this will enforce new parameters being applied to the joints on the next callback)
			for( int bot_nr=1; bot_nr<=10; bot_nr++ ) {
				for( int leg_nr=1; leg_nr<=1; leg_nr++ ) {
					for( int joint_nr=2; joint_nr<=3; joint_nr++ ) {
						vec_time_[bot_nr-1][leg_nr-1][joint_nr] = ros::Time(0,0);
					}
				}
			}
			
			// new filename
			std::string log_path = "/opt/shared/developer/logs/arm_test/";
			std::string time_str;
			nice_time_string( time_str );
			joint_log_filename_ = log_path + "opt_ctrl.joints." + time_str + ".log";
			pop_log_filename_ = log_path + "opt_ctrl.gen_pop." + time_str + ".log";
			
			// close old files (new files will be opened automatically)
			pop_log_file_.close();
			joint_log_file_.close();
		}
		
		/// @brief called when the joint error is published
		void subErrCallback( const gazebo_crab_plugin::pid_joint_error::ConstPtr &msg, int bot_nr, int leg_nr, int joint_nr ) {
			// save the joint error
			double error = msg->velocity_error;
			vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr].push_back( error*error );	// save the square (velocity) error
			error = msg->angle_error;
			vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr].push_back( error*error );	// save the square (position/angle) error
			
			// we only react on events from joints with joint number 3 (last in the kinematic chain)
			if( joint_nr != 3 )
				return;
			
			//std::cout << "subErrCallback(" << bot_nr << ", " << leg_nr << ", " << joint_nr << ")" << std::endl;
			double p,i,d,i_clamp,max_vel,damping;
			if( vec_time_[bot_nr-1][leg_nr-1][joint_nr].isZero() ) {
				
				std::vector< j_param_t > vec_new_params( 4 );
				
				// set parameters of the joint and set time to now
				generateParams( vec_new_params );
				for( int j=2; j<=3; j++ ) {
					char str_params[256];
					snprintf( str_params, sizeof(str_params), "%f %f %f %f %f 1.0 %f %f",
						vec_new_params[j].p,
						vec_new_params[j].i,
						vec_new_params[j].d,
						vec_new_params[j].i_clamp,
						-vec_new_params[j].i_clamp,
						vec_new_params[j].max_vel,
						vec_new_params[j].damping
					);
					std::cout << "  first time initialization for "
						<< bot_nr << "."
						<< leg_nr << "."
						<< j
						<< "[" << str_params << "]"
						<< std::endl;
					vec_params_[bot_nr-1][leg_nr-1][j] = vec_new_params[j];
					std_msgs::String msg_param;
					msg_param.data = str_params;
					vec_pub_params_[bot_nr-1][leg_nr-1][j].publish( msg_param );
					
					vec_time_[bot_nr-1][leg_nr-1][j] = ros::Time::now();
				}
				return;
			} else {
				//std::cout << "  valid" << std::endl;
				//return;
				// check duration
				ros::Duration dt = ros::Time::now() - vec_time_[bot_nr-1][leg_nr-1][joint_nr];
				if( dt.sec < 5 )		// we want at least 5 seconds of movement before we compute the error
					return;
			}
			
			// reaching this line, it is time to set new parameters for the joint
			
			generation_++;
			// check if we have exceeded the maximum number of generations.
			if( generation_ / max_population_ > max_generation_ ) {
				reset();
				return;
			}
			
			// get the parameters for joint 2 & 3
			std::vector< j_param_t > vec_old_params;
			vec_old_params.resize( 4 );
			vec_old_params[2] = vec_params_[bot_nr-1][leg_nr-1][2];
			vec_old_params[3] = vec_params_[bot_nr-1][leg_nr-1][3];
			
			// compute square mean errors for both joints
			computeError( bot_nr, leg_nr, joint_nr, vec_old_params[2].vel_sq_mean_error, vec_old_params[2].pos_sq_mean_error );
			computeError( bot_nr, leg_nr, joint_nr, vec_old_params[3].vel_sq_mean_error, vec_old_params[3].pos_sq_mean_error );
			
			// reset vectors that save the error per step
			vec_vel_err_[bot_nr-1][leg_nr-1][2].resize( 0 );
			vec_pos_err_[bot_nr-1][leg_nr-1][2].resize( 0 );
			vec_vel_err_[bot_nr-1][leg_nr-1][3].resize( 0 );
			vec_pos_err_[bot_nr-1][leg_nr-1][3].resize( 0 );
			
			// debug message
			/*
			std::cout << "error for [" << bot_nr << ", " << leg_nr << "]: "
				<<  vec_old_params[2] << " | "
				<<  vec_old_params[3]
				<< std::endl;
			*/
			
			if( log_enable_ ) {
				do {
					if( !joint_log_file_.is_open() ) {
						joint_log_file_.open( joint_log_filename_ );
						if( !joint_log_file_.is_open() ) {
							std::cout << "failed to open log file '" << joint_log_filename_ << "'" << std::endl;
							break;
						}
					}
					
					joint_log_file_ << "joint=" << bot_nr << "." << leg_nr << "." << 2
						<< " " << vec_old_params[2]
						<< " joint="<< bot_nr << "." << leg_nr << "." << 3
						<< " " << vec_old_params[2] << std::endl;
				} while( 0 );
			}
			
			// save parameter set (if error is better then the current worst particle)
			poolParams( vec_old_params );
			
			std::vector< j_param_t > vec_new_params( 4 );
			//vec_new_params.resize( 4 );
			//double p,i,d,i_clamp,max_vel,damping;
			generateParams( vec_new_params );
			//std::cout << "new parameter set generated" << std::endl;
			for( int j=2; j<=3; j++ ) {
				vec_params_[bot_nr-1][leg_nr-1][j] = vec_new_params[j];
				char str_params[256];
				snprintf( str_params, sizeof(str_params), "%f %f %f %f %f 1.0 %f %f",
					vec_new_params[j].p,
					vec_new_params[j].i,
					vec_new_params[j].d,
					vec_new_params[j].i_clamp,
					-vec_new_params[j].i_clamp,
					vec_new_params[j].max_vel,
					vec_new_params[j].damping
				);
				std_msgs::String msg_param;
				msg_param.data = str_params;
				vec_pub_params_[bot_nr-1][leg_nr-1][j].publish( msg_param );
				
				vec_time_[bot_nr-1][leg_nr-1][j] = ros::Time::now();
			}
		};
		
		
		/** @brief if the parameters in 'params' have a lower error rating than the highest error ranking
		 *        in our population than we replace the highest ranked parameter set with 'params'
		 */
		void poolParams( j_param_t params ) {
			int length = pop_params_.size();		// current size of the population
			
			if( params.vel_sq_mean_error <= 0 ) {
				std::cout << "invalid error value - skipping particle" << std::endl;
				return;
			}
			
			// if our population pool is not full yet, we simply add the particle to the pool
			if( length < max_population_ ) {
				pop_params_.push_back( params );
				return;
			}
			
			// sort by error (smalles (combined) error has the smalles index)
			std::sort( pop_params_.begin(), pop_params_.end(), p_sort_fun );
			
			// check if out current error is smaller than the maximum stored error
			if( pop_params_[length-1].error() > params.error() ) {
				pop_params_[length-1] = params;
				printPopulation();
			}
		};
		
		
		void poolParams( std::vector< j_param_t > &params ) {
			int pop_size = vec_pop_params_.size();		// current size of the population
			
			for( int i=2; i<=3; i++ ) {
				if( params[i].vel_sq_mean_error <= 0  ||  params[i].pos_sq_mean_error <= 0 ) {
					std::cout << "invalid error value - skipping particle (joint=" << i << ", errors="
						<< params[i].vel_sq_mean_error << ", "
						<< params[i].pos_sq_mean_error << ")"
						<< std::endl;
					return;
				}
			}
			
			// if our population pool is not full yet, we simply add the particle to the pool
			if( pop_size < max_population_ ) {
				vec_pop_params_.push_back( params );
				printVecPopulation();
				return;
			}
			
			// sort by error (smalles (combined) error has the smalles index)
			std::sort( vec_pop_params_.begin(), vec_pop_params_.end(), vec_p_sort_fun );
			
			// check if out current error is smaller than the maximum error in the population
			if( vec_p_sort_fun(params, vec_pop_params_[pop_size-1]) ) {
				vec_pop_params_[pop_size-1] = params;
				printVecPopulation();
			}
		};
		
		
		/// @brief reads a starting population from a file. if the file is empty or cannot be read the starting population will be empty
		void readPopulation() {
			char path[256];
			snprintf( path, sizeof(path), "/opt/shared/developer/logs/opt_ctrl.input.txt" );
			std::ifstream file;
			std::string line;
			
			file.open( path, std::ifstream::in );
			
			do {
				if( file.eof() )
					break;
				
				// read the input line by line
				std::getline( file, line );
				
				// trim whitespaces from both sides
				boost::algorithm::trim( line );
				if( line.size() < 1 )
					continue;
				
				// split into tokens (such as "p=0.0123")
				std::vector<std::string> nv_pairs;	// name-value pairs (in a single string), if well formated
				boost::split( nv_pairs, line, boost::is_any_of(" ") );
				
				j_param_t params;
				memset( &params, 0, sizeof(params) );	// reset the structure
				params.multiplier = 1.0;	// we currently don't use this in the optimization process, so we set to the default value
				for( int i=0; i<nv_pairs.size(); i++ ) {
					// split name-value pair into name and value (both as string)
					std::vector<std::string> nv_vec;
					boost::split( nv_vec, nv_pairs[i], boost::is_any_of("=") );
					if( nv_vec.size() < 2 ) {
						std::cout << "warning: malformed token in input #" << i <<", size=" << nv_pairs.size() << "(" << nv_pairs[i] << ")" << std::endl;
						continue;
					}
					if( nv_vec[0] == "error" ) {	// old name for vel_error
						params.vel_sq_mean_error = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "vel_error" ) {
						params.vel_sq_mean_error = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "pos_error" ) {
						params.pos_sq_mean_error = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "p" ) {
						params.p = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "i" ) {
						params.i = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "d" ) {
						params.d = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "i_clamp" ) {
						params.i_clamp = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "v_max" ) {
						params.max_vel = atof( nv_vec[1].c_str() );
					} else if(nv_vec[0] == "damp" ) {
						params.damping = atof( nv_vec[1].c_str() );
					} else {
						std::cout << "warning: unrecognized token (" << nv_pairs[i] << ")" << std::endl;
					}
				}
				
				// perform a simple check: the velocity error must be positive
				if( params.vel_sq_mean_error <= 0 ) {
					std::cout << "warning: invalid velocity error in params (" << params.vel_sq_mean_error << ")" << std::endl;
					continue;
				}
				
				pop_params_.push_back( params );	// note: we allow to set a higher population than max_population_
			} while( 1 );
		}
		
		
		/// @brief computes the total error of the given joint. the error is set to NaN if the vector length is zero.
		void computeError( int bot_nr, int leg_nr, int joint_nr, double &vel_error, double &pos_error ) {
			double sum;
			int length;
			
			length = vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr].size();
			if( length < 1 ) {
				vel_error = std::numeric_limits<double>::quiet_NaN();
				std::cout << "empty vel error vector for joint" 
					<< bot_nr << "."
					<< leg_nr << "."
					<< joint_nr << "."
					<< std::endl;
			} else {
				/*
				sum = std::accumulate(
					vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr].begin(),
					vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr].end(),
					0 );//#include <numeric>
				*/
				sum = 0.0;
				for( int i=0; i<length; i++ ) {
					if( vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr][i] <= 0.0 )
						std::cout << "invalid entry: " << vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr][i] << std::endl;
					sum += vec_vel_err_[bot_nr-1][leg_nr-1][joint_nr][i];
				}
				vel_error = sum / length;
			}
			
			length = vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr].size();
			if( length < 1 ) {
				pos_error = std::numeric_limits<double>::quiet_NaN();
				std::cout << "empty pos error vector for joint" 
					<< bot_nr << "."
					<< leg_nr << "."
					<< joint_nr << "."
					<< std::endl;
			} else {
				/* DOES NOT WORK, DON'T KNOW WHY
				sum = std::accumulate(
					vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr].begin(),
					vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr].end(),
					0 );
				*/
				sum = 0.0;
				for( int i=0; i<length; i++ ) {
					if( vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][i] <= 0.0 )
						std::cout << "invalid entry: " << vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][i] << std::endl;
					sum += vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][i];
				}
				pos_error = sum / length;
			}
			
			/* debug messages
			std::cout << "computed errors: vel=" << vel_error << ", pos=" << pos_error << std::endl;
			std::cout << "  sum=" << sum << ", length=" << length << std::endl;
			std::cout << "  "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][0] << ", "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][1] << ", "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][2] << ", "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][3] << ", "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][4] << ", "
				<< vec_pos_err_[bot_nr-1][leg_nr-1][joint_nr][5] << ", "
				<< std::endl;
			*/
		};
		
		
		/// @brief generates a parameter set. at the moment all parameters are choosen randomly, however this will be changed in the future
		void generateParams( double &p, double &i, double &d, double &i_clamp, double &max_vel, double &damping ) {
			generateParams2( p, i, d, i_clamp, max_vel, damping );
		};
		
		
		void generateParams( std::vector<j_param_t> &params ) {
			generateParams2( params );
		};
		
		
		/// @brief chooses the parameters randomly from pre-set intervals. does not take any particles into account
		void generateParamsBlind( double &p, double &i, double &d, double &i_clamp, double &max_vel, double &damping ) {
			//std::default_random_engine generator;
			std::normal_distribution<double> distribution(5.0,2.0);
			std::uniform_real_distribution<double> uni_dist(0.0001,0.1);
			
			p = distribution( generator_ );
			i = 0.001 * distribution( generator_ );
			d = 0.001 * distribution( generator_ );
			i_clamp = 0.01 * distribution( generator_ );
			max_vel = distribution( generator_ );
			damping = uni_dist( generator_ );
			
			p = p > 0.01 ? p : 0.01;
			i = i > 0.0 ? i : 0.0;
			d = d > 0.0 ? d : 0.0;
			i_clamp = i_clamp > 0.0 ? i_clamp : 0.0;
			max_vel = max_vel > 0.5 ? max_vel : 0.5;
		};
		
		
		/// @brief chooses the parameters randomly from pre-set intervals. does not take any particles into account
		void generateParamsBlind( std::vector<j_param_t> &params ) {
			//std::default_random_engine generator;
			std::normal_distribution<double> norm_dist(5.0,2.0);
			std::uniform_real_distribution<double> uni_dist(0.0001,0.1);
			
			if( params.size() < 4 )
				params.resize(4);
			
			for( int joint_nr=2; joint_nr<=3; joint_nr++ ) {
				// set random values
				params[joint_nr].p = norm_dist( generator_ );
				params[joint_nr].i = 0.001 * norm_dist( generator_ );
				params[joint_nr].d = 0.001 * norm_dist( generator_ );
				params[joint_nr].i_clamp = 0.01 * norm_dist( generator_ );
				params[joint_nr].max_vel = norm_dist( generator_ );
				params[joint_nr].damping = uni_dist( generator_ );
				
				// ensure valid parameters
				params[joint_nr].p = params[joint_nr].p > 0.01 ? params[joint_nr].p : 0.01;
				params[joint_nr].i = params[joint_nr].i > 0.0 ? params[joint_nr].i : 0.0;
				params[joint_nr].d = params[joint_nr].d > 0.0 ? params[joint_nr].d : 0.0;
				params[joint_nr].i_clamp = params[joint_nr].i_clamp > 0.0 ? params[joint_nr].i_clamp : 0.0;
				params[joint_nr].max_vel = params[joint_nr].max_vel > 0.5 ? params[joint_nr].max_vel : 0.5;
				
				std::cout << "BLIND <" << joint_nr << "> " << params[joint_nr] << std::endl;
			}
		};
		
		
		void generateParams2( double &p, double &i, double &d, double &i_clamp, double &max_vel, double &damping ) {
			// choose a random particle
			int length = pop_params_.size();
			if( length < 1 ) {
				std::cout << "warning - empty population" << std::endl;
				generateParamsBlind( p, i, d, i_clamp, max_vel, damping );
				return;
			}
			std::uniform_int_distribution<int> uni_int_dist(0,length-1);
			int index = uni_int_dist( generator_ );
			
			j_param_t params = pop_params_[index];
			
			std::uniform_real_distribution<double> uni_real_dist( 0.0, 1.0 );
			boost::math::beta_distribution<> beta_dist( 2, 2 );
			
			// multiply the value of the select particle with a random number from [0.5,1.5], choosen with a beta distribution (a=2,b=2)
			p = params.p * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			i = params.i * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			d = params.d * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			i_clamp = params.i_clamp * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			max_vel = params.max_vel * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			damping = params.damping * (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
		}
		
		
		void generateParams2( std::vector<j_param_t> &params ) {
			int pop_size = vec_pop_params_.size();
			// note: use "pop_size < 1" if you want to use the pool as soon as posible, "pop_size < max_population" if you
			// want all random parameters until the pool is full
			//if( pop_size < 1 ) {
			if( pop_size < max_population_ ) {
				std::cout << "pool not full yet (size=" << pop_size << "), creating blind parameters" << std::endl;
				generateParamsBlind( params );
				return;
			}
			// choose a random particle from the stored population (uniform distribution)
			std::uniform_int_distribution<int> uni_int_dist(0,pop_size-1);
			int index = uni_int_dist( generator_ );
			params = vec_pop_params_[index];
			
			std::uniform_real_distribution<double> uni_real_dist( 0.0, 1.0 );
			boost::math::beta_distribution<> beta_dist( 2, 2 );
			for( int joint_nr=2; joint_nr<=3; joint_nr++ ) {
				//double rand_uni = uni_real_dist( generator_ );
				
				// multiply the value of the select particle with a random number from [0.5,1.5], choosen with a beta distribution (a=2,b=2)
				params[joint_nr].p *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
				params[joint_nr].i *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
				params[joint_nr].d *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
				params[joint_nr].i_clamp *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
				params[joint_nr].max_vel *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
				params[joint_nr].damping *= (0.5 + boost::math::quantile(beta_dist, uni_real_dist( generator_ )) );
			}
		}
		
		
		/// @brief prints the population
		void printPopulation() {
			std::sort( pop_params_.begin(), pop_params_.end(), p_sort_fun );
			int length = pop_params_.size();
			
			std::cout << "population (size=" << length << ") top 10:" << std::endl;
			for( int i=0; i<length && i<10; i++ ) {
				std::cout << "  " << pop_params_[i] << std::endl;
				/*
				std::cout << "  error=" << pop_params_[i].vel_sq_mean_error
					<< " p=" << pop_params_[i].p
					<< " i=" << pop_params_[i].i
					<< " d=" << pop_params_[i].d
					<< " i_clamp=" << pop_params_[i].i_clamp
					<< " v_max=" << pop_params_[i].max_vel
					<< " damp=" << pop_params_[i].damping
					<< std::endl;
				*/
			}
		};
		
		
		void printVecPopulation() {
			std::sort( vec_pop_params_.begin(), vec_pop_params_.end(), vec_p_sort_fun );
			int pop_size = vec_pop_params_.size();
			
			std::cout << "gen " << generation_
				<< " (" << (generation_/30+1) << ")"
				<< ", population (size=" << pop_size << ") top 10:"
				<< std::endl;
			/*
			for( int i=0; i<pop_size && i<10; i++ ) {
				for( int j=2; j<=3; j++ ) {
					std::cout << " joint=" << j << " " << vec_pop_params_[i][j];
				}
				std::cout << std::endl;
			}
			*/
			
			if( log_enable_ ) {
				do {
					if( !pop_log_file_.is_open() ) {
						pop_log_file_.open( pop_log_filename_ );
						if( !pop_log_file_.is_open() ) {
							std::cout << "failed to open log file '" << pop_log_filename_ << "'" << std::endl;
							break;
						}
					}
					
					for( int i=0; i<pop_size; i++ ) {
						for( int j=2; j<=3; j++ ) {
							pop_log_file_ << " generation=" << generation_ << " joint=" << j << " " << vec_pop_params_[i][j];
						}
						pop_log_file_ << std::endl;
					}
				} while( 0 );
			}
		}
		
		
	private:
        ros::NodeHandle nh_;
		std::default_random_engine generator_;	// random number generator (requires C++11)
        vec_sub_3d vec_sub_err_;		// subscribers (joint error)
        vec_pub_3d vec_pub_params_;		// publisher (joint parameters)
        vec_time_3d vec_time_;			// timestamp of last param update
        vec_err_4d vec_vel_err_;		// vector of errors since param update
        vec_err_4d vec_pos_err_;		// vector of errors since param update
        vec_params_3d vec_params_;		// current parameters of the joints
        int max_population_;			// maximum number of entries in pop_params_
		population_params pop_params_;	// saved parameters of the population (for single joint optimization)
		vec_pop_params vec_pop_params_;	// saved parameters of the population (for multi-joint (e.g. full arm/leg) optimization)
		bool log_enable_;				// if true, we write the results to a log file
		std::string joint_log_filename_;	// filename (including path) of the log file (joints)
		std::ofstream joint_log_file_;		// file object for the log data (joints)
		std::string pop_log_filename_;		// filename (including path) of the log file (gen population)
		std::ofstream pop_log_file_;		// file object for the log data (gen population)
		int generation_;				// the current generation
		int max_generation_;			// if our current generation is bigger than this, then we reset everything and start anew (including a new log file)
		int reset_count_;				// number of resets since start
};






} // end of namespace 'opt_ctrl'




int main( int argc, char** argv ) {
    ros::init(argc, argv, "opt_ctrl");
    opt_ctrl::OptCtrl opt_ctrl;

	ros::spin();
}






















