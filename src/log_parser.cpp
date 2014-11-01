// project headers
#include "../include/gazebo_crab_plugin/joint_param.hpp"

// C++ headers
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>



// BOOST headers
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
//#include <boost/lexical_cast.hpp>


namespace fs = boost::filesystem;


typedef std::pair< std::string, std::time_t > file_entry_t;		// filename and last-write time as a pair
typedef std::vector< file_entry_t > file_entry_list_t;			// vector of files (for sorting by last-write time)


/** @brief this class stores information about a single generation, which typically contains multiple particles.
 */
class GenInfo {
	public:
		GenInfo() : gen_nr_(-1), pop_size_(0) {};
		
		/// @brief computes the errors (mean/min/max) over all particles in this generation
		bool computeErrors() {
			int length = params_.size();
			double sum=0, mean=0, min=0, max=0;
			if( !length ) {
				std::cout << "empty set, cannot compute errors" << std::endl;
				return false;
			}
			//std::cout << "computing errors" << std::endl;
			
			sum = min = max = params_[0].error();
			
			for( int i=1; i<length; i++ ) {
				double error = params_[i].error();
				if( error < min )
					min = error;
				if( max < error )
					max = error;
				sum += error;
			}
			
			mean = sum / length;
			/*
			std::cout << "computed errors: mean=" << mean
				<< ", min=" << min
				<< ", max=" << max
				<< std::endl;
			*/
			
			mean_error_ = mean;
			min_error_ = min;
			max_error_ = max;
			
			return true;
		}
		
		/// @brief '<<' operator for convenient output to files or cout. format is an easy-to-plot space-separated "csv" format
		friend std::ostream& operator<< (std::ostream& stream, const GenInfo& gen_info) {
			return stream << gen_info.gen_nr_
				<< " " << gen_info.pop_size_
				<< " " << gen_info.mean_error_
				<< " " << gen_info.min_error_
				<< " " << gen_info.max_error_;
		}
		
		void push( j_param_t p ) {
			params_.push_back( p );
		}
		
		std::vector< j_param_t > params_;	// list of the parameters of the population of this generation
		int gen_nr_;				// generation number. this is set to -1 by the default constuctor, indicating that the other values have yet to be set
		int pop_size_;				// population size of this generation
		double mean_error_;			// mean square error (combined, weighted error of velocity and position error. first squared, then weighted and added)
		double min_error_;			// minimum square error within this generation
		double max_error_;			// maximum square error within this generation
};



bool file_sort_fun( file_entry_t left, file_entry_t right ) {
	return left.second > right.second;
}




class LogParser {
	
	
	public:
		LogParser() : verbose_(true), max_generation_(100) {
			
			askInFilename();
			/*
			std::string filename = "/opt/shared/developer/logs/arm_test/opt_parser_test.log";
			std::ifstream file;
			// for testing:
			file.open( "/opt/shared/developer/logs/arm_test/opt_ctrl.gen_pop.2014-10-24_10:28:59.log" );
			//file.open( filename.c_str() );
			do {
				if( !file.is_open() ) {
					std::cout << "failed to open file" << std::endl;
					return;
				}
				parse( file );
			} while( 0 );
			file.close();
			
			std::ofstream out;
			out.open( "/opt/shared/developer/logs/arm_test/opt_parser.mmm.log" );	// mmm = mean,min,max
			if( out.is_open() ) {
				writeMMM( out );
				out.close();
			}
			
			//std::ofstream out;
			out.open( "/opt/shared/developer/logs/arm_test/opt_parser.particles.log" );	// mmm = mean,min,max
			if( out.is_open() ) {
				writeParticles( out );
				out.close();
			}
			*/
		};
		
		void askInFilename() {
			std::string folder = "/opt/shared/developer/logs/arm_test";
			fs::path someDir( folder );
			file_entry_list_t file_list;
			fs::directory_iterator end_iter;
			getFileList( folder, file_list );
			
			
			std::cout << "select input file (list size=" << file_list.size() << "):" << std::endl;
			for( int i=0; i<file_list.size() && i<10; i++ ) {
				std::cout << "  " << i
					<< ": " << file_list[i].first
					<< "  " << niceFileSize( fs::file_size(folder + "/" + file_list[i].first) )
					<< std::endl;
			}
			int index;
			std::cin >> index;
			if( index < 0  &&  index >= file_list.size() ) {
				std::cout << "invalid index" << std::endl;
				return;
			}
			
			inFilenameCallback( folder + "/" + file_list[index].first );
		}
		
		std::string niceFileSize( unsigned int size ) {
			char str[64];
			char modifier = 0;
			if( size > 1000*1000 ) {
				size /= 1000*1000;
				modifier = 'M';
			} else if( size > 1000 ) {
				size /= 1000;
				modifier = 'k';
			}
			if( modifier ) {
				snprintf( str, sizeof(str), "%5u%cB", size, modifier );
			} else {
				snprintf( str, sizeof(str), "%5uB", size );
			}
			
			return str;
		}
		
		void getFileList( const std::string folder, file_entry_list_t &file_list ) {
			fs::path path( folder );
			fs::directory_iterator iter_end;
			
			// check if path exists
			if ( !fs::exists(path)  ||  !fs::is_directory(path) ) {
				std::cout << "invalid path: '" << folder << "'" << std::endl;
				return;
			}
			
			// iterate over all files in the folder
			for( fs::directory_iterator iter(path) ; iter != iter_end ; ++iter) {
				if (!fs::is_regular_file(iter->status()) ) {
					continue;
				}
					
				// check filename
				if( !boost::starts_with(iter->path().filename().string(), "opt_ctrl.gen_pop.")) {
					continue;
				}
				
				file_list.push_back( file_entry_t(iter->path().filename().string(), fs::last_write_time(iter->path()) ));
			}
			
			// sort the vector
			std::sort( file_list.begin(), file_list.end(), file_sort_fun );

		}
		
		void inFilenameCallback( const std::string &filename ) {
			std::ifstream file;
			file.open( filename.c_str() );
			do {
				if( !file.is_open() ) {
					std::cout << "failed to open file" << std::endl;
					return;
				}
				parse( file );
			} while( 0 );
			file.close();
			
			std::ofstream out;
			out.open( "/opt/shared/developer/logs/arm_test/opt_parser.mmm.log" );	// mmm = mean,min,max
			if( out.is_open() ) {
				writeMMM( out );
				out.close();
			} else {
				std::cout << "failed to open output file" << std::endl;
			}
			
			//std::ofstream out;
			out.open( "/opt/shared/developer/logs/arm_test/opt_parser.particles.log" );
			if( out.is_open() ) {
				writeParticles( out );
				out.close();
			} else {
				std::cout << "failed to open output file" << std::endl;
			}
			
			out.open( "/opt/shared/developer/logs/arm_test/opt_parser.params.log" );
			if( out.is_open() ) {
				writeParams( out );
				out.close();
			} else {
				std::cout << "failed to open output file" << std::endl;
			}
			
		}
		
		void parse( std::ifstream &file ) {
			std::string line;
			const std::string delimiter = "generation=";
			int line_count = 0;
			int generation;
			std::vector< std::string > params_set;		// a string containing the parameters for one or more j_params_t structures (an input line may contain multiple of those, typically one for each joint)
			std::string params_str;						// a string containing the parameters for exactly one j_params_t structure
			std::vector< std::string > parse_tokens;	// vector of space-separated tokens
			std::vector< std::string > vec_nv;			// vector of two strings: name and value
			std::vector< j_param_t > vec_params;		// prameters of the current generation
			vec_gen_info_.resize(1);
			
			std::cout << "reading from file" << std::endl;
			// read line by line
			while ( 1 ) {
				// read line
				std::getline( file, line );
				line_count++;
				
				// debug message (info about the current line)
				//if( verbose_) {
				//	std::cout << "reading line #" << line_count << " (length=" << line.size() << ")" << std::endl;
				//	std::cout << "  >" << line << std::endl;
				//}
				
				// trim whitespaces from both sides. note: removes any '\r' that might be present at the end of the line, too.
				boost::algorithm::trim( line );
				
				// lines that start with a '#' are comments. we ignore these
				if( line[0] == '#' )
					continue;
				
				// split the strings, so that each substring contains the parameters for a single joint. strings start with 'generation='.
				size_t start = 0;
				size_t end = line.find( delimiter );
				do {
					// create a sub-string that contains the parameters for a single joint
					start = end;
					if( end == std::string::npos  ||  line.size() <= end+11 ) {
						break;
					} else {
						end = line.find(delimiter, end+11);
						if( end == std::string::npos ) {
							params_str = line.substr(start, end);
						} else {
							params_str = line.substr(start, end - start);
						}
					}
					
					//std::cout << "# " << params_str << std::endl;
					
					boost::split( params_set, params_str, boost::is_any_of(" ") );
					
					// reset vector (faster than creating a new one every time)
					parse_tokens.resize( 0 );
					
					// spit the line into tokens that are separated by whitespaces (note: tokens might be empty)
					boost::split( parse_tokens, line, boost::is_any_of(" ") );
					
					// parse all tokens, one by one, until we find the 'generation' token
					for( int i=0; i<parse_tokens.size(); i++ ) {
						// ignore empty tokens
						if( parse_tokens[i].size() < 1 )
							continue;
						
						
						vec_nv.resize( 0 );
						
						// split token into name and value strings
						boost::split( vec_nv, parse_tokens[i], boost::is_any_of("=") );
						if( vec_nv.size() != 2 ) {
							std::cout << "warning: invalid token ('" << parse_tokens[i] << "'" << std::endl;
							continue;
						}
						
						
						if( vec_nv[0] == "generation" ) {
							//generation = atoi(vec_nv[1].c_str() );
							generation = (std::stoi( vec_nv[1] )-1) / 30 + 1;	// we consider it a generation when 10 particles have been tested, and start counting with 1 instead of 0
							//std::cout << "  gen=" << vec_nv[1] << std::endl;
							break;
						}
						
					}
					
					if( generation > max_generation_ )
						break;
					
					if( vec_gen_info_.size() < generation+1 )
						vec_gen_info_.resize( generation+1 );	// note: index 0 is unused in this structure
					
					j_param_t params;
					params.apply( params_str );
					
					vec_gen_info_[generation].push( params );
					
				} while( end != std::string::npos );
								
				if( file.eof() ) {
					std::cout << "finished after reading " << line_count << " lines" << std::endl;
					break;
				}
			}
			
			std::cout << "finished parsing" << std::endl;
		}
		
		/** @brief writes generation #, mean, min & max to the log file (in that order)
		 */
		void writeMMM( std::ofstream &file ) {
			for( int i=0; i<vec_gen_info_.size(); i++ ) {
				if( !vec_gen_info_[i].computeErrors() )
					continue;
				
				file << i
					<< " " << vec_gen_info_[i].mean_error_
					<< " " << vec_gen_info_[i].min_error_
					<< " " << vec_gen_info_[i].max_error_
					<< std::endl;
			}
		}
		
		/** @brief writes generation # and particle errors (joint 1, joint 2, joint 1+2) to the log file
		 */
		void writeParticles( std::ofstream &file ) {
			for( int g=0; g<vec_gen_info_.size(); g++ ) {
				for( int p=0; p<vec_gen_info_[g].params_.size(); p++ ) {
					double error;
					double e = vec_gen_info_[g].params_[p].error();
					// each particle handles two joints, hence the modulo 2
					if( p%2 == 0 ) {
						file << g << " " << e;	// generation number and error of the first joint
						error = e;
					} else {
						error += e;
						file << " " << e << " " << error << std::endl;	// error of the second joint, combined error and end of line
					}
				}
			}
		}
		
		/** @brief writes generation number and all params to the log file. each line
		 *         starts with the generation number and the parameters of all joints of the particle.
		 */
		void writeParams( std::ofstream &file ) {
			for( int g=0; g<vec_gen_info_.size(); g++ ) {
				for( int p=0; p<vec_gen_info_[g].params_.size(); p++ ) {
					if( p%2 == 0 )
						file << g;
					
					file
						<< " " << vec_gen_info_[g].params_[p].p
						<< " " << vec_gen_info_[g].params_[p].i
						<< " " << vec_gen_info_[g].params_[p].d
						<< " " << vec_gen_info_[g].params_[p].i_clamp
						<< " " << vec_gen_info_[g].params_[p].multiplier
						<< " " << vec_gen_info_[g].params_[p].max_vel
						<< " " << vec_gen_info_[g].params_[p].damping;
					
					if( p%2 == 0 ) {
					} else {
						file << std::endl;
					}
				}
				file << std::endl;
			}
		}
		
	private:
		
		std::vector< GenInfo > vec_gen_info_;
		bool verbose_;
		int max_generation_;
};








int main( int argc, char **argv ) {
	LogParser parser;
	return 0;
}

















