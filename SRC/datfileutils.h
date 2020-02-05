#ifndef _DATFILEUTILS_H
#define _DATFILEUTILS_H
//==============================================================================
/** Utils that should simplify reading of MUSCOD-II datfiles
 *
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <errno.h>

/** \brief parses a datfile and returns the stripped line that follows the 
    line with value id
 
  e.g. for a datfile containing 
  ----
  c3d_file
     blaaa mya file * a comment that does not get returned
  ----
 
  a call to datfile_get_string("c3dfile" would return "blaa mya file".
 */
std::string datfile_get_string (const std::string &filename, 
                                const std::string &id);

int datfile_get_int (const std::string &filename, 
                     const std::string &id);

double datfile_get_double ( const std::string &filename, 
                            const std::string &id);

inline RigidBodyDynamics::Math::VectorNd 
        datfile_get_vector (const std::string &filename, 
                                    std::string id, 
                                    int stage_id = -1, 
                                    int node_id = -1);

/** \brief Strips comments and whitespaces from the beginning 
and the start of a string
 */
inline std::string strip_datfile_line (std::string line) {
    const std::string remove_chars ("\r\n \t");

    // strip comments
    if (line.find ("*") != std::string::npos) {
        line = line.substr (0, line.find("*") - 1);
    }

    if (line.find ("!") != std::string::npos) {
        line = line.substr (0, line.find("!") - 1);
    }

    // strip values from the front
    while (remove_chars.find (line[0]) != std::string::npos) {
        line = line.substr (1, line.size());

        // break out if there are no chars anymore
        if (line.size() == 0)
            break;
    }

    // break out if there are no chars anymore
    if (line.size() == 0)
        return "";

    // strip values from the back
    while (remove_chars.find (line[line.size() - 1]) != std::string::npos) {
        line = line.substr(0, line.size() - 1);

        // break out if there are no chars anymore
        if (line.size() == 0)
            break;
    }

    return line;
}

inline std::string datfile_get_string (    const std::string &filename, 
                                        const std::string &id) {

    std::ifstream datfile (filename.c_str());
    if (!datfile) {
        std::cerr     << "Error: could not open datfile '" 
                    << filename << "'" << std::endl;
        exit(1);
    }

    std::string line;
    while (getline (datfile, line)) {
//        cout << line << "stripped = " << strip 3: 0.15 
        
        line = strip_datfile_line (line);

        if (line == id) {
            getline (datfile, line);
            return strip_datfile_line(line);
        }
    }

    if (datfile.eof()) {
        std::cerr     << "Error: could not find value for " 
                    << id << " in file " << filename 
                    << "." << std::endl;
        assert (0 && !"Could not find datfile value");
        exit (0);
    }
    datfile.close();

    return ("");
}

inline int datfile_get_int (const std::string &filename, 
                            const std::string &id) {
    int result = 0;
    std::string result_str = datfile_get_string (filename, id);

    if (result_str == "")
        return 0;

    std::istringstream conversion_stream (result_str);

    conversion_stream >> result;

    return result;
}

inline double datfile_get_double (    const std::string &filename, 
                                    const std::string &id) {
    double result = 0;
    std::string result_str = datfile_get_string (filename, id);

    if (result_str == "")
        return 0;

    std::istringstream conversion_stream (result_str);

    conversion_stream >> result;

    return result;
}

inline RigidBodyDynamics::Math::VectorNd 
        datfile_get_vector (const std::string &filename, 
                                    std::string id, 
                                    int stage_id, 
                                    int node_id) {

    std::vector<double> result_data;
    std::ifstream datfile (filename.c_str());

    if (!datfile) {
        std::cerr << "Error: could not open input file '" << filename << "'" << std::endl;
        exit(1);
    }

    int line_index = 0;
    std::string line;
    while (getline (datfile, line)) {
        line_index ++;
        
        line = strip_datfile_line (line);
//        cout << line << "stripped = " << strip 3: 0.15 

        // if it is an empty line, we can skip it
        if (line.size() == 0)
            continue;

        // if it is the content of a section (i.e. starts with a number), we
        // can skip it
        if (line[0] >= '0' && line[0] < '9')
            continue;

        // otherwise we have some kind of id and we continue parsing
        std::string line_id     = line;
        std::string stage_str     = "*";
        std::string node_str     = "*";
        if (line.find ("(") != std::string::npos)
            line_id     = line.substr (0, line.find ("("));

//        cout << "line = " << line;
//        cout << " line_id = " << line_id;
//        cout << std::endl;

        if (line_id != id) {
            continue;
        }

        if (line.find ("(") != std::string::npos 
                && line.find(")") != std::string::npos) {

            size_t start_bracket, end_bracket;
            start_bracket     = line.find_first_of("(");
            end_bracket     = line.find_first_of(")");
            assert (start_bracket < end_bracket);

            // we have something like "sd(0,3)" which we split up properly
            std::string stage_and_node 
                = line.substr (    start_bracket + 1, 
                                end_bracket - start_bracket - 1);

            if (stage_and_node.find (",") != std::string::npos) {
                stage_str = stage_and_node.substr (    0, 
                                                    stage_and_node.find(","));
                node_str = stage_and_node.substr (    stage_and_node.find(",") + 1, 
                                                    stage_and_node.size());
            }

//            cout << " stage = " << stage_str << " node = " << node_str << std::endl;
        }

        int stage_num;
        int node_num;

        if (stage_str == "*")
            stage_num = stage_id;
        else {
            std::istringstream convert_stream (stage_str);
            convert_stream >> stage_num;
        }
        if (node_str == "*")
            node_num = node_id;
        else {
            std::istringstream convert_stream (node_str);
            convert_stream >> node_num;
        }

//        cout     << "stage num = " << stage_num << " node_num = " 
//                << node_num << std::endl;

        if (stage_id == stage_num && node_id == node_num) {
            // we are at the right section, so lets get the data out!
            while (getline (datfile, line)) {
                line_index ++;
                line = strip_datfile_line (line);

//                cout     << "content line for " << id << ": " 
//                        << line << std::endl;

                // if we are at the end, break out of the current while loop
                if (line.size() == 0)
                    break;

                assert (line.find (":") != std::string::npos);
                line = line.substr(line.find (":") + 1, line.size());
        
//                cout << "value for " << id << ": " << line << std::endl;
                std::istringstream convert_stream (line);
                double value;
                convert_stream >> value;
//                cout     << "line = " << line << " double value = " 
//                         << value << std::endl;

                result_data.push_back (value);

//                cout << line << std::endl;
            }
            break;
        }
    }
//    cout << "Stoped at line " << line_index << std::endl;

    if (datfile.eof()) {
        std::cerr   << "Error! Could not find values for " 
                    << id << "(" << stage_id << "," 
                    << node_id << ") in file " << filename 
                    << std::endl;
        assert (0 && !"Value not in datfile");
        exit (0);
    }

    datfile.close();

    // copy the values to a proper numerical vector
    RigidBodyDynamics::Math::VectorNd result(result_data.size());
    for (unsigned int i = 0; i < result_data.size(); i++) {
        result[i] = result_data[i];
    }

    return result;
}

inline bool run_external_command (const char* command) {
    // backup up the version information
    pid_t pid;

    std::cout   << "[run_external_command] running '" 
                << command << "'" << std::endl;
    std::cout.flush();
    pid = fork();

    int exec_state = 0;

    if ( pid == -1) {
        std::cerr << "[run_external_command] Error: could not fork process!" 
                  << std::endl;
        exit(0);
    } else if (pid == 0) {
        exec_state = execlp (command, command, NULL);
        if (exec_state != -1) {
            std::cout << " success!" << std::endl;
            exit(0);
        } else {
            std::cerr   << "[run_external_command] Error: running command " 
                        << command << ": " << errno 
                        << " (probably file not found)" << std::endl;
            exit (-1);
        }
    }

    int status;
    wait (&status);

    if (status != 0) {
        std::cerr << "[run_external_command] failed!" << std::endl;
        return false;
    }
        
    std::cout << "[run_external_command] success!" << std::endl;
    return true;
}

#endif
