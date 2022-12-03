#include <librealsense2/rs.hpp>
#include <iostream>
#include "example.hpp"

int main(int argc, char* argv[]){
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    window app(1280, 720, "RealSense Capture Example");

    rs2::colorizer color_map;
    rs2::rates_printer printer;

    rs2::pipeline pipe;

    pipe.start();

    while(app)
    {
        rs2::frameset data = pipe.wait_for_frames().apply_filter(printer).apply_filter(color_map);
        app.show(data);
    }

    return EXIT_SUCCESS;
}
// catch (const rs2::error & e)
// {
//     std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception& e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }