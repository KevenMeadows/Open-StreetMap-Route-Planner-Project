#include <optional>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

// Input function to validate inputs

int UserInput(const std::string InputName, float &Input, int Minimum, int Maximum)
{
    bool tryagain = false;

    // do while for validation
    do
    {
        std:: cout << InputName;
        std:: cin >> Input;
        if(std:: cin.fail())
        {
            std:: cin.clear(); // Clear for next input
            std:: cin.ignore(); // Ignore to break output loop
            std:: cout << "\nYour input was invalid.";
            std:: cout << "\nEnter a valid ";
            tryagain = true; // To continue the loop
        }
        else if (Input < Minimum || Input > Maximum)
            {
                std:: cin.clear(); // Clear for next input
                std:: cin.ignore(); // Ignore to break output loop
                std:: cout << "\nInput is either smaller than the minimum or greater than the maximum.";
                std:: cout << "\nEnter a valid ";
                tryagain = true; // Continue loop
            } 
        else
        {
            tryagain = false;
            return Input; // Input good
        }
    }
    while (tryagain || Input < Minimum || Input > Maximum);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    // Declare floats
    float start_x = 0.0;
    float start_y = 0.0;
    float end_x = 0.0;
    float end_y = 0.0;

    // Ask for input
    std:: cout << "\nEnter your starting x and y values and your ending x and y values (x and y values should be between 0 and 100)";
    std:: cout << "\n\n";
    // Get inputs and send to input validation function
    // String for prompt, each x and y for the inputs, and both min and max
    // Made an assumption for the min and max values (didn't see any specification for distances)
    UserInput("Starting X Value: ", start_x, 0, 100); 
    UserInput("Starting Y Value: ", start_y, 0, 100);
    UserInput("Ending X Value: ", end_x, 0, 100);
    UserInput("Ending Y Value: ", end_y, 0, 100);
    
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
