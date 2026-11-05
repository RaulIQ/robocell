#include <karto_sdk/Mapper.h>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Graph export requires posegraph and output paths\n";
        return 1;
    }
    karto::Mapper mapper;
    mapper.LoadFromFile(argv[1]);
    std::ofstream output(argv[2]);
    if (!output) {
        std::cerr << "Graph export cannot open its output path\n";
        return 1;
    }
    output << std::setprecision(17) << "{\"nodes\":[";
    bool comma = false;
    for (const auto& sensor : mapper.GetGraph()->GetVertices()) {
        for (const auto& vertex : sensor.second) {
            const auto* scan = vertex.second->GetObject();
            const auto& pose = scan->GetCorrectedPose();
            if (comma) {
                output << ',';
            }
            comma = true;
            output << '[' << scan->GetUniqueId() << ',' << scan->GetTime()
                      << ',' << pose.GetX() << ',' << pose.GetY() << ','
                      << pose.GetHeading() << ']';
        }
    }
    output << "],\"edges\":[";
    comma = false;
    for (const auto* edge : mapper.GetGraph()->GetEdges()) {
        if (comma) {
            output << ',';
        }
        comma = true;
        output << '[' << edge->GetSource()->GetObject()->GetUniqueId()
                  << ',' << edge->GetTarget()->GetObject()->GetUniqueId() << ']';
    }
    output << "]}\n";
    return 0;
}
