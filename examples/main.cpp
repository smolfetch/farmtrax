#include <iostream>

#include "geoson/parser.hpp"
#include "geoson/writter.hpp"
#include "rerun/recording_stream.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");

        std::cout << fc << "\n";

        fc.datum.lat += 5.1;
        std::cout << "new datum is: " << fc.datum.lat << ", " << fc.datum.lon << ", " << fc.datum.alt << "\n";

        geoson::WriteFeatureCollection(fc, "misc/field4.geojson");
        std::cout << "Saved modified GeoJSON to misc/field4_modified.geojson\n";
    } catch (std::exception &e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
