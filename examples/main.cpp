#include <iomanip>
#include <iostream>
#include <thread>

#include "concord/concord.hpp"
#include "geoson/geoson.hpp"
#include "geotiv/geotiv.hpp"

#include "rerun/recording_stream.hpp"

#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "rerun.hpp"
#include "thread"

#include "farmtrax/utils/visualize.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    concord::Polygon poly;
    concord::Datum datum; // Will be set from GeoJSON
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
        datum = fc.datum; // Use the datum from the GeoJSON file
        for (auto &f : fc.features) {
            if (std::get_if<concord::Polygon>(&f.geometry)) {
                poly = std::get<concord::Polygon>(f.geometry);
                break;
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Failed to parse geojson: " << e.what() << "\n";
        return 1;
    }

    farmtrax::Field field(poly, 0.1, datum, true, 50000.0);

    field.add_noise();

    farmtrax::visualize::show_field(field, rec);

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 2;

    std::this_thread::sleep_for(std::chrono::seconds(5));
    farmtrax::visualize::show_field(field, rec);

    return 0;
}
