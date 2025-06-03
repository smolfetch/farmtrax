#include <iostream>
#include <thread>

#include "concord/concord.hpp"
#include "geoson/geoson.hpp"
#include "geotiv/geotiv.hpp"

#include "rerun/recording_stream.hpp"

#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "rerun.hpp"

#include "farmtrax/utils/visualize.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.80182312011719};

    concord::Polygon poly;
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
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

    for (auto &p : poly.getPoints()) {
        std::cout << "x: " << p.enu.x << ", y: " << p.enu.y << ", z: " << p.enu.z << "\n";
    }

    farmtrax::Field field(poly, 0.1, datum, true, 0.5);
    field.add_noise();

    // geotiv::Layer layer;
    // layer.grid = field.get_grid(0);
    // layer.samplesPerPixel = 1;
    // layer.planarConfig = 1;
    // geotiv::RasterCollection rc;
    // rc.crs = concord::CRS::WGS;
    // rc.datum = concord::Datum();
    // rc.heading = concord::Euler{0.0, 0.0, 0.0};
    // rc.resolution = 0.1;
    // rc.layers.push_back(layer);
    //
    // std::filesystem::path outPath = "output.tif";
    // geotiv::WriteRasterCollection(rc, outPath);

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 2;

    auto part_cnt = field.get_parts().size();
    std::cout << "Part count: " << part_cnt << "\n";

    farmtrax::visualize::show_field(field, rec);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (size_t f = 0; f < field.get_parts().size(); f++) {

        auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[f]);
        farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
        std::cout << "here" << std::endl;
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec);

        std::this_thread::sleep_for(std::chrono::seconds(10));

        num_machines = 4;
        divy.set_machine_count(num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec);

        auto &res = divy.result();
        for (std::size_t m = 0; m < num_machines; ++m) {
            if (res.swaths_per_machine.at(m).empty()) {
                std::cout << "Machine " << m << " has no swaths assigned\n";
                continue;
            }

            // Create Nety instance directly from swaths
            farmtrax::Nety nety(res.swaths_per_machine.at(m));
            auto path = nety.field_traversal();

            std::cout << "Machine " << m << " path has " << path.size() << " vertices\n";

            // Visualize the optimized swath tour with connections
            farmtrax::visualize::show_swath_tour(nety, path, rec, m);

            // … convert vertex descriptors back to ENU coords or swath indices …
        }
    }

    return 0;
}
