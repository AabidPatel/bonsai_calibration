#include <basalt/calibration/cam_calib.h>
#include <CLI/CLI.hpp>
#include <iostream>

int main(int argc, char **argv) {
    std::string dataset_path;
    std::string dataset_type;
    std::string aprilgrid_path;
    std::string result_path;
    std::vector<std::string> cam_types;
    int skip_images = 1;

    CLI::App app{"Non-GUI Camera Calibration"};

    app.add_option("--dataset-path", dataset_path, "Path to dataset")->required();
    app.add_option("--result-path", result_path, "Path to result folder")->required();
    app.add_option("--dataset-type", dataset_type, "Dataset type (euroc, bag)")->required();
    app.add_option("--aprilgrid", aprilgrid_path, "Path to Aprilgrid config file")->required();
    app.add_option("--cam-types", cam_types, "Type of cameras (eucm, ds, kb4, pinhole)")->required();
    app.add_option("--skip-images", skip_images, "Number of images to skip");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    basalt::CamCalib calib(dataset_path, dataset_type, aprilgrid_path, result_path, "calib-cam", skip_images, cam_types, false);

    calib.loadDataset();
    calib.detectCorners();
    calib.initCamIntrinsics();
    calib.initCamPoses();
    calib.initCamExtrinsics();
    calib.initOptimization();

    bool opt_until_convg = true;

    // Range-based for loop
    while (opt_until_convg) {
    	bool converged = calib.optimizeWithParam(true);
            if (converged) opt_until_convg = false;
	}
    
    calib.saveCalib();

    std::cout << "Camera calibration completed successfully!" << std::endl;
    return 0;
}
