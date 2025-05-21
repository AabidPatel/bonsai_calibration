#include <basalt/calibration/cam_calib.h>
#include <CLI/CLI.hpp>
#include <iostream>
#include <chrono>
#include <iostream>

void opt_until_converge(auto& cv, int timeout_seconds = 30, int max_retries = 3) {

    double current_thresh = cv.getStopThresh();
    std::cout << "Current Stop Threshold: " << current_thresh << std::endl;
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        std::cout << "Optimization attempt " << attempt << "...\n";

        auto start_time = std::chrono::steady_clock::now();

        if (attempt == 1){
            double desired_thresh = 1e-08;
            cv.setStopThresh(desired_thresh);
            std::cout << "Setting Stop Threshold: " << desired_thresh << "\n";
        }
        else if (attempt == 2){
            double desired_thresh = 5.0e-05;
            cv.setStopThresh(desired_thresh);
            std::cout << "Setting Stop Threshold: " << desired_thresh << "\n";
        }
        else if (attempt > 2){
            double desired_thresh = 5.0e-03;
            cv.setStopThresh(desired_thresh);
            std::cout << "Setting Stop Threshold: " << desired_thresh << "\n";
        }

        while (true) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
            // std::cout << "Elapsed time: " << elapsed.count() << " seconds\n";
  
            if (elapsed.count() >= timeout_seconds) {
                std::cout << "Attempt " << attempt << " timed out after " << timeout_seconds << " seconds.\n";
                break;
            }
  
            bool converged = cv.optimizeWithParam(false);
            if (converged) {
                double mean_reproj_error = cv.getMeanReprojectionError();
                std::cout << "Mean reprojection error: " << mean_reproj_error << "\n";
                if (mean_reproj_error > 0.12) {
                    throw std::runtime_error("Mean reprojection error is too high: " + std::to_string(mean_reproj_error));
                    // std::cout << "Mean reprojection error is too high: " << mean_reproj_error << "\n";
                    // break;
                }
                else {
                    std::cout << "Optimization converged successfully on attempt " << attempt << ".\n";
                    return;
                }
            }
        }
        if (attempt >= max_retries) {
            std::cout << "Optimization failed to converge after " << max_retries << " attempts.\n";
        }
    }
}

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

    while (true){
        calib.loadDataset();
        calib.detectCorners();
        calib.initCamIntrinsics();
        calib.initCamPoses();
        calib.initCamExtrinsics();
        calib.initOptimization();
        try {
            opt_until_converge(calib);
        } catch (const std::exception& e) {
            std::cerr << "Calibration failed: " << e.what() << std::endl;
            std::cout << "Press 'y' to run calibration again, or 'n' to cancel and record again: ";
            std::string input;
            std::getline(std::cin, input);
            if (input == "y" || input == "Y") {
                continue; // Run calibration again
            } else {
                std::cout << "Calibration cancelled. Please record again." << std::endl;
                return 1;
            }
        }
        calib.saveCalib();
        
        std::cout << "Camera calibration completed successfully!" << std::endl;
        return 0;
    }
}
