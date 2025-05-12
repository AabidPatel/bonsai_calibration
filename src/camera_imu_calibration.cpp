#include <basalt/optimization/spline_optimize.h>
#include <basalt/calibration/cam_imu_calib.h>
#include <CLI/CLI.hpp>

int main(int argc, char **argv) {
  std::string dataset_path;
  std::string dataset_type;
  std::string aprilgrid_path;
  std::string result_path;
  std::string cache_dataset_name = "calib-cam-imu";
  int skip_images = 1;

  double accel_noise_std = 0.04;
  double gyro_noise_std = 0.0025;
  double accel_bias_std = 0.02;
  double gyro_bias_std = 0.000105;

  CLI::App app{"Non-GUI Calibrate IMU"};

  app.add_option("--dataset-path", dataset_path, "Path to dataset")->required();
  app.add_option("--result-path", result_path, "Path to result folder")->required();
  app.add_option("--dataset-type", dataset_type, "Dataset type (euroc, bag)")->required();
  app.add_option("--aprilgrid", aprilgrid_path, "Path to Aprilgrid config file)")->required();
  app.add_option("--gyro-noise-std", gyro_noise_std, "Gyroscope noise std");
  app.add_option("--accel-noise-std", accel_noise_std, "Accelerometer noise std");
  app.add_option("--gyro-bias-std", gyro_bias_std, "Gyroscope bias random walk std");
  app.add_option("--accel-bias-std", accel_bias_std, "Accelerometer bias random walk std");
  app.add_option("--cache-name", cache_dataset_name, "Name to save cached files");
  app.add_option("--skip-images", skip_images, "Number of images to skip");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  basalt::CamImuCalib cv(dataset_path, dataset_type, aprilgrid_path, result_path, cache_dataset_name, skip_images,
                         {accel_noise_std, gyro_noise_std, accel_bias_std, gyro_bias_std}, false);

  cv.loadDataset();
  cv.detectCorners();
  cv.initCamPoses();
  cv.initCamImuTransform();
  cv.initOptimization();

  bool opt_until_convg = true;

  // double mean_reprojection = cv.getMeanReprojectionError();

  while (opt_until_convg) {
    	bool converged = cv.optimizeWithParam(true);
        if (converged) opt_until_convg = false;
	}

  cv.setOptCamTimeOffset(true); 
  cv.setOptImuScale(true); 

  opt_until_convg = true;

    while (opt_until_convg) {
    	bool converged = cv.optimizeWithParam(true);
        if (converged) opt_until_convg = false;
	} 

  cv.saveCalib();

  std::cout << "Camera-IMU calibration completed successfully!" << std::endl;
  return 0;
}