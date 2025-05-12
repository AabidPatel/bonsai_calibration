import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import argparse
import numpy as np
import cv2
import os

from sensor_msgs.msg import CompressedImage, Imu, Image
from cv_bridge import CvBridge

parser = argparse.ArgumentParser()
parser.add_argument('-r', '--rosbag', help='Path to the rosbag files')
args = parser.parse_args()

cam_recordings = args.rosbag

### set these two paths
#PATH2MCAP =    "/big_disk/hawk_calib_recs/JNCJ0881/rosbag2_2025_01_09-03_23_00/rosbag2_2025_01_09-03_23_00_0.mcap"
#PATH2DATASET = "/big_disk/hawk_calib_recs/JNCJ0881/0881-euroc/1a-1"
ROS2BAG_TYPE = "mcap" # "mcap" or "sqlite3"
LEFT_CAMERA_COMPRESSED_TOPIC = "/camera_front_center/left/image_mono_raw"
RIGHT_CAMERA_COMPRESSED_TOPIC = "/camera_front_center/right/image_mono_raw"
IMU_TOPIC = "/camera_front_center/imu"
###

bridge = CvBridge()

def uncompress_img(msg: CompressedImage):
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id=ROS2BAG_TYPE),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader

def convert(PATH2MCAP, PATH2DATASET):

    print(f"Converting MCAP at {PATH2MCAP} and saving to {PATH2DATASET}")

    # paths for the datset
    PATH2DATASET_MAV = PATH2DATASET + '/mav0'
    path2cam0 = PATH2DATASET_MAV + '/cam0'
    path2cam1 = PATH2DATASET_MAV + '/cam1'
    path2cam0data = path2cam0 + '/data'
    path2cam1data = path2cam1 + '/data'
    path2imu = PATH2DATASET_MAV + '/imu0'


    # create a folder strucure
    if not os.path.exists(PATH2DATASET_MAV):
        os.makedirs(PATH2DATASET_MAV)
    if not os.path.exists(path2cam0):
        os.makedirs(path2cam0)
        os.makedirs(path2cam0data)
    if not os.path.exists(path2cam1):
        os.makedirs(path2cam1)
        os.makedirs(path2cam1data)
    if not os.path.exists(path2imu):
        os.makedirs(path2imu)


    # create imu file
    path2imu_txt = path2imu + '/data.csv'
    traj_txt = open(path2imu_txt, 'w')
    traj_txt.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
    traj_txt.close()

    # cam0
    path2cam0_times_txt = path2cam0 + '/data.csv'
    cam0_txt = open(path2cam0_times_txt, 'w')
    cam0_txt.write('#timestamp [ns],filename\n')
    cam0_txt.close()

    # cam1
    path2cam1_times_txt = path2cam1 + '/data.csv'
    cam1_txt = open(path2cam1_times_txt, 'w')
    cam1_txt.write('#timestamp [ns],filename\n')
    cam1_txt.close()

    counter = 0

    for topic, msg, timestamp in read_messages(PATH2MCAP):

        # debug only: counter for unpacking just part of the dataset
        counter += 1
        # if counter == 20000:
        #     break

        if topic == LEFT_CAMERA_COMPRESSED_TOPIC:
            stamp = msg.header.stamp
            name_no_ext = str(stamp.sec) + str(stamp.nanosec).zfill(9)
            name = name_no_ext + '.png'
            # img_cv = uncompress_img(msg)
            # img_cv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            img_cv = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')


            # # Convert the image to grayscale
            # img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

            # Resize the grayscale image to half its original size
            # width = int(img_cv.shape[1] * 0.5)
            # height = int(img_cv.shape[0] * 0.5)
            # dim = (width, height)
            # img_cv = cv2.resize(img_cv, dim, interpolation=cv2.INTER_AREA)

            cv2.imwrite(path2cam0data +'/' + name, img_cv)  # save img

            with open(path2cam0_times_txt, 'a') as f:
                # #timestamp [ns],filename
                line = name_no_ext + ',' + name + '\n'
                f.write(line)

        elif topic == RIGHT_CAMERA_COMPRESSED_TOPIC:
            stamp = msg.header.stamp
            name_no_ext = str(stamp.sec) + str(stamp.nanosec).zfill(9)
            name = name_no_ext + '.png'
            # img_cv = uncompress_img(msg)
            # img_cv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            img_cv = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # # Convert the image to grayscale
            # img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

            # # Resize the grayscale image to half its original size
            # width = int(img_cv.shape[1] * 0.5)
            # height = int(img_cv.shape[0] * 0.5)
            # dim = (width, height)
            # img_cv = cv2.resize(img_cv, dim, interpolation=cv2.INTER_AREA)

            cv2.imwrite(path2cam1data + '/' + name, img_cv)

            with open(path2cam1_times_txt, 'a') as f:
                # #timestamp [ns],filename
                line = name_no_ext + ',' + name + '\n'
                f.write(line)

        elif topic == IMU_TOPIC and isinstance(msg, Imu):
            stamp = msg.header.stamp
            accel = msg.linear_acceleration
            gyro = msg.angular_velocity

            with open(path2imu_txt, 'a') as f:
                # # timestamp[ns] w.x w.y w.z a.x a.y a.z
                line = str(stamp.sec) + str(stamp.nanosec).zfill(9)
                line += (
                    ','
                    + '{:.10f}'.format(gyro.x)
                    + ','
                    + '{:.10f}'.format(gyro.y)
                    + ','
                    + '{:.10f}'.format(gyro.z)
                )
                line += (
                    ','
                    + '{:.10f}'.format(accel.x)
                    + ','
                    + '{:.10f}'.format(accel.y)
                    + ','
                    + '{:.10f}'.format(accel.z)
                )

                f.write(line)
                f.write('\n')


def main():

    a_id = 1
    b_id = 1

    try:
        for item in os.listdir(cam_recordings):
            item_path = os.path.join(cam_recordings, item)
            if os.path.isdir(item_path):
                for rosbag in os.listdir(item_path):
                    if rosbag.endswith(".mcap"):
                        PATH2MCAP = os.path.join(item_path, rosbag)
                        size_in_bytes = os.path.getsize(PATH2MCAP)
                        size_in_GB = size_in_bytes / (1024 ** 3)
                        if size_in_GB < 1.6:
                            PATH2DATASET = os.path.join(cam_recordings, f"euroc/1a_{a_id}")
                            convert(PATH2MCAP, PATH2DATASET)
                            print("Converted mcaps for Step 1a")
                            a_id += 1
                            break
                        elif size_in_GB > 2:
                            PATH2DATASET = os.path.join(cam_recordings, f"euroc/1b_{b_id}")
                            convert(PATH2MCAP, PATH2DATASET)
                            print("Converted mcaps for Step 1b")
                            b_id += 1
                            break


    except FileNotFoundError as fnf_error:
        print(fnf_error)

    

    print("finished")


if __name__ == "__main__":
    main()

