import matplotlib.pyplot as plt
import numpy as np
import time
import json
import pandas as pd
import math

plt.ion()


def make_polygon(sides, x_centre_position, y_centre_position, radius, rotate):
    """Draw an n-sided regular polygon.

    Args:
            sides (int): Number of polygon sides.
            x_centre_position, y_centre_position (float): Coordinates of center point.
            radius (int): Radius.
            color (int): RGB565 color value.
            rotate (Optional float): Rotation in degrees relative to origin.
        Note:
            The center point is the center of the x_centre_position,y_centre_position pixel.
            Since pixels are not divisible, the radius is integer rounded
            up to complete on a full pixel.  Therefore diameter = 2 x r + 1.
    """
    x_coordinates = []
    y_coordinates = []
    theta = rotate + math.pi
    n = sides + 1
    for s in range(n):
        t = 2.0 * math.pi * s / sides + theta
        x_coordinates.append(radius * math.cos(t) + x_centre_position)
        y_coordinates.append(radius * math.sin(t) + y_centre_position)

    return x_coordinates, y_coordinates


def make_pointer(endx, endy, orientation):
    return make_polygon(3, endx, endy, 0.07, orientation)


def generate_direction_ray(x_start, y_start, angle):
    x_data = []
    y_data = []
    y_end = y_start + 0.7 * math.sin(angle)
    x_end = x_start + 0.7 * math.cos(angle)
    
    x_data = [x_start,x_end]
    y_data = [y_start,y_end]

    x_polygon_pointer, y_polygon_pointer = make_pointer(
        x_end, y_end, angle + math.pi
    )
    x_data.extend(x_polygon_pointer)
    y_data.extend(y_polygon_pointer)
    x_data.append(np.NaN)
    y_data.append(np.NaN)
    
    return x_data,y_data

def generate_drawing_object_points(x, y, orientation, polygon_sides, radius, offset=0):
    x_polygon, y_polygon = make_polygon(polygon_sides, x, y, radius, orientation)

    xdata = x_polygon
    ydata = y_polygon

    xdata.append(np.NaN)
    ydata.append(np.NaN)
    # find the end point
    x_line, y_line = generate_direction_ray(x,y, orientation - offset)


    xdata.extend(x_line)
    ydata.extend(y_line)


    return xdata, ydata


def generate_drawing_robot_points(x, y, orientation):
    return generate_drawing_object_points(x, y, orientation, 5, 0.5)


def generate_drawing_carriers_points(poses_list):
    xdata = []
    ydata = []
    for pose in poses_list:
        pose_df = pd.DataFrame(pose, index=[0])

        x_generated, y_generated = generate_drawing_object_points(
            pose_df["x"].astype(float),
            pose_df["y"].astype(float),
            pose_df["theta"].astype(float) + math.pi / 4,
            4,
            0.8,
            math.pi / 4,
        )
        xdata.extend(x_generated)
        ydata.extend(y_generated)

    return xdata, ydata


def read_json(path, record_path_option):
    with open(path, "r") as file_robot_poses:
        data_robot_poses = json.load(file_robot_poses)
    return pd.json_normalize(data_robot_poses, record_path=[record_path_option])


def initialize_plot(min_axis_length, max_axis_length):
    figure, ax = plt.subplots()
    (lines,) = ax.plot([], [], "-")
    # Autoscale on unknown axis and known lims on the other
    ax.set_autoscaley_on(True)
    ax.set_xlim(min_axis_length, max_axis_length)
    ax.set_ylim(min_axis_length, max_axis_length)
    # Other stuff
    ax.grid()
    return figure, ax, lines


def draw_data_snap_shot(x_data, y_data, lines, ax, figure):
    lines.set_xdata(x_data)
    lines.set_ydata(y_data)
    # Need both of these in order to rescale
    ax.relim()
    ax.autoscale_view()
    # We need to draw *and* flush
    figure.canvas.draw()
    figure.canvas.flush_events()


def main():

    df_robot_poses = read_json(
        "C:/Users/bader/OneDrive/Documents/RoboticChallenge/main/robot_poses.json",
        "robotPose",
    )

    df_detection_output = read_json(
        "C:/Users/bader/OneDrive/Documents/RoboticChallenge/main/detections_output.json",
        "detections",
    )

    min_axis_length = 0
    max_axis_length = 25
    figure, ax, lines = initialize_plot(min_axis_length, max_axis_length)

    for x, y, orientation, poses_list in zip(
        df_robot_poses["x"].astype(float),
        df_robot_poses["y"].astype(float),
        df_robot_poses["theta"].astype(float),
        df_detection_output["poses"],
    ):
        x_data_robot, y_data_robot = generate_drawing_robot_points(x, y, orientation)
        x_data_carriers, y_data_carriers = generate_drawing_carriers_points(poses_list)

        x_data = x_data_robot
        y_data = y_data_robot
        x_data.extend(x_data_carriers)
        y_data.extend(y_data_carriers)

        draw_data_snap_shot(x_data, y_data, lines, ax, figure)


if __name__ == "__main__":
    main()
