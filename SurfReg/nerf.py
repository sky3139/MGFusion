

import os
from unicodedata import name


def NERF_DATA_TEST():
    path = "whiteroom"
    # complete_kitchen 不好用  green_room grey_white_room
    full_path = "./surfReg -r /dataset/dataset/nerf/{0}/mgfusion.ply -m /home/u20/d2/dataset/mesh/{0}/gt_mesh_culled.ply -t 99".format(
        path)
    print(full_path)
    os.system(full_path)
    full_path = "./hausdorff /dataset/dataset/nerf/{0}/mgfusion.ply  /home/u20/d2/dataset/mesh/{0}/gt_mesh_culled.ply".format(
        path)
    os.system(full_path)


def NERF_TEST():
    task = ["breakfast_room",	"complete_kitchen",	"green_room",	"grey_white_room",
            "kitchen", "morning_apartment", "staircase", "whiteroom",  "1"]
    for path in task:
        # complete_kitchen 不好用  green_room grey_white_room
        full_path = "./surfReg -r /home/u20/d2/dataset/mesh/{0}/neural_rgbd.ply -m /home/u20/d2/dataset/mesh/{0}/gt_mesh_culled.ply -t 99".format(
            path)
        print(full_path)
        os.system(full_path)
        full_path = "./hausdorff /dataset/dataset/nerf/{0}/mgfusion.ply  /home/u20/d2/dataset/mesh/{0}/gt_mesh_culled.ply".format(
            path)
        os.system(full_path)


NERF_TEST()

# 测试 ICL数据集的实验


def ICL_TEST():
    task = ["2", "3", "1"]
    name = "mgfusion"
    for path in task:
        # # complete_kitchen 不好用  green_room grey_white_room
        full_path = "./surfReg -r /home/u20/dataset/paper/{0}/{1}.ply -m /home/u20/dataset/paper/livingroom.ply -t {0}".format(
            path, name)
        print(full_path)
        os.system(full_path)
        # full_path="./hausdorff /home/u20/dataset/paper/{0}/fastfusion.ply  /home/u20/dataset/paper/livingroom.ply".format(path)
        # os.system(full_path)
