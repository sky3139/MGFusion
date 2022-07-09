
complete_kitchen
./surfReg -r /dataset/dataset/nerf/breakfast_room/mgfusion.ply -m /home/u20/d2/dataset/mesh/breakfast_room/gt_mesh_culled.ply -t 99
./hausdorff /dataset/dataset/nerf/breakfast_room/mgfusion.ply  /home/u20/d2/dataset/mesh/breakfast_room/gt_mesh_culled.ply 

./surfReg -r /dataset/dataset/nerf/complete_kitchen/mgfusion.ply -m /home/u20/d2/dataset/mesh/breakfast_room/gt_mesh_culled.ply -t 99
./hausdorff /dataset/dataset/nerf/breakfast_room/mgfusion.ply  /home/u20/d2/dataset/mesh/breakfast_room/gt_mesh_culled.ply 
CloudCompare  /dataset/dataset/nerf/complete_kitchen/mgfusion.ply  /home/u20/d2/dataset/mesh/complete_kitchen/gt_mesh_culled.ply 

# ./surfReg -r /dataset/dataset/nerf/breakfast_room/mgfusion.ply -m /home/u20/d2/dataset/mesh/breakfast_room/gt_mesh_culled.ply -t 99
