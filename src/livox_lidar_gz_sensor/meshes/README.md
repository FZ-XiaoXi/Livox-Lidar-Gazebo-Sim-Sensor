# Livox Mesh Files

Put the visual mesh files used by the Livox xacro macros in this directory.
The common Gazebo formats are .dae, .stl, .obj, .gltf, and .glb.

The `livox_mid70_link` xacro parameter follows the same package path style as
the CSV parameter:

<xacro:livox_mid70_link
  link_name="livox_link"
  mesh_file="$(find livox_lidar_gz_sensor)/meshes/mid70.dae"
  mesh_scale="1 1 1"/>

The macro derives the sensor names automatically:

```text
link_name=livox_link
sensor name=livox_link_sensor
frame_id=livox_link_frame
```
