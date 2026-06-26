# openarm_description — meshes placeholder
#
# Place your actual robot mesh files here. Supported formats:
#   .stl   — STL binary/ASCII (collision or visual)
#   .dae   — COLLADA (recommended for visual with texture/colour)
#   .obj   — Wavefront OBJ
#
# Recommended structure:
#   meshes/
#     visual/
#       base_link.dae
#       left_link1.dae
#       ...  (one file per link)
#       right_link1.dae
#       ...
#     collision/
#       base_link.stl
#       left_link1.stl
#       ...
#
# Reference them in the URDF like:
#   <mesh filename="package://openarm_description/meshes/visual/base_link.dae"/>
#
# The current URDF (openarm_bimanual.urdf) uses primitive geometry shapes.
# Once you have the mesh files, update each <visual> / <collision> block to
# reference the mesh instead of the primitive geometry.
