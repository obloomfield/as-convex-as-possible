import bpy
import os
import random
from mathutils import Color

# Change this path to the folder containing your .obj files
folder_path = "/Users/obloomfield/Coding/cs224/final/as-convex-as-possible/fragments/keep/out"

# Helper function to set a random color for the default material
def set_random_color(obj):
    materials = obj.data.materials
    if len(materials) == 0:
        material = bpy.data.materials.new(name="Default Material")
        material.use_nodes = True
        obj.data.materials.append(material)
    else:
        material = materials[0]
    
    bsdf_node = material.node_tree.nodes["Principled BSDF"]
    
    random_color = Color((random.random(), random.random(), random.random()))
    bsdf_node.inputs["Base Color"].default_value = (random_color.r, random_color.g, random_color.b, 1)


# Add a plane with specified scale and location
bpy.ops.mesh.primitive_plane_add(size=100, enter_editmode=False, align='WORLD', location=(0, 0, -2))
plane = bpy.context.active_object

# Set passive rigidbody physics with mesh collision for the plane
bpy.ops.rigidbody.object_add(type='PASSIVE')
plane.rigid_body.collision_shape = 'MESH'
# Import all .obj files and apply a random color to the default material of each
for file_name in os.listdir(folder_path):
    if file_name.endswith(".obj"):
        file_path = os.path.join(folder_path, file_name)
        bpy.ops.import_scene.obj(filepath=file_path)
        
        imported_obj = bpy.context.selected_objects[0]
        set_random_color(imported_obj)
        # Add Rigidbody Physics Properties to the imported object
        bpy.context.view_layer.objects.active = imported_obj
        imported_obj.select_set(True)
        bpy.ops.rigidbody.object_add()
        imported_obj.rigid_body.collision_shape = 'MESH'