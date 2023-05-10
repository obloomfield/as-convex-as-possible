# GETS THE BOUNDING VERTICES OF THE MESH.

import bpy

def print(data):
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type == 'CONSOLE':
                override = {'window': window, 'screen': screen, 'area': area}
                bpy.ops.console.scrollback_append(override, text=str(data), type="OUTPUT")

obj = bpy.context.object

if obj and obj.type == 'MESH':
  print("Object name: " + obj.name)
  print("Plane mesh vertices:")
  vertices = [obj.matrix_world @ v.co for v in obj.data.vertices]

  # Print the world space coordinates of the vertices
  vs = []
  for v in vertices:
      vs.append(v)
  print(vs)
  
else:
    print("Selected object is not a mesh.")


    #
    # [Vector((-1.4538562297821045, -0.8824989199638367, 0.8811805248260498)), Vector((0.4524270296096802, -0.9855093359947205, 0.284964919090271)), Vector((-1.3615524768829346, 1.1147454977035522, 0.8312325477600098)), Vector((0.5447308421134949, 1.0117350816726685, 0.23501692712306976))]